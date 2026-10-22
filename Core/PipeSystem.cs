using System.Numerics;
namespace FluidSim.Core;

public sealed record Pipe(string Name, int Start, int Count, float Dx, float Area, float X1, float Y1, float X2, float Y2)
{
    internal State LeftFlux,RightFlux;
    public float FrictionFactor;
    internal float FrictionRate => FrictionFactor/(2*MathF.Sqrt(4*Area/MathF.PI));
}

public sealed class PipeSystem
{
    public readonly List<Pipe> Pipes = [];
    public float[] Rho = [], Momentum = [], Energy = [], Air = [];
    private float[] fr = [], fm = [], fe = [], fa = [];
    public float MomentumDamping = .3f, ThermalRelaxation = .02f;
    public bool UseSimd = true;
    public long Repairs { get; private set; }
    public Pipe Add(string name, float length, float area, float x1, float y1, float x2, float y2, int count = 10)
    {
        var pipe = new Pipe(name,Rho.Length,count,length/count,area,x1,y1,x2,y2);
        Pipes.Add(pipe);
        int n = Rho.Length + count;
        Array.Resize(ref Rho,n); Array.Resize(ref Momentum,n); Array.Resize(ref Energy,n); Array.Resize(ref Air,n);
        fr = new float[n+Pipes.Count]; fm = new float[fr.Length]; fe = new float[fr.Length]; fa = new float[fr.Length];
        for(int i=pipe.Start;i<n;i++) Set(i,Gas.Ambient);
        return pipe;
    }
    public State Get(int i) => new(Rho[i],Momentum[i],Energy[i],Air[i]);
    public void Set(int i, State s) { Rho[i]=s.Rho; Momentum[i]=s.M; Energy[i]=s.E; Air[i]=s.Air; }
    public float MaxWaveSpeed()
    {
        float s=1; int i=0,w=Vector<float>.Count;
        if(UseSimd && Vector.IsHardwareAccelerated)
        {
            var maximum=Vector<float>.One;
            for(;i<=Rho.Length-w;i+=w)
            {
                var rho=new Vector<float>(Rho,i); var m=new Vector<float>(Momentum,i);
                var u=m/rho;
                var pressure=new Vector<float>(Gas.Gamma-1)*(new Vector<float>(Energy,i)-new Vector<float>(.5f)*m*u);
                var c=Vector.SquareRoot(new Vector<float>(Gas.Gamma)*Vector.Max(Vector<float>.One,pressure)/rho);
                maximum=Vector.Max(maximum,Vector.Abs(u)+c);
            }
            for(int lane=0;lane<w;lane++) s=MathF.Max(s,maximum[lane]);
        }
        for(;i<Rho.Length;i++) { var q=Get(i); s=MathF.Max(s,MathF.Abs(q.U)+q.C); }
        return s;
    }
    public void Advance(float dt, BoundarySystem boundaries)
    {
        int offset=0;
        int w=Vector<float>.Count;
        bool simd=UseSimd && Vector.IsHardwareAccelerated;
        // Compute transcendental damping coefficients once per substep, not once per cell.
        float damping=MathF.Exp(-MomentumDamping*dt),relaxation=MathF.Min(1,ThermalRelaxation*dt);
        foreach(var p in Pipes)
        {
            float drag=p.FrictionFactor>0?p.FrictionRate*dt:0;
            Store(offset,boundaries.Flux(p,false));
            int face=1;
            // Never span segment boundaries: all lanes are interior faces of this pipe.
            if(simd) for(;face+w<=p.Count;face+=w) InteriorFlux(p.Start+face-1,offset+face);
            // Fluxes only read old state: an overlapping final vector is safe and avoids
            // seven scalar faces on the common 16-cell runner/header segments (AVX2).
            if(simd && p.Count> w && p.Count-face>=3)
            {
                InteriorFlux(p.Start+p.Count-w-1,offset+p.Count-w);
                face=p.Count;
            }
            for(;face<p.Count;face++) Store(offset+face,State.Rusanov(Get(p.Start+face-1),Get(p.Start+face)));
            Store(offset+p.Count,boundaries.Flux(p,true));
            float k=dt/p.Dx; int jv=0;
            if(simd) for(;jv<=p.Count-w;jv+=w)
            {
                int i=p.Start+jv,f=offset+jv;
                var r=Updated(Rho,fr,i,f,k); var m=Updated(Momentum,fm,i,f,k);
                var e=Updated(Energy,fe,i,f,k); var a=Updated(Air,fa,i,f,k);
                EnsureFinite(r,p.Name); EnsureFinite(m,p.Name); EnsureFinite(e,p.Name); EnsureFinite(a,p.Name);
                var floor=new Vector<float>(.00001f); var safeR=Vector.Max(floor,r);
                var minE=new Vector<float>(.5f)*m*m/safeR+new Vector<float>(10);
                var repaired=Vector.LessThan(r,floor)|Vector.LessThan(e,minE);
                if(!Vector.EqualsAll(repaired,Vector<int>.Zero))
                    for(int lane=0;lane<w;lane++) if(repaired[lane]!=0) Repairs++;
                e=Vector.Max(minE,e); a=Vector.Min(Vector.Max(Vector<float>.Zero,a),safeR);
                m*=new Vector<float>(damping);
                if(drag>0) m/=Vector<float>.One+new Vector<float>(drag)*Vector.Abs(m/safeR);
                e+=(safeR*new Vector<float>(Gas.Cv*Gas.AmbientTemperature)+new Vector<float>(.5f)*m*m/safeR-e)*new Vector<float>(relaxation);
                safeR.CopyTo(Rho,i); m.CopyTo(Momentum,i); e.CopyTo(Energy,i); a.CopyTo(Air,i);
            }
            int scalarStart=jv;
            for(;jv<p.Count;jv++)
            {
                int i=p.Start+jv,f=offset+jv;
                Rho[i]-=k*(fr[f+1]-fr[f]); Momentum[i]-=k*(fm[f+1]-fm[f]);
                Energy[i]-=k*(fe[f+1]-fe[f]); Air[i]-=k*(fa[f+1]-fa[f]);
            }
            // Scalar remainder and scalar-reference path share the same safeguards.
            for(int j=scalarStart;j<p.Count;j++)
            {
                int i=p.Start+j;
                if(!float.IsFinite(Energy[i]) || !float.IsFinite(Rho[i]) || !float.IsFinite(Momentum[i]) || !float.IsFinite(Air[i]))
                    throw new ArithmeticException($"Nonfinite pipe state in {p.Name}, cell {j}");
                float r=MathF.Max(.00001f,Rho[i]);
                float kinetic=.5f*Momentum[i]*Momentum[i]/r;
                if(r!=Rho[i] || Energy[i]<kinetic+10) Repairs++;
                Rho[i]=r; Energy[i]=MathF.Max(kinetic+10,Energy[i]); Air[i]=Math.Clamp(Air[i],0,r);
                Momentum[i]*=damping;
                if(drag>0) Momentum[i]/=1+drag*MathF.Abs(Momentum[i]/r);
                Energy[i]+=(r*Gas.Cv*Gas.AmbientTemperature+.5f*Momentum[i]*Momentum[i]/r-Energy[i])*relaxation;
            }
            offset+=p.Count+1;
        }
    }
    private static Vector<float> Updated(float[] u,float[] f,int i,int j,float k) =>
        new Vector<float>(u,i)-new Vector<float>(k)*(new Vector<float>(f,j+1)-new Vector<float>(f,j));
    private static void EnsureFinite(Vector<float> value,string pipe)
    {
        if(!Vector.LessThanOrEqualAll(Vector.Abs(value),new Vector<float>(float.MaxValue)))
            throw new ArithmeticException($"Nonfinite pipe state in {pipe}");
    }
    private void InteriorFlux(int i,int face)
    {
        var rl=new Vector<float>(Rho,i); var rr=new Vector<float>(Rho,i+1);
        var ml=new Vector<float>(Momentum,i); var mr=new Vector<float>(Momentum,i+1);
        var el=new Vector<float>(Energy,i); var er=new Vector<float>(Energy,i+1);
        var al=new Vector<float>(Air,i); var ar=new Vector<float>(Air,i+1);
        var ul=ml/rl; var ur=mr/rr;
        var half=new Vector<float>(.5f); var gamma=new Vector<float>(Gas.Gamma);
        var gm1=new Vector<float>(Gas.Gamma-1);
        var pl=gm1*(el-half*ml*ul); var pr=gm1*(er-half*mr*ur);
        var cl=Vector.SquareRoot(gamma*Vector.Max(Vector<float>.One,pl)/rl);
        var cr=Vector.SquareRoot(gamma*Vector.Max(Vector<float>.One,pr)/rr);
        var alpha=half*Vector.Max(Vector.Abs(ul)+cl,Vector.Abs(ur)+cr);
        (half*(ml+mr)-alpha*(rr-rl)).CopyTo(fr,face);
        (half*((ml*ul+pl)+(mr*ur+pr))-alpha*(mr-ml)).CopyTo(fm,face);
        (half*((el+pl)*ul+(er+pr)*ur)-alpha*(er-el)).CopyTo(fe,face);
        (half*(ml*(al/rl)+mr*(ar/rr))-alpha*(ar-al)).CopyTo(fa,face);
    }
    private void Store(int i,State f) { fr[i]=f.Rho; fm[i]=f.M; fe[i]=f.E; fa[i]=f.Air; }
}
