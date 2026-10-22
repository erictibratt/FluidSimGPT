using System.Numerics;
namespace FluidSim.Core;

public sealed record Pipe(string Name, int Start, int Count, float Dx, float Area, float X1, float Y1, float X2, float Y2)
{
    internal State LeftFlux,RightFlux;
}

public sealed class PipeSystem
{
    public readonly List<Pipe> Pipes = [];
    public float[] Rho = [], Momentum = [], Energy = [], Air = [];
    private float[] fr = [], fm = [], fe = [], fa = [];
    public float MomentumDamping = .3f, ThermalRelaxation = .02f;
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
    public float MaxWaveSpeed() { float s=1; for(int i=0;i<Rho.Length;i++) { var q=Get(i); s=MathF.Max(s,MathF.Abs(q.U)+q.C); } return s; }
    public void Advance(float dt, BoundarySystem boundaries)
    {
        int offset=0;
        foreach(var p in Pipes)
        {
            Store(offset,boundaries.Flux(p,false));
            for(int j=1;j<p.Count;j++) Store(offset+j,State.Rusanov(Get(p.Start+j-1),Get(p.Start+j)));
            Store(offset+p.Count,boundaries.Flux(p,true));
            float k=dt/p.Dx; int jv=0, w=Vector<float>.Count;
            for(;jv<=p.Count-w;jv+=w)
            {
                Update(Rho,fr,p.Start+jv,offset+jv,k); Update(Momentum,fm,p.Start+jv,offset+jv,k);
                Update(Energy,fe,p.Start+jv,offset+jv,k); Update(Air,fa,p.Start+jv,offset+jv,k);
            }
            for(;jv<p.Count;jv++)
            {
                int i=p.Start+jv,f=offset+jv;
                Rho[i]-=k*(fr[f+1]-fr[f]); Momentum[i]-=k*(fm[f+1]-fm[f]);
                Energy[i]-=k*(fe[f+1]-fe[f]); Air[i]-=k*(fa[f+1]-fa[f]);
            }
            for(int j=0;j<p.Count;j++)
            {
                int i=p.Start+j;
                if(!float.IsFinite(Energy[i]) || !float.IsFinite(Rho[i]) || !float.IsFinite(Momentum[i]))
                    throw new ArithmeticException($"Nonfinite pipe state in {p.Name}, cell {j}");
                float r=MathF.Max(.00001f,Rho[i]);
                float kinetic=.5f*Momentum[i]*Momentum[i]/r;
                if(r!=Rho[i] || Energy[i]<kinetic+10) Repairs++;
                Rho[i]=r; Energy[i]=MathF.Max(kinetic+10,Energy[i]); Air[i]=Math.Clamp(Air[i],0,r);
                Momentum[i]*=MathF.Exp(-MomentumDamping*dt);
                Energy[i]+=(r*Gas.Cv*Gas.AmbientTemperature+.5f*Momentum[i]*Momentum[i]/r-Energy[i])*MathF.Min(1,ThermalRelaxation*dt);
            }
            offset+=p.Count+1;
        }
    }
    private static void Update(float[] u,float[] f,int i,int j,float k) =>
        (new Vector<float>(u,i)-new Vector<float>(k)*(new Vector<float>(f,j+1)-new Vector<float>(f,j))).CopyTo(u,i);
    private void Store(int i,State f) { fr[i]=f.Rho; fm[i]=f.M; fe[i]=f.E; fa[i]=f.Air; }
}
