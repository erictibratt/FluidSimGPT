using FluidSim.Components;
namespace FluidSim.Core;

public sealed class Boundary(Pipe pipe,bool right,Port? port,bool open)
{
    public Pipe Pipe=pipe;
    public bool Right=right, Open=open;
    public Port? Port=port;
    public float LastMassFlow, Inertance, Resistance=100, InertialFlow;
    internal State FaceFlux
    {
        get => Right?Pipe.RightFlux:Pipe.LeftFlux;
        set { if(Right) Pipe.RightFlux=value; else Pipe.LeftFlux=value; }
    }
}
public sealed class BoundarySystem(PipeSystem pipes)
{
    private const int FlowTableSize=1024;
    private static readonly float CriticalRatio=MathF.Pow(2/(Gas.Gamma+1),Gas.Gamma/(Gas.Gamma-1));
    private static readonly float ChokedFactor=MathF.Sqrt(Gas.Gamma)*MathF.Pow(2/(Gas.Gamma+1),(Gas.Gamma+1)/(2*(Gas.Gamma-1)));
    private static readonly float[] FlowTable=BuildFlowTable();
    public List<Boundary> Ends { get; }=[];
    public Boundary Connect(Pipe p,bool right,Port? port=null,bool open=false)
    { var b=new Boundary(p,right,port,open); Ends.Add(b); return b; }
    public State Flux(Pipe p,bool right) => right?p.RightFlux:p.LeftFlux;
    public static float Orifice(State a,State b,float area,float discharge=.72f)
    {
        if(area<=0 || MathF.Abs(a.P-b.P)<.01f) return 0;
        bool positive=a.P>b.P; State hi=positive?a:b,lo=positive?b:a;
        float ratio=Math.Clamp(lo.P/hi.P,0,1);
        float f=ChokedFactor;
        if(ratio>CriticalRatio)
        {
            // Parameterize by sqrt(dp/p): smooth at equal pressure, unlike a linear pressure table.
            float index=MathF.Sqrt((1-ratio)/(1-CriticalRatio))*FlowTableSize;
            int i=Math.Min((int)index,FlowTableSize-1);
            f=FlowTable[i]+(FlowTable[i+1]-FlowTable[i])*(index-i);
        }
        return (positive?1:-1)*discharge*area*hi.P/MathF.Sqrt(Gas.R*hi.T)*f;
    }
    private static float[] BuildFlowTable()
    {
        var table=new float[FlowTableSize+1]; double g=Gas.Gamma;
        for(int i=1;i<=FlowTableSize;i++)
        {
            double x=(double)i/FlowTableSize,ratio=1-x*x*(1-CriticalRatio);
            table[i]=(float)Math.Sqrt(Math.Max(0,2*g/(g-1)*(Math.Pow(ratio,2/g)-Math.Pow(ratio,(g+1)/g))));
        }
        table[^1]=ChokedFactor; return table;
    }
    public void Resolve(float dt)
    {
        foreach(var b in Ends)
        {
            var p=b.Pipe; int i=p.Start+(b.Right?p.Count-1:0); var s=pipes.Get(i);
            float sign=b.Right?1:-1;
            if(b.Port is null && !b.Open || b.Port is {Area: <=0})
            { b.FaceFlux=new(0,s.P,0,0); b.LastMassFlow=0; continue; }
            float q; State donor;
            if(b.Open)
            {
                State a=Gas.Ambient;
                float outward=sign*s.U;
                if(outward>=s.C) { b.FaceFlux=s.Flux; b.LastMassFlow=outward*s.Rho*p.Area; continue; }
                // Outgoing interior invariant and incoming atmospheric invariant.
                float jPlus=outward+2*s.C/(Gas.Gamma-1), jMinus=-2*a.C/(Gas.Gamma-1);
                float v=.5f*(jPlus+jMinus), c=MathF.Max(10,.25f*(Gas.Gamma-1)*(jPlus-jMinus));
                float cr=c/a.C,cr2=cr*cr;
                float rho=a.Rho*cr2*cr2*cr; // gamma=1.4 -> isentropic exponent 5
                var ghost=State.FromPT(rho*c*c/Gas.Gamma,c*c/(Gas.Gamma*Gas.R),sign*v,v>=0?s.Y:1);
                var flux=b.Right?State.Rusanov(s,ghost):State.Rusanov(ghost,s);
                q=sign*flux.Rho*p.Area;
                float cap=.2f*s.Rho*p.Area*p.Dx/dt;
                if(q>cap) flux=(cap/q)*flux;
                b.FaceFlux=flux; b.LastMassFlow=sign*flux.Rho*p.Area; continue;
            }
            var port=b.Port!; var owner=port.Owner; State reservoir=owner.BoundaryState;
            q=Orifice(s,reservoir,MathF.Min(p.Area,port.Area),port.DischargeCoefficient); // positive pipe -> volume
            if(b.Inertance>0)
            {
                b.InertialFlow+=dt*(s.P-reservoir.P-b.Resistance*b.InertialFlow)/b.Inertance;
                q=Math.Clamp(b.InertialFlow,-reservoir.Rho*p.Area*300,s.Rho*p.Area*300);
            }
            float cellBudget=.2f*s.Rho*p.Area*p.Dx/dt;
            float volumeBudget=owner.Infinite?float.MaxValue:(float)(Math.Max(0,.2*owner.Mass+owner.OutflowThisStep)/dt);
            q=Math.Clamp(q,-volumeBudget,cellBudget);
            donor=q>=0?s:reservoir;
            float f=sign*q/p.Area;
            float velocity=f/donor.Rho;
            // One shared conservative flux: the exact mass/enthalpy removed from a pipe enters its component.
            b.FaceFlux=new(f,s.P+f*velocity,f*donor.H,f*donor.Y);
            b.LastMassFlow=q;
            port.MassDelta+=q*dt; port.AirDelta+=q*donor.Y*dt; port.EnergyDelta+=q*donor.H*dt;
            if(q<0) owner.OutflowThisStep+=q*dt;
        }
    }
}
