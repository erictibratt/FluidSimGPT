using FluidSim.Core;
using FluidSim.Components;
namespace FluidSim.Scenarios;

public sealed record Controls(float Throttle=0,float Brake=0,bool Clutch=false,int Shift=0);
public sealed record Snapshot(float[] Pressure,float[] Temperature,float[] CylinderPressure,float[] CylinderVolume,float[] IntakeLift,float[] ExhaustLift,float Rpm,float Throttle,float Brake,int Gear,float Speed,float Torque,float Power,double Angle,int Substeps,long Repairs,float AudioL,float AudioR);

public abstract class Scenario
{
    public const int SampleRate=44100;
    public const float Dt=1f/SampleRate;
    public readonly PipeSystem Pipes=new();
    public readonly BoundarySystem Boundaries;
    public readonly List<Volume0D> Components=[];
    public readonly Crankshaft Crank=new();
    public readonly Vehicle Vehicle=new();
    public readonly List<EngineCylinder> Cylinders=[];
    public readonly SoundMixer Mixer=new();
    public Solver Solver=null!;
    public float LastMixL,LastMixR,Throttle,Brake;
    public abstract string Name { get; }
    protected Boundary? IntakeEnd,ExhaustEnd;
    protected Port? ThrottlePort;
    protected Scenario() => Boundaries=new(Pipes);
    public virtual void Apply(Controls input)
    {
        Throttle=input.Throttle; Brake=input.Brake;
        if(input.Shift!=0) Vehicle.Shift(input.Shift);
        clutch=input.Clutch;
    }
    private bool clutch;
    public virtual void Process()
    {
        if(ThrottlePort!=null) ThrottlePort.Area=.000012f+Throttle*.001f;
        foreach(var c in Cylinders) c.PreStep(Dt,Crank);
        Solver.Step(Dt);
        float torque=0,burn=0;
        foreach(var c in Cylinders) { torque+=c.Torque; burn+=c.BurnPulse; }
        float starter=Cylinders.Count>0?Math.Clamp((1050-Crank.Rpm)*.06f,0,55):0;
        float load=Vehicle.Update(Dt,Crank,clutch);
        float friction=1.2f+Crank.Omega*.007f;
        Crank.Step(Dt,torque+starter-friction-Brake*65-load,Vehicle.ReflectedInertia);
        (LastMixL,LastMixR)=Mixer.Process(IntakeEnd?.LastMassFlow??0,ExhaustEnd?.LastMassFlow??0,Crank.Angle,Crank.Rpm,burn);
    }
    public Snapshot Capture() => new(
        Enumerable.Range(0,Pipes.Rho.Length).Select(i=>Pipes.Get(i).P).ToArray(),
        Enumerable.Range(0,Pipes.Rho.Length).Select(i=>Pipes.Get(i).T).ToArray(),
        Cylinders.Select(c=>c.Pressure).ToArray(),Cylinders.Select(c=>c.Volume).ToArray(),
        Cylinders.Select(c=>c.IntakeLift).ToArray(),Cylinders.Select(c=>c.ExhaustLift).ToArray(),
        Crank.Rpm,Throttle,Brake,Vehicle.Gear,Vehicle.Speed,Crank.AverageTorque,Crank.AveragePower,Crank.Angle,Solver.LastSubsteps,Pipes.Repairs,LastMixL,LastMixR);
}

public class SingleCylScenario : Scenario
{
    public override string Name => "600 / PARALLEL TWIN";
    public SingleCylScenario(bool twoStroke=false)
    {
        var plenum=new Volume0D("Intake plenum",.0018f); var collector=new Volume0D("Exhaust collector",.0012f);
        Components.Add(plenum); Components.Add(collector);
        var inlet=Pipes.Add("AIR INLET",.45f,.0012f,95,320,310,320);
        IntakeEnd=Boundaries.Connect(inlet,false,open:true);
        ThrottlePort=plenum.AddPort(.00002f); Boundaries.Connect(inlet,true,ThrottlePort);
        var c1=new EngineCylinder("Cylinder 1",0,twoStroke); var c2=new EngineCylinder("Cylinder 2",Math.PI,twoStroke);
        Cylinders.Add(c1); Cylinders.Add(c2); Components.Add(c1); Components.Add(c2);
        for(int i=0;i<2;i++)
        {
            float y=i==0?260:470;
            var intake=Pipes.Add($"INTAKE {i+1}",.28f,.0008f,350,320,510,y);
            Boundaries.Connect(intake,false,plenum.AddPort(.0008f)); Boundaries.Connect(intake,true,Cylinders[i].Intake);
            var exhaust=Pipes.Add($"HEADER {i+1}",.6f,.0009f,650,y,820,360);
            Boundaries.Connect(exhaust,false,Cylinders[i].Exhaust); Boundaries.Connect(exhaust,true,collector.AddPort(.0009f));
        }
        var mid=new Volume0D("Muffler",.0025f); Components.Add(mid);
        var tail=Pipes.Add("COLLECTOR",.55f,.0015f,855,360,1020,360);
        Boundaries.Connect(tail,false,collector.AddPort(.0015f)); Boundaries.Connect(tail,true,mid.AddPort(.0015f));
        var tip=Pipes.Add("TAILPIPE",.5f,.0012f,1040,360,1210,360);
        Boundaries.Connect(tip,false,mid.AddPort(.0012f)); ExhaustEnd=Boundaries.Connect(tip,true,open:true);
        Solver=new(Pipes,Boundaries,Components);
    }
}
public sealed class TwoStrokeScenario() : SingleCylScenario(true)
{ public override string Name => "TWO STROKE / PORTED TWIN"; }
public class TestScenario : Scenario
{
    public override string Name => "SHOCK TUBE / FLOW TEST";
    public TestScenario()
    {
        var p=Pipes.Add("SHOCK TUBE",1,.001f,120,360,1180,360,100);
        for(int i=0;i<100;i++) Pipes.Set(i,State.FromPT(i<50?150000:100000,293));
        Boundaries.Connect(p,false); Boundaries.Connect(p,true);
        Pipes.MomentumDamping=Pipes.ThermalRelaxation=0;
        Solver=new(Pipes,Boundaries,Components);
    }
}
public sealed class HelmholtzScenario : Scenario
{
    public override string Name => "HELMHOLTZ / RESONATOR";
    public HelmholtzScenario()
    {
        var v=new Volume0D("Cavity",.002f,105000) { HeatRelaxation=0 }; Components.Add(v);
        var p=Pipes.Add("RESONATOR NECK",.16f,.0003f,350,360,1080,360,20);
        Boundaries.Connect(p,false,v.AddPort(.0003f)); ExhaustEnd=Boundaries.Connect(p,true,open:true);
        Solver=new(Pipes,Boundaries,Components);
    }
}
