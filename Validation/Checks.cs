using System.Diagnostics;
using FluidSim.Audio;
using FluidSim.Core;
using FluidSim.Components;
using FluidSim.Scenarios;
namespace FluidSim.Validation;

public static class Checks
{
    private static void Require(bool condition,string message) { if(!condition) throw new Exception(message); }
    private static void Near(double actual,double expected,double tolerance,string message) => Require(Math.Abs(actual-expected)<=tolerance,$"{message}: {actual} != {expected}");
    public static int Run()
    {
        (string Name,Action Test)[] tests=[
            ("Euler uniform flux",()=> { var s=State.FromPT(101325,300,25,.7f); var f=State.Rusanov(s,s); Near(f.Rho,s.M,1e-5,"mass flux"); Near(f.E,s.Flux.E,1,"energy flux"); }),
            ("Uniform pipe and reflecting wall",()=>
            {
                var p=new PipeSystem { MomentumDamping=0,ThermalRelaxation=0 }; var pipe=p.Add("uniform",1,.001f,0,0,1,0);
                var b=new BoundarySystem(p); b.Connect(pipe,false); b.Connect(pipe,true); var solver=new Solver(p,b,[]);
                for(int i=0;i<1000;i++) solver.Step(Scenario.Dt);
                Near(p.Get(5).P,Gas.AmbientPressure,.2,"pressure drift"); Near(p.Get(0).M,0,.0001,"wall momentum");
                var s=State.FromPT(101325,293,15); var flux=State.Rusanov(s,s.Reflect()); Near(flux.Rho,0,1e-6,"wall mass"); Near(flux.E,0,1e-6,"wall energy");
            }),
            ("Orifice direction, choking and closure",()=>
            {
                var a=State.FromPT(200000,300); var b=Gas.Ambient;
                float q=BoundarySystem.Orifice(a,b,.001f);
                Require(q>0,"forward flow"); Near(BoundarySystem.Orifice(b,a,.001f),-q,1e-6,"reverse flow");
                Near(BoundarySystem.Orifice(a,b,0),0,0,"closed valve");
                Near(BoundarySystem.Orifice(a,State.FromPT(1000,300),.001f),q,.001,"choked flow");
            }),
            ("Coupled mass and energy conservation / donor limits",Conservation),
            ("Cycle timing and twin phase",()=>
            {
                Require(EngineCylinder.Crossed(12.4,12.8,0,4*Math.PI),"720 degree wrap event");
                Require(!EngineCylinder.Crossed(6,6.4,0,4*Math.PI),"four stroke must not fire each revolution");
                var s=new SingleCylScenario(); Near(s.Cylinders[1].PhaseOffset,Math.PI,1e-12,"phase");
                Near(s.Cylinders[0].VolumeAt(0)*10.5,s.Cylinders[0].VolumeAt(MathF.PI),1e-9,"compression ratio");
                Near(EngineCylinder.Lift(6,0,4),0,0,"valve closure");
            }),
            ("Ring wraparound, interpolation, underrun",()=>
            {
                var b=new StereoRingBuffer(8); for(int i=0;i<8;i++) Require(b.Write(i,-i),"initial write"); Require(!b.Write(9,9),"overflow");
                for(int i=0;i<5;i++) { Require(b.Read(1,out var l,out var r),"read"); Near(l,i,1e-6,"order"); Near(r,-i,1e-6,"stereo"); }
                for(int i=8;i<13;i++) Require(b.Write(i,-i),"wrap write");
                for(int i=5;i<12;i++) { b.Read(1,out var l,out _); Near(l,i,1e-6,"wrap order"); }
                Require(!b.Read(1,out _,out _),"underrun");
                var slow=new StereoRingBuffer(8); for(int i=0;i<8;i++) slow.Write(i,i);
                slow.Read(.5f,out _,out _); slow.Read(.5f,out var v,out _); Near(v,.5,1e-6,"interpolation");
            }),
            ("Twin stability, throttle, shift and clutch",()=>
            {
                var s=new SingleCylScenario(); float idleRpm=0,acceleratedRpm=0;
                for(int i=0;i<132300;i++)
                {
                    if(i==44100) { idleRpm=s.Crank.Rpm; s.Apply(new(.6f,0,false,1)); }
                    if(i==88200) { acceleratedRpm=s.Crank.Rpm; s.Apply(new(.2f,.3f,true,-1)); }
                    s.Process(); Validate(s);
                }
                Require(s.Vehicle.Gear==0,"shift back to neutral"); Require(s.Crank.Rpm>0,"engine running");
                Require(acceleratedRpm>idleRpm+300,"throttle must accelerate engine");
                Near(s.Vehicle.ClutchTorque,0,1e-6,"clutch disengages torque");
                Require(s.Vehicle.Speed>1,"engaged drivetrain moves vehicle");
                Require(s.Pipes.Repairs==0,"positivity repairs required"); Console.WriteLine($"  final RPM {s.Crank.Rpm:0}; road speed {s.Vehicle.Speed*3.6:0.0} km/h");
            }),
            ("Alternative scenarios",()=> { foreach(Scenario s in new Scenario[]{new TestScenario(),new HelmholtzScenario(),new TwoStrokeScenario()}) for(int i=0;i<22050;i++) { s.Process(); Validate(s); } })
        ];
        int failures=0; var watch=Stopwatch.StartNew();
        foreach(var (name,test) in tests) try { test(); Console.WriteLine($"PASS {name}"); } catch(Exception ex) { failures++; Console.WriteLine($"FAIL {name}: {ex.Message}"); }
        Console.WriteLine($"{tests.Length-failures}/{tests.Length} passed in {watch.Elapsed.TotalSeconds:0.00}s"); return failures==0?0:1;
    }
    private static void Conservation()
    {
        var p=new PipeSystem { MomentumDamping=0,ThermalRelaxation=0 }; var pipe=p.Add("closed system",.5f,.001f,0,0,1,0);
        var v=new Volume0D("reservoir",.001f,140000) { HeatRelaxation=0 };
        var b=new BoundarySystem(p); b.Connect(pipe,false); var port=v.AddPort(.001f); b.Connect(pipe,true,port);
        var solver=new Solver(p,b,[v]);
        double Mass()=>v.Mass+p.Rho.Sum(x=>(double)x*pipe.Area*pipe.Dx);
        double Energy()=>v.InternalEnergy+p.Energy.Sum(x=>(double)x*pipe.Area*pipe.Dx);
        double m=Mass(),e=Energy();
        for(int i=0;i<4000;i++) solver.Step(Scenario.Dt);
        Near(Mass(),m,m*2e-5,"total mass"); Near(Energy(),e,e*2e-5,"total energy");
        Require(v.Mass>0 && p.Rho.All(x=>x>0),"donor positivity"); Require(p.Repairs==0,"no repairs");
    }
    private static void Validate(Scenario s)
    {
        Require(float.IsFinite(s.LastMixL)&&float.IsFinite(s.LastMixR)&&MathF.Abs(s.LastMixL)<=1&&MathF.Abs(s.LastMixR)<=1,"audio bounds");
        foreach(var c in s.Components) Require(c.Mass>0 && c.Pressure>0 && float.IsFinite(c.Temperature),"component state");
        for(int i=0;i<s.Pipes.Rho.Length;i++) { var q=s.Pipes.Get(i); Require(q.Rho>0 && q.P>0 && float.IsFinite(q.T),"pipe state"); }
    }
    public static int Headless(Scenario s,string[] args)
    {
        double seconds=double.Parse(args.FirstOrDefault(a=>a.StartsWith("--seconds="))?.Split('=')[1]??"5",System.Globalization.CultureInfo.InvariantCulture);
        string? wav=args.FirstOrDefault(a=>a.StartsWith("--wav="))?.Substring(6);
        using var file=wav==null?null:new BinaryWriter(File.Create(wav));
        int frames=checked((int)(seconds*Scenario.SampleRate));
        if(file!=null) { file.Write("RIFF"u8); file.Write(36+frames*4); file.Write("WAVEfmt "u8); file.Write(16); file.Write((short)1); file.Write((short)2); file.Write(44100); file.Write(176400); file.Write((short)4); file.Write((short)16); file.Write("data"u8); file.Write(frames*4); }
        var watch=Stopwatch.StartNew(); float peak=0; double square=0;
        for(int i=0;i<frames;i++)
        {
            if(i%44100==0) { s.Apply(new(i<44100?0:.65f)); Console.WriteLine($"t={i/44100}s rpm={s.Crank.Rpm:0} max T={s.Capture().Temperature.Max():0} K"); }
            s.Process(); Validate(s); peak=MathF.Max(peak,MathF.Abs(s.LastMixL)); square+=s.LastMixL*s.LastMixL;
            file?.Write((short)(s.LastMixL*32767)); file?.Write((short)(s.LastMixR*32767));
        }
        Console.WriteLine($"{s.Name}: {seconds}s in {watch.Elapsed.TotalSeconds:0.00}s; RPM {s.Crank.Rpm:0}; peak {peak:0.000}; RMS {Math.Sqrt(square/frames):0.000}; repairs {s.Pipes.Repairs}");
        return 0;
    }
}
