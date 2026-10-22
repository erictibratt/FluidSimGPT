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
            ("B205 geometry, cam events, firing order, NA network and stability",SaabValidation),
            ("Orifice table against analytic compressible flow",OrificeTable),
            ("SIMD/scalar agreement, segment edges and remainders",SimdAgreement),
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
            ("PCM lifetime, callback seams and starvation recovery",()=>
            {
                var ring=new StereoRingBuffer(); var playback=new PlaybackProcessor(ring);
                var samples=playback.Samples;
                var address=System.Runtime.InteropServices.Marshal.UnsafeAddrOfPinnedArrayElement(samples,0);
                for(int i=0;i<5000;i++) ring.Write(.3f,.3f);
                playback.Fill(1); playback.Fill(1);
                int previous=samples[^2];
                playback.Fill(1);
                Require(Math.Abs(samples[0]-previous)<30,"callback seam introduces click");
                for(int block=0;block<14;block++)
                {
                    previous=samples[^2]; playback.Fill(1);
                    for(int i=0;i<samples.Length;i+=2)
                    {
                        Require(Math.Abs(samples[i]-previous)<900,"starvation introduces discontinuity at 2.5x output gain");
                        previous=samples[i];
                    }
                }
                Require(ring.Underruns>0,"exercise starvation path");
                for(int i=0;i<5000;i++) ring.Write(.3f,.3f);
                previous=samples[^2]; playback.Fill(1);
                Require(Math.Abs(samples[0]-previous)<900,"recovery click at 2.5x output gain");
                GC.Collect(2,GCCollectionMode.Forced,true,true);
                Require(address==System.Runtime.InteropServices.Marshal.UnsafeAddrOfPinnedArrayElement(samples,0),"PCM pointer moved during GC");
                Require(ReferenceEquals(samples,playback.Samples),"callback allocated replacement PCM");
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
            ("Gear coupling, shift release and clutch pedal",()=>
            {
                var crank=new Crankshaft { Omega=200 };
                var car=new Vehicle { Mass=1400,WheelRadius=.31f,FinalDrive=4.05f,ClutchCapacity=240 };
                car.Ratios[1]=3.38f; car.Ratios[2]=1.76f;
                car.Shift(1);
                Near(car.Update(.01f,crank,false),0,0,"shift must release clutch");
                for(int i=0;i<60;i++) car.Update(.01f,crank,false);
                Require(car.ClutchTorque>100,"first gear must couple engine to wheels");
                car.Shift(1);
                Near(car.Update(.01f,crank,false),0,0,"upshift must release clutch");
                for(int i=0;i<60;i++) car.Update(.01f,crank,false);
                Require(car.Gear==2 && car.ClutchTorque>0,"second gear must transmit drive");
                Near(car.Update(.01f,crank,true),0,0,"clutch pedal must disengage drive");
                car.Shift(-1); Require(car.Gear==1,"downshift must select first");
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
        var second=p.Add("second closed branch",.5f,.001f,0,0,1,0);
        for(int i=0;i<second.Count;i++) p.Set(second.Start+i,State.FromPT(90000,300));
        var v=new Volume0D("reservoir",.001f,140000) { HeatRelaxation=0 };
        var b=new BoundarySystem(p); b.Connect(pipe,false); var port=v.AddPort(.001f); b.Connect(pipe,true,port);
        b.Connect(second,false,v.AddPort(.001f)); b.Connect(second,true);
        var solver=new Solver(p,b,[v]);
        double Mass()=>v.Mass+p.Rho.Sum(x=>(double)x*pipe.Area*pipe.Dx);
        double Energy()=>v.InternalEnergy+p.Energy.Sum(x=>(double)x*pipe.Area*pipe.Dx);
        double m=Mass(),e=Energy();
        for(int i=0;i<4000;i++) solver.Step(Scenario.Dt);
        Near(Mass(),m,m*2e-5,"total mass"); Near(Energy(),e,e*2e-5,"total energy");
        Require(v.Mass>0 && p.Rho.All(x=>x>0),"donor positivity"); Require(p.Repairs==0,"no repairs");
    }
    private static void SimdAgreement()
    {
        var scalar=new PipeSystem { UseSimd=false }; var simd=new PipeSystem();
        var bs=new BoundarySystem(scalar); var bv=new BoundarySystem(simd);
        foreach(int count in new[]{1,3,7,8,10,12,16,17,31})
        {
            var a=scalar.Add($"scalar {count}",.05f*count,.001f,0,0,1,0,count);
            var b=simd.Add($"simd {count}",.05f*count,.001f,0,0,1,0,count);
            a.FrictionFactor=b.FrictionFactor=.025f;
            bs.Connect(a,false); bs.Connect(a,true); bv.Connect(b,false); bv.Connect(b,true);
            for(int j=0;j<count;j++)
            {
                var state=State.FromPT(90000+12000*MathF.Sin(j*.4f),290+20*MathF.Cos(j),12*MathF.Sin(j),.3f+.2f*MathF.Cos(j));
                scalar.Set(a.Start+j,state); simd.Set(b.Start+j,state);
            }
        }
        var ss=new Solver(scalar,bs,[]); var sv=new Solver(simd,bv,[]);
        for(int step=0;step<80;step++)
        {
            Near(simd.MaxWaveSpeed(),scalar.MaxWaveSpeed(),.002,"CFL max wave speed");
            ss.Step(Scenario.Dt); sv.Step(Scenario.Dt);
            for(int i=0;i<scalar.Rho.Length;i++)
            {
                var a=scalar.Get(i); var b=simd.Get(i);
                Near(b.Rho,a.Rho,2e-5,"SIMD density"); Near(b.M,a.M,.002,"SIMD momentum");
                Near(b.E,a.E,3,"SIMD energy"); Near(b.Air,a.Air,2e-5,"SIMD species");
            }
        }
        Require(simd.Repairs==scalar.Repairs,"SIMD repair accounting");
    }
    private static void OrificeTable()
    {
        var a=State.FromPT(200000,300);
        for(int i=1;i<2000;i++)
        {
            double ratio=.0005*i,g=Gas.Gamma,critical=Math.Pow(2/(g+1),g/(g-1));
            double f=ratio<=critical?Math.Sqrt(g)*Math.Pow(2/(g+1),(g+1)/(2*(g-1))):Math.Sqrt(2*g/(g-1)*(Math.Pow(ratio,2/g)-Math.Pow(ratio,(g+1)/g)));
            double expected=.72*.001*a.P/Math.Sqrt(Gas.R*a.T)*f;
            Near(BoundarySystem.Orifice(a,State.FromPT((float)(a.P*ratio),300),.001f),expected,Math.Max(.000002,expected*.00015),"orifice lookup error");
        }
    }
    private static void SaabValidation()
    {
        var s=new SaabB205Scenario();
        Near(s.Cylinders.Sum(c=>c.Displacement)*1e6,1984.91,.2,"B205 displacement");
        foreach(var c in s.Cylinders)
        {
            Near(c.Rod,.159,1e-6,"B205 rod"); Near(c.Compression,8.8,1e-5,"stock compression");
            Near(c.IntakeMaxLift,.00831,1e-7,"intake lift"); Near(c.ExhaustMaxLift,.00807,1e-7,"exhaust lift");
            Near(c.VolumeAt(MathF.PI)/c.VolumeAt(0),8.8,1e-5,"compression geometry");
            Require(EngineCylinder.Lift(c.IntakeOpen-.01, c.IntakeOpen,c.IntakeClose)==0,"intake opens too early");
            Require(EngineCylinder.Lift(c.IntakeClose+.01,c.IntakeOpen,c.IntakeClose)==0,"intake closes too late");
        }
        var eventOrder=new List<int>(); double period=4*Math.PI;
        double start=2*Math.PI-s.Cylinders[0].SparkAdvance;
        for(int eventIndex=0;eventIndex<4;eventIndex++)
        {
            double eventAngle=start+eventIndex*Math.PI;
            for(int cylinder=0;cylinder<4;cylinder++)
                if(EngineCylinder.Crossed(eventAngle+s.Cylinders[cylinder].PhaseOffset-.001,eventAngle+s.Cylinders[cylinder].PhaseOffset+.001,start,period)) eventOrder.Add(cylinder+1);
        }
        Require(eventOrder.SequenceEqual(new[]{1,3,4,2}),"incorrect firing order");
        var headers=s.Pipes.Pipes.Where(p=>p.Name.StartsWith("PRIMARY")).ToArray();
        var merge=headers.Select(p=>s.Boundaries.Ends.Single(b=>b.Pipe==p&&b.Right).Port!.Owner).ToArray();
        Require(merge[0]==merge[3]&&merge[1]==merge[2]&&merge[0]!=merge[1],"4-2-1 pairing");
        Require(s.Boundaries.Ends.Count(b=>b.Open)==2,"NA network must have two ambient open ends");
        float cruisingSlip=0,cruisingRpm=0,cruisingSpeed=0;
        for(int i=0;i<220500;i++)
        {
            if(i==88200) s.Apply(new(.7f,0,false,1));
            if(i==176400) s.Apply(new(.1f,.3f,true));
            s.Process(); Validate(s);
            if(i==176399)
            {
                cruisingRpm=s.Crank.Rpm; cruisingSpeed=s.Vehicle.Speed;
                cruisingSlip=MathF.Abs(s.Crank.Omega-s.Vehicle.Speed/s.Vehicle.WheelRadius*s.Vehicle.Ratios[s.Vehicle.Gear]*s.Vehicle.FinalDrive);
            }
        }
        Require(cruisingSpeed>1,"B205 first gear must accelerate the car");
        Require(cruisingSlip<cruisingRpm*(2*MathF.PI/60)*.2f,$"B205 clutch slips excessively at {cruisingRpm:0} RPM and {cruisingSpeed*3.6f:0.0} km/h");
        Require(s.Pipes.Repairs==0,"B205 positivity repairs"); Require(s.Crank.Rpm>200&&s.Crank.Rpm<7000,"B205 engine speed");
    }
    private static void Validate(Scenario s)
    {
        Require(float.IsFinite(s.LastMixL)&&float.IsFinite(s.LastMixR)&&MathF.Abs(s.LastMixL)<=1&&MathF.Abs(s.LastMixR)<=1,"audio bounds");
        foreach(var c in s.Components) Require(c.Mass>0 && c.Pressure>0 && float.IsFinite(c.Temperature),"component state");
        for(int i=0;i<s.Pipes.Rho.Length;i++) { var q=s.Pipes.Get(i); Require(q.Rho>0 && q.P>0 && float.IsFinite(q.T),"pipe state"); }
    }
    public static int Headless(Scenario s,string[] args)
    {
        s.Pipes.UseSimd=!args.Contains("--scalar");
        s.Solver.EnableProfiling=args.Contains("--profile");
        if(args.Contains("--flow-only")) { s.Mixer.Mechanical=0; s.Mixer.Wet=0; }
        double seconds=double.Parse(args.FirstOrDefault(a=>a.StartsWith("--seconds="))?.Split('=')[1]??"5",System.Globalization.CultureInfo.InvariantCulture);
        string? wav=args.FirstOrDefault(a=>a.StartsWith("--wav="))?.Substring(6);
        using var file=wav==null?null:new BinaryWriter(File.Create(wav));
        int frames=checked((int)(seconds*Scenario.SampleRate));
        if(file!=null) { file.Write("RIFF"u8); file.Write(36+frames*4); file.Write("WAVEfmt "u8); file.Write(16); file.Write((short)1); file.Write((short)2); file.Write(44100); file.Write(176400); file.Write((short)4); file.Write((short)16); file.Write("data"u8); file.Write(frames*4); }
        var watch=Stopwatch.StartNew(); float peak=0; double square=0;
        double idleSeconds=double.Parse(args.FirstOrDefault(a=>a.StartsWith("--idle-seconds="))?.Split('=')[1]??"1",System.Globalization.CultureInfo.InvariantCulture);
        float throttle=float.Parse(args.FirstOrDefault(a=>a.StartsWith("--throttle="))?.Split('=')[1]??"0.65",System.Globalization.CultureInfo.InvariantCulture);
        for(int i=0;i<frames;i++)
        {
            if(i%44100==0) { s.Apply(new(i<idleSeconds*44100?0:throttle)); Console.WriteLine($"t={i/44100}s rpm={s.Crank.Rpm:0} max T={s.Capture().Temperature.Max():0} K"); }
            s.Process(); Validate(s); peak=MathF.Max(peak,MathF.Abs(s.LastMixL)); square+=s.LastMixL*s.LastMixL;
            file?.Write((short)(s.LastMixL*32767)); file?.Write((short)(s.LastMixR*32767));
        }
        Console.WriteLine($"{s.Name}: {seconds}s in {watch.Elapsed.TotalSeconds:0.00}s; RPM {s.Crank.Rpm:0}; peak {peak:0.000}; RMS {Math.Sqrt(square/frames):0.000}; repairs {s.Pipes.Repairs}; SIMD {s.Pipes.UseSimd&&System.Numerics.Vector.IsHardwareAccelerated}, lanes {System.Numerics.Vector<float>.Count}");
        if(s.Solver.EnableProfiling)
        {
            double scale=1000.0/Stopwatch.Frequency;
            Console.WriteLine($"Solver profile: boundaries {s.Solver.BoundaryTicks*scale:0.0} ms; pipes {s.Solver.PipeTicks*scale:0.0} ms; components {s.Solver.ComponentTicks*scale:0.0} ms");
        }
        return 0;
    }
}
