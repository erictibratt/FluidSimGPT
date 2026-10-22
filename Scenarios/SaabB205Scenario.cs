using FluidSim.Core;
using FluidSim.Components;
namespace FluidSim.Scenarios;

// Stock B205 long-block geometry and cams; a custom naturally aspirated gas path.
// Sources and all assumed dimensions are recorded in docs/SAAB_B205.md.
public sealed class SaabB205Scenario : Scenario
{
    public override string Name => "SAAB B205 / 2.0 NA";
    public override string Description => "1985 cc   /   1-3-4-2   /   4-2-1 HEADER   /   STOCK 8.8:1";
    private static float Area(float diameter) => MathF.PI*diameter*diameter*.25f;
    public SaabB205Scenario()
    {
        IdleTarget=825; StarterGain=.12f; StarterMax=80;
        FrictionBase=8; FrictionSpeed=.025f; BrakeCapacity=220;
        ThrottleLeak=Area(.003f); ThrottleArea=Area(.060f);
        Crank.Inertia=.24f; Crank.Omega=100;
        Vehicle.Mass=1400; Vehicle.WheelRadius=.31f; Vehicle.FinalDrive=4.05f;
        Vehicle.RollingForce=155; Vehicle.DragCoefficient=.40f; Vehicle.ClutchCapacity=240;
        Vehicle.MaximumGear=5;
        float[] ratios=[0,3.38f,1.76f,1.18f,.89f,.70f];
        Array.Copy(ratios,Vehicle.Ratios,ratios.Length);
        Mixer.Mechanical=.012f; Mixer.Wet=.12f;

        var airbox=Reservoir("AIRBOX 3 L",.003f,125,350,55,60);
        var plenum=Reservoir("PLENUM 3 L",.003f,295,275,45,220);
        var inlet=Duct("SNORKEL",.45f,.065f,8,45,380,125,380);
        IntakeEnd=Boundaries.Connect(inlet,false,open:true); End(inlet,true,airbox);
        var feed=Duct("FILTER / THROTTLE 60",.25f,.065f,8,180,380,295,380);
        End(feed,false,airbox,.85f); ThrottlePort=plenum.AddPort(ThrottleLeak);
        ThrottlePort.DischargeCoefficient=.72f; Boundaries.Connect(feed,true,ThrottlePort);

        double[] phases=[0,Math.PI,3*Math.PI,2*Math.PI];
        var pairs=new[] { Reservoir("PAIR 1 + 4",.00020f,775,285,45,40), Reservoir("PAIR 2 + 3",.00020f,775,420,45,40) };
        for(int i=0;i<4;i++)
        {
            var c=new EngineCylinder($"B205 cylinder {i+1}",phases[i])
            {
                Bore=.090f, Stroke=.078f, Rod=.159f, Compression=8.8f,
                IntakeOpen=Rad(-12), IntakeClose=MathF.PI+Rad(39),
                ExhaustOpen=3*MathF.PI-Rad(34), ExhaustClose=4*MathF.PI+Rad(14),
                IntakeValveDiameter=.033f,ExhaustValveDiameter=.029f,
                IntakeMaxLift=.00831f,ExhaustMaxLift=.00807f,
                SparkAdvance=Rad(24),BurnDuration=Rad(65),RevLimit=6500
            };
            c.ResetCharge(); Cylinders.Add(c); Components.Add(c);
            float y=270+i*75;
            CylinderVisuals.Add(new(485,y-20,125,55));
            var runner=Duct($"RUNNER {i+1}",.40f,.038f,16,340,y,485,y);
            End(runner,false,plenum); Boundaries.Connect(runner,true,c.Intake);
            int pair=i==0||i==3?0:1; float py=pair==0?305:440;
            var header=Duct($"PRIMARY {i+1}",.55f,.038f,16,610,y,775,py);
            Boundaries.Connect(header,false,c.Exhaust); End(header,true,pairs[pair]);
        }
        var collector=Reservoir("2 INTO 1",.00035f,910,350,30,45);
        for(int i=0;i<2;i++)
        {
            float y=i==0?305:440;
            var secondary=Duct($"SECONDARY {i+1}",.42f,.045f,12,820,y,910,372);
            End(secondary,false,pairs[i]); End(secondary,true,collector);
        }
        var resonator=Reservoir("RESONATOR 3 L",.003f,1015,350,38,45);
        var main=Duct("MAIN 54",.70f,.054f,16,940,372,1015,372);
        End(main,false,collector); End(main,true,resonator);
        var rear=Reservoir("REAR SILENCER 6 L",.006f,1140,350,50,45);
        var link=Duct("LINK 54",.65f,.054f,16,1053,372,1140,372);
        End(link,false,resonator); End(link,true,rear);
        var tip=Duct("TIP 54",.45f,.054f,12,1190,372,1260,372);
        End(tip,false,rear); ExhaustEnd=Boundaries.Connect(tip,true,open:true);
        Solver=new(Pipes,Boundaries,Components);
    }
    private static float Rad(float degrees) => degrees*MathF.PI/180;
    private Volume0D Reservoir(string name,float volume,float x,float y,float width,float height)
    {
        var v=new Volume0D(name,volume) { HeatRelaxation=.6f };
        Components.Add(v); VolumeVisuals.Add(new(name,x,y,width,height)); return v;
    }
    private Pipe Duct(string name,float length,float diameter,int count,float x1,float y1,float x2,float y2)
    {
        var p=Pipes.Add(name,length,Area(diameter),x1,y1,x2,y2,count);
        p.FrictionFactor=.025f; return p;
    }
    private void End(Pipe pipe,bool right,Volume0D owner,float discharge=.95f)
    {
        var port=owner.AddPort(pipe.Area); port.DischargeCoefficient=discharge;
        Boundaries.Connect(pipe,right,port);
    }
}
