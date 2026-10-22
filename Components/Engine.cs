using FluidSim.Core;
namespace FluidSim.Components;

public sealed class Crankshaft
{
    public double Angle, TotalAngle;
    public float Omega=125, Inertia=.16f, AverageTorque, AveragePower;
    public float Rpm => Omega*60/(2*MathF.PI);
    public void Step(float dt,float torque,float reflectedInertia)
    {
        Omega=MathF.Max(0,Omega+dt*torque/(Inertia+reflectedInertia));
        TotalAngle+=Omega*dt; Angle=TotalAngle%(4*Math.PI);
        float k=1-MathF.Exp(-dt*5); AverageTorque+=k*(torque-AverageTorque);
        AveragePower+=k*(torque*Omega-AveragePower);
    }
}

public sealed class EngineCylinder : Volume0D
{
    public readonly Port Intake, Exhaust;
    public double PhaseOffset;
    public float Bore=.072f, Stroke=.0737f, Rod=.125f, Compression=10.5f;
    public float SparkAdvance=.24f, BurnDuration=1.25f, RevLimit=9500;
    public float Torque, BurnPulse, IntakeLift, ExhaustLift;
    public float Displacement => MathF.PI*Bore*Bore*.25f*Stroke;
    public bool TwoStroke;
    private double previousLocal=-1, fuelEnergy, burnFraction, burnTravel=-1, limiterTime;
    private bool limiting;
    public EngineCylinder(string name,double phase,bool twoStroke=false) : base(name,.0000316f)
    {
        PhaseOffset=phase; TwoStroke=twoStroke; HeatRelaxation=0;
        Intake=AddPort(0); Exhaust=AddPort(0);
        Volume=VolumeAt((float)phase); Mass=Gas.AmbientPressure*Volume/(Gas.R*Gas.AmbientTemperature);
        AirMass=Mass; InternalEnergy=Mass*Gas.Cv*Gas.AmbientTemperature;
    }
    public float VolumeAt(float angle)
    {
        float r=Stroke*.5f,s=MathF.Sin(angle);
        float x=r*(1-MathF.Cos(angle))+Rod-MathF.Sqrt(Rod*Rod-r*r*s*s);
        return Displacement/(Compression-1)+MathF.PI*Bore*Bore*.25f*x;
    }
    public static bool Crossed(double before,double after,double target,double period) =>
        before>=0 && Math.Floor((before-target)/period)<Math.Floor((after-target)/period);
    public static float Lift(double a,double open,double close)
    {
        double period=4*Math.PI;
        double span=close-open, x=((a-open)%period+period)%period;
        if(x>span) return 0;
        double ramp=span*.28;
        if(x<ramp) return (float)(.5-.5*Math.Cos(Math.PI*x/ramp));
        if(x>span-ramp) return (float)(.5-.5*Math.Cos(Math.PI*(span-x)/ramp));
        return 1;
    }
    public void PreStep(float dt,Crankshaft crank)
    {
        double local=crank.TotalAngle+PhaseOffset,period=TwoStroke?2*Math.PI:4*Math.PI;
        double delta=previousLocal<0?0:local-previousLocal;
        float old=Volume,p=Pressure; Volume=VolumeAt((float)(local%(2*Math.PI)));
        float dv=Volume-old;
        InternalEnergy*=Math.Pow(old/Volume,Gas.Gamma-1);
        InternalEnergy+=(Mass*Gas.Cv*420-InternalEnergy)*Math.Min(.02,dt*12);
        Torque=delta>1e-10?(p-Gas.AmbientPressure)*dv/(float)delta:0;
        IntakeLift=TwoStroke?Window(local,3.45,5.1):Lift(local,-.18,Math.PI+ .7);
        ExhaustLift=TwoStroke?Window(local,2.4,4.8):Lift(local,3*Math.PI-.8,4*Math.PI+.2);
        Intake.Area=IntakeLift*.00065f; Exhaust.Area=ExhaustLift*.00055f;
        double intakeClose=TwoStroke?5.1:Math.PI+.7;
        if(Crossed(previousLocal,local,intakeClose,period)) fuelEnergy=AirMass/14.7*44e6*.90;
        if(crank.Rpm>RevLimit) { limiting=true; limiterTime=.06; }
        limiterTime-=dt;
        if(crank.Rpm<RevLimit-500 && limiterTime<0) limiting=false;
        if(Crossed(previousLocal,local,2*Math.PI-SparkAdvance,period))
        {
            burnTravel=0; burnFraction=0;
            if(limiting) burnTravel=-1;
        }
        BurnPulse=0;
        if(burnTravel>=0)
        {
            burnTravel+=delta;
            double x=Math.Clamp(burnTravel/BurnDuration,0,1);
            double fraction=1-Math.Exp(-6.9*x*x*x);
            double increment=Math.Max(0,fraction-burnFraction);
            InternalEnergy+=increment*fuelEnergy;
            AirMass=Math.Max(0,AirMass-increment*fuelEnergy/(44e6*.90)*14.7);
            BurnPulse=(float)(increment/Math.Max(dt,1e-9f)); burnFraction=fraction;
            if(x>=1) burnTravel=-1;
        }
        previousLocal=local;
    }
    private static float Window(double a,double start,double end)
    { a=(a%(2*Math.PI)+2*Math.PI)%(2*Math.PI); return a>start && a<end?(float)Math.Sin(Math.PI*(a-start)/(end-start)):0; }
}

public sealed class Vehicle
{
    public int Gear { get; private set; }
    public float Speed, ClutchTorque, ReflectedInertia;
    public readonly float[] Ratios=[0,2.8f,2.0f,1.55f,1.25f,1.05f,.9f];
    public void Shift(int direction) => Gear=Math.Clamp(Gear+direction,0,Ratios.Length-1);
    public float Update(float dt,Crankshaft crank,bool clutch)
    {
        float ratio=Ratios[Gear]*3.2f;
        float engage=clutch || Gear==0?0:Math.Clamp((crank.Rpm-650)/650,0,1);
        ClutchTorque=Math.Clamp((crank.Omega-Speed/.3f*ratio)*.35f,-75,75)*engage;
        Speed=MathF.Max(0,Speed+dt*(ClutchTorque*ratio/.3f-12-.4f*Speed*Speed)/220);
        ReflectedInertia=engage*.015f;
        return ClutchTorque;
    }
}
