using FluidSim.Core;
namespace FluidSim.Components;

public sealed class Port(Volume0D owner, float area)
{
    public Volume0D Owner { get; } = owner;
    public float Area = area;
    public float DischargeCoefficient=.72f;
    public float MassFlowRate, EnthalpyRate;
    internal double MassDelta, AirDelta, EnergyDelta;
    internal void Reset() { MassDelta=AirDelta=EnergyDelta=0; MassFlowRate=EnthalpyRate=0; }
}

public class Volume0D
{
    public string Name { get; }
    public float Volume { get; protected set; }
    public double Mass, AirMass, InternalEnergy;
    public float HeatRelaxation = 1;
    public List<Port> Ports { get; } = [];
    public bool Infinite { get; protected set; }
    internal State BoundaryState;
    internal double OutflowThisStep;
    public float Pressure => (float)(InternalEnergy*(Gas.Gamma-1)/Volume);
    public float Temperature => (float)(InternalEnergy/(Mass*Gas.Cv));
    public State State => State.FromPT(Pressure,Temperature,0,(float)(AirMass/Mass));
    public Volume0D(string name,float volume,float pressure=Gas.AmbientPressure,float temperature=Gas.AmbientTemperature)
    { Name=name; Volume=volume; Mass=pressure*volume/(Gas.R*temperature); AirMass=Mass; InternalEnergy=Mass*Gas.Cv*temperature; }
    public Port AddPort(float area) { var p=new Port(this,area); Ports.Add(p); return p; }
    public void BeginStep() { BoundaryState=State; OutflowThisStep=0; foreach(var p in Ports) p.Reset(); }
    public virtual void UpdateState(float dt)
    {
        foreach(var p in Ports)
        {
            p.MassFlowRate=(float)(p.MassDelta/dt); p.EnthalpyRate=(float)(p.EnergyDelta/dt);
            if(!Infinite) { Mass+=p.MassDelta; AirMass+=p.AirDelta; InternalEnergy+=p.EnergyDelta; }
        }
        if(Infinite) return;
        if(Mass<=0 || InternalEnergy<=0 || !double.IsFinite(InternalEnergy)) throw new ArithmeticException($"Invalid reservoir {Name}");
        AirMass=Math.Clamp(AirMass,0,Mass);
        InternalEnergy+=(Mass*Gas.Cv*Gas.AmbientTemperature-InternalEnergy)*Math.Min(1,HeatRelaxation*dt);
    }
}
public sealed class Atmosphere() : Volume0D("Atmosphere",1000)
{ public void SetInfinite() => Infinite=true; }
