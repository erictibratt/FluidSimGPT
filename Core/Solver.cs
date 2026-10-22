using FluidSim.Components;
namespace FluidSim.Core;

public sealed class Solver(PipeSystem pipes, BoundarySystem boundaries,List<Volume0D> components)
{
    public int MinimumSubsteps=2;
    public int LastSubsteps { get; private set; }
    public void Step(float dt)
    {
        foreach(var c in components) c.BeginStep();
        float dx=float.MaxValue;
        foreach(var p in pipes.Pipes) dx=MathF.Min(dx,p.Dx);
        LastSubsteps=Math.Max(MinimumSubsteps,(int)MathF.Ceiling(dt*pipes.MaxWaveSpeed()/(.8f*dx)));
        float sub=dt/LastSubsteps;
        for(int j=0;j<LastSubsteps;j++) { boundaries.Resolve(sub); pipes.Advance(sub,boundaries); }
        foreach(var c in components) c.UpdateState(dt);
    }
}
