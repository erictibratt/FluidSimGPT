using FluidSim.Components;
namespace FluidSim.Core;

public sealed class Solver(PipeSystem pipes, BoundarySystem boundaries,List<Volume0D> components)
{
    public bool EnableProfiling;
    public long BoundaryTicks,PipeTicks,ComponentTicks,Steps;
    private int cachedPipeCount=-1;
    private float minimumDx;
    public int MinimumSubsteps=2;
    public int LastSubsteps { get; private set; }
    public void Step(float dt)
    {
        foreach(var c in components) c.BeginStep();
        if(cachedPipeCount!=pipes.Pipes.Count)
        {
            minimumDx=float.MaxValue;
            foreach(var p in pipes.Pipes) minimumDx=MathF.Min(minimumDx,p.Dx);
            cachedPipeCount=pipes.Pipes.Count;
        }
        float dx=minimumDx;
        LastSubsteps=Math.Max(MinimumSubsteps,(int)MathF.Ceiling(dt*pipes.MaxWaveSpeed()/(.8f*dx)));
        float sub=dt/LastSubsteps;
        if(!EnableProfiling)
        {
            for(int j=0;j<LastSubsteps;j++) { boundaries.Resolve(sub); pipes.Advance(sub,boundaries); }
        }
        else for(int j=0;j<LastSubsteps;j++)
        {
            long start=System.Diagnostics.Stopwatch.GetTimestamp(); boundaries.Resolve(sub);
            long now=System.Diagnostics.Stopwatch.GetTimestamp(); BoundaryTicks+=now-start;
            pipes.Advance(sub,boundaries); PipeTicks+=System.Diagnostics.Stopwatch.GetTimestamp()-now;
        }
        long componentStart=EnableProfiling?System.Diagnostics.Stopwatch.GetTimestamp():0;
        foreach(var c in components) c.UpdateState(dt);
        if(EnableProfiling) { ComponentTicks+=System.Diagnostics.Stopwatch.GetTimestamp()-componentStart; Steps++; }
    }
}
