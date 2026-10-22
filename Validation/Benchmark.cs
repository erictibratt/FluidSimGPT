using System.Diagnostics;
using System.Numerics;
using FluidSim.Core;
using FluidSim.Scenarios;
namespace FluidSim.Validation;

public static class Benchmark
{
    public static int Run()
    {
        Console.WriteLine($"Hardware SIMD: {Vector.IsHardwareAccelerated}; float lanes: {Vector<float>.Count}");
        // Warm both code paths before timing; compare identical fresh states on each run.
        MeasurePipes(false,3000); MeasurePipes(true,3000);
        double scalar=0,simd=0;
        for(int trial=0;trial<3;trial++)
        {
            // Alternate order to reduce CPU warmup/frequency bias.
            if(trial%2==0) { scalar+=MeasurePipes(false,30000); simd+=MeasurePipes(true,30000); }
            else { simd+=MeasurePipes(true,30000); scalar+=MeasurePipes(false,30000); }
        }
        Console.WriteLine($"70-cell pipe loop, 90,000 substeps: scalar {scalar:0.000}s; SIMD {simd:0.000}s; speedup {scalar/simd:0.00}x");
        Console.WriteLine("Timings exclude rendering/audio, include boundary resolution, and use the default seven 10-cell segments.");
        return 0;
    }
    private static double MeasurePipes(bool vector,int steps)
    {
        var pipes=new PipeSystem { UseSimd=vector }; var boundaries=new BoundarySystem(pipes);
        for(int j=0;j<7;j++)
        {
            var p=pipes.Add($"pipe {j}",.5f,.001f,0,0,1,0);
            boundaries.Connect(p,false); boundaries.Connect(p,true);
            for(int i=0;i<p.Count;i++) pipes.Set(p.Start+i,State.FromPT(100000+5000*MathF.Sin(i*.4f+j),300,10*MathF.Sin(i),.6f));
        }
        var timer=Stopwatch.StartNew();
        for(int i=0;i<steps;i++) { boundaries.Resolve(Scenario.Dt/2); pipes.Advance(Scenario.Dt/2,boundaries); }
        timer.Stop();
        if(pipes.Repairs!=0) throw new ArithmeticException("Benchmark required numerical repairs.");
        return timer.Elapsed.TotalSeconds;
    }
}
