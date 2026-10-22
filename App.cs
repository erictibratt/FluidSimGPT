using System.Collections.Concurrent;
using System.Diagnostics;
using FluidSim.Audio;
using FluidSim.Scenarios;
using FluidSim.Validation;
using SFML.Graphics;
using SFML.System;
using SFML.Window;
namespace FluidSim;

public static class App
{
    public static int Run(string[] args)
    {
        if(args.Contains("--self-test")) return Checks.Run();
        if(args.Contains("--benchmark")) return Benchmark.Run();
        string option=args.FirstOrDefault(a=>a.StartsWith("--scenario="))?.Split('=')[1]??"saab";
        Scenario scenario=option switch { "saab" or "b205"=>new SaabB205Scenario(),"test"=>new TestScenario(),"helmholtz"=>new HelmholtzScenario(),"two-stroke"=>new TwoStrokeScenario(),"twin"=>new SingleCylScenario(),_=>throw new ArgumentException($"Unknown scenario: {option}") };
        if(args.Contains("--headless")) return Checks.Headless(scenario,args);
        using var window=new RenderWindow(new VideoMode(new Vector2u(1300,880)),"FluidSim | Engine flow laboratory",Styles.Titlebar|Styles.Close,State.Windowed);
        uint fps=uint.Parse(args.FirstOrDefault(a=>a.StartsWith("--fps="))?.Split('=')[1]??"30");
        window.SetFramerateLimit(Math.Clamp(fps,15u,120u)); window.SetKeyRepeatEnabled(false);
        using var dashboard=new Dashboard(); var ring=new StereoRingBuffer();
        using var stream=new EngineStream(ring); using var cancellation=new CancellationTokenSource();
        var commands=new ConcurrentQueue<Controls>(); Snapshot snapshot=scenario.Capture(); string? error=null;
        int renderRequested=1;
        float target=.45f,brake=0,lastSpeed=.25f; bool temperature=false,smokeShifted=false,smokeCaptured=false;
        window.Closed+=(_,_)=>window.Close();
        window.MouseWheelScrolled+=(_,e)=>Volatile.Write(ref stream.Speed,Math.Clamp(stream.Speed*MathF.Pow(1.25f,e.Delta),.05f,2));
        window.KeyPressed+=(_,e)=>
        {
            switch(e.Code)
            {
                case Keyboard.Key.Escape: window.Close(); break;
                case Keyboard.Key.Space:
                    if(MathF.Abs(stream.Speed-1)<.001f) stream.Speed=lastSpeed; else { lastSpeed=stream.Speed; stream.Speed=1; } break;
                case Keyboard.Key.T: temperature=!temperature; break;
                case Keyboard.Key.Q: commands.Enqueue(new(Keyboard.IsKeyPressed(Keyboard.Key.W)?target:0,brake,Keyboard.IsKeyPressed(Keyboard.Key.C),-1)); break;
                case Keyboard.Key.E: commands.Enqueue(new(Keyboard.IsKeyPressed(Keyboard.Key.W)?target:0,brake,Keyboard.IsKeyPressed(Keyboard.Key.C),1)); break;
            }
        };
        // Warm up tiered JIT before starting the time-critical audio consumer.
        for(int i=0;i<4410;i++) scenario.Process();
        // SFML queues several blocks immediately; 1,024 frames starved on startup at 1x.
        for(int i=0;i<8192;i++) { scenario.Process(); ring.Write(scenario.LastMixL,scenario.LastMixR); }
        var producer=Task.Factory.StartNew(()=>
        {
            try
            {
                while(!cancellation.IsCancellationRequested)
                {
                    while(commands.TryDequeue(out var c)) scenario.Apply(c);
                    if(ring.Count>=8192) { cancellation.Token.WaitHandle.WaitOne(1); continue; }
                    for(int i=0;i<64;i++) { scenario.Process(); ring.Write(scenario.LastMixL,scenario.LastMixR); }
                    if(Interlocked.Exchange(ref renderRequested,0)!=0) Volatile.Write(ref snapshot,scenario.Capture());
                }
            }
            catch(Exception ex) { Volatile.Write(ref error,ex.Message); Console.Error.WriteLine(ex); }
        },CancellationToken.None,TaskCreationOptions.LongRunning,TaskScheduler.Default);
        stream.Play(); var timer=Stopwatch.StartNew(); double previous=timer.Elapsed.TotalSeconds;
        double smokeSeconds=args.Contains("--smoke")?5:double.PositiveInfinity;
        try
        {
            while(window.IsOpen && timer.Elapsed.TotalSeconds<smokeSeconds)
            {
                window.DispatchEvents(); if(!window.IsOpen) break;
                double now=timer.Elapsed.TotalSeconds; float dt=(float)Math.Min(.1,now-previous); previous=now;
                bool focused=window.HasFocus(); bool Key(Keyboard.Key k)=>focused&&Keyboard.IsKeyPressed(k);
                target=Math.Clamp(target+dt*.35f*((Key(Keyboard.Key.Up)?1:0)-(Key(Keyboard.Key.Down)?1:0)),0,1);
                brake=Math.Clamp(brake+dt*.4f*((Key(Keyboard.Key.Right)?1:0)-(Key(Keyboard.Key.Left)?1:0)),0,1);
                if(args.Contains("--smoke"))
                {
                    stream.Speed=now<1?.25f:now<2?1:now<3?2:.5f;
                    temperature=now>2.5;
                    int shift=now>2&&!smokeShifted?1:0;
                    if(shift!=0) smokeShifted=true;
                    commands.Enqueue(new(now>1?.6f:0,now>4?.25f:0,now>3.5,shift));
                }
                else commands.Enqueue(new(Key(Keyboard.Key.W)?target:0,brake,Key(Keyboard.Key.C)));
                Interlocked.Exchange(ref renderRequested,1);
                dashboard.Draw(window,scenario,Volatile.Read(ref snapshot),stream.Speed,ring.Count,ring.Capacity,Interlocked.Read(ref ring.Underruns),temperature,target,Volatile.Read(ref error));
                if(args.Contains("--smoke") && now>3 && !smokeCaptured)
                { using var texture=new Texture(window.Size); texture.Update(window); using var image=texture.CopyToImage(); image.SaveToFile("smoke.png"); smokeCaptured=true; }
            }
        }
        finally { cancellation.Cancel(); stream.Stop(); producer.GetAwaiter().GetResult(); }
        if(args.Contains("--smoke")) Console.WriteLine($"Window/audio smoke: RPM {snapshot.Rpm:0}, gear {snapshot.Gear}, underruns {ring.Underruns}, repairs {snapshot.Repairs}, error {error??"none"}");
        return error==null?0:1;
    }
}
