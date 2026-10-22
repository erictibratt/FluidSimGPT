using SFML.Audio;
using SFML.System;
namespace FluidSim.Audio;
public sealed class EngineStream : SoundStream
{
    private readonly PlaybackProcessor playback;
    public float Speed=1;
    public EngineStream(StereoRingBuffer ring)
    {
        playback=new(ring);
        Initialize(2,44100,[SoundChannel.FrontLeft,SoundChannel.FrontRight]); Volume=65;
    }
    protected override bool OnGetData(out short[] samples)
    {
        playback.Fill(Volatile.Read(ref Speed));
        samples=playback.Samples; return true;
    }
    protected override void OnSeek(Time offset) { }
}
