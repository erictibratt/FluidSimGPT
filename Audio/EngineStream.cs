using SFML.Audio;
using SFML.System;
namespace FluidSim.Audio;
public sealed class EngineStream : SoundStream
{
    private readonly StereoRingBuffer ring;
    private readonly short[] pcm=new short[1024];
    private float previousL,previousR,dcL,dcR;
    public float Speed=.25f;
    public EngineStream(StereoRingBuffer ring)
    {
        this.ring=ring;
        Initialize(2,44100,[SoundChannel.FrontLeft,SoundChannel.FrontRight]); Volume=65;
    }
    protected override bool OnGetData(out short[] samples)
    {
        float speed=Volatile.Read(ref Speed);
        for(int i=0;i<512;i++)
        {
            ring.Read(speed,out float l,out float r);
            dcL=l-previousL+.995f*dcL; dcR=r-previousR+.995f*dcR; previousL=l; previousR=r;
            pcm[i*2]=(short)(Math.Clamp(dcL,-1,1)*32767); pcm[i*2+1]=(short)(Math.Clamp(dcR,-1,1)*32767);
        }
        samples=pcm; return true;
    }
    protected override void OnSeek(Time offset) { }
}
