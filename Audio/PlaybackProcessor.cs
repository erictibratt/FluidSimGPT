namespace FluidSim.Audio;

// Shared by the native callback and offline regression checks; no callback allocations.
public sealed class PlaybackProcessor(StereoRingBuffer ring)
{
    // SFML.Net's temporary fixed scope ends before native code copies the samples.
    public short[] Samples { get; } = GC.AllocateArray<short>(1024,pinned:true);
    private float previousL,previousR,dcL,dcR,lastL,lastR,recovery;
    private float speed=1;
    public void Fill(float targetSpeed)
    {
        targetSpeed=Math.Clamp(targetSpeed,.05f,2);
        for(int i=0;i<512;i++)
        {
            // Continuous rate changes avoid a pitch/phase step at callback boundaries.
            speed+=.002f*(targetSpeed-speed);
            float l,r;
            if(ring.Read(speed,out float inputL,out float inputR))
            {
                recovery=MathF.Min(1,recovery+1f/128);
                l=inputL*recovery; r=inputR*recovery;
            }
            else
            {
                recovery=0;
                // Fade the previous output instead of jumping to hard zero on starvation.
                l=lastL*.97f; r=lastR*.97f;
            }
            lastL=l; lastR=r;
            // About 10 Hz: remove DC without deleting the low engine fundamental.
            dcL=l-previousL+.998576f*dcL; dcR=r-previousR+.998576f*dcR; previousL=l; previousR=r;
            Samples[i*2]=(short)(Math.Clamp(dcL,-1,1)*32767);
            Samples[i*2+1]=(short)(Math.Clamp(dcR,-1,1)*32767);
        }
    }
}
