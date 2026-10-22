namespace FluidSim.Audio;
public sealed class StereoRingBuffer
{
    private readonly float[] left,right;
    private readonly int mask;
    private long read,write;
    private double fraction;
    public int Capacity => left.Length;
    public int Count => (int)(Volatile.Read(ref write)-Volatile.Read(ref read));
    public long Underruns;
    public StereoRingBuffer(int capacity=16384)
    {
        if(capacity<4 || (capacity&(capacity-1))!=0) throw new ArgumentException("Capacity must be a power of two.");
        left=new float[capacity]; right=new float[capacity]; mask=capacity-1;
    }
    public bool Write(float l,float r)
    {
        long w=write;
        if(w-Volatile.Read(ref read)>=Capacity) return false;
        left[(int)(w&mask)]=l; right[(int)(w&mask)]=r; Volatile.Write(ref write,w+1); return true;
    }
    public bool Read(float speed,out float l,out float r)
    {
        long rd=read; int available=(int)(Volatile.Read(ref write)-rd);
        double next=fraction+speed; int consume=(int)next;
        if(available<Math.Max(2,consume+1)) { l=r=0; Interlocked.Increment(ref Underruns); return false; }
        int i=(int)(rd&mask),j=(int)((rd+1)&mask);
        l=left[i]+(left[j]-left[i])*(float)fraction; r=right[i]+(right[j]-right[i])*(float)fraction;
        fraction=next-consume; Volatile.Write(ref read,rd+consume); return true;
    }
}
