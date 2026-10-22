namespace FluidSim.Core;

public sealed class DelayLine(int size)
{
    private readonly float[] data=new float[size]; private int position;
    public float Read(int delay) => data[(position-delay+data.Length)%data.Length];
    public void Push(float v) { data[position]=v; position=(position+1)%data.Length; }
    public float Tick(float input) { float v=data[position]; Push(input); return v; }
}
public sealed class RadiationSource(float pan,float distance,float gain)
{
    private float smooth,previous,absorb,shadow;
    private readonly DelayLine delay=new(128);
    public float Mono { get; private set; }
    public (float L,float R) Process(float flow)
    {
        smooth+=.16f*(flow-smooth);
        float derivative=(smooth-previous)*44100; previous=smooth;
        Mono=MathF.Tanh(derivative*gain/(1+distance))*.65f;
        absorb+=(.55f/(1+distance*.15f))*(Mono-absorb);
        delay.Push(absorb);
        float far=delay.Read((int)(MathF.Abs(pan)*25)); shadow+=.18f*(far-shadow);
        return pan<0?(absorb,shadow*(1+pan*.4f)):(shadow*(1-pan*.4f),absorb);
    }
}
public sealed class Reverb
{
    private readonly DelayLine early=new(8192);
    private readonly DelayLine[] lines=[new(1493),new(1601),new(1747),new(1867),new(1999),new(2137),new(2269),new(2411)];
    private readonly float[] low=new float[8], high=new float[8], output=new float[8];
    private float envelope;
    public (float L,float R) Process(float input)
    {
        float predelay=early.Read(661),reflections=.3f*early.Read(1103)+.2f*early.Read(2003)+.1f*early.Read(3203);
        early.Push(input); float sum=0;
        for(int i=0;i<8;i++) { output[i]=lines[i].Read(0); sum+=output[i]; }
        envelope+=.002f*(MathF.Abs(input)-envelope);
        float transient=Math.Clamp(MathF.Abs(input)-envelope,0,.7f);
        float l=0,r=0;
        for(int i=0;i<8;i++)
        {
            float v=output[i]-.25f*sum; // Householder orthogonal feedback matrix
            low[i]+=.025f*(v-low[i]); high[i]+=.45f*(v-high[i]);
            float feedback=.86f*low[i]+.79f*(high[i]-low[i])+.52f*(v-high[i]);
            lines[i].Push(predelay*.12f+feedback*(1-transient*.15f));
            l+=output[i]*(i%2==0?.23f:-.13f); r+=output[i]*(i%3==0?-.18f:.21f);
        }
        return (l+reflections,r+reflections*.88f);
    }
}
public sealed class SoundMixer
{
    private readonly RadiationSource intake=new(-.65f,1.4f,.018f), exhaust=new(.65f,2,.025f);
    private readonly Reverb reverb=new();
    public float Wet=.24f, Mechanical=.035f;
    public (float L,float R) Process(float inlet,float outlet,double angle,float rpm,float burn)
    {
        var a=intake.Process(inlet); var b=exhaust.Process(outlet);
        var wet=reverb.Process(intake.Mono+exhaust.Mono);
        float mechanical=Mechanical*(MathF.Sin((float)angle*2)*Math.Clamp(rpm/3000,0,1)+MathF.Tanh(burn*.002f));
        return (Math.Clamp(MathF.Tanh(a.L+b.L)+wet.L*Wet+mechanical,-1,1),Math.Clamp(MathF.Tanh(a.R+b.R)+wet.R*Wet+mechanical*.8f,-1,1));
    }
}
