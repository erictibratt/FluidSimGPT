using FluidSim.Scenarios;
using SFML.Graphics;
using SFML.System;
namespace FluidSim;

public sealed class Dashboard : IDisposable
{
    private readonly Font font=new(Path.Combine(AppContext.BaseDirectory,"Assets","Inter.ttf"));
    private readonly Color ink=new(224,235,243), muted=new(121,145,165), accent=new(78,225,187);
    private readonly float[] history=new float[440]; private int cursor;
    private void Label(RenderWindow w,string value,float x,float y,uint size=14,Color? color=null)
    { using var t=new Text(font,value,size) { Position=new(x,y),FillColor=color??ink }; w.Draw(t); }
    private static void Box(RenderWindow w,float x,float y,float width,float height,Color color)
    { using var r=new RectangleShape(new Vector2f(width,height)) { Position=new(x,y),FillColor=color }; w.Draw(r); }
    private static void Line(RenderWindow w,float x,float y,float xx,float yy,float thickness,Color c)
    {
        float dx=xx-x,dy=yy-y;
        using var r=new RectangleShape(new Vector2f(MathF.Sqrt(dx*dx+dy*dy),thickness)) { Position=new(x,y),Rotation=MathF.Atan2(dy,dx)*180/MathF.PI,FillColor=c };
        w.Draw(r);
    }
    private static Color Heat(float v)
    {
        v=Math.Clamp(v,0,1);
        return v<.5f?new((byte)(40+v*70),(byte)(100+v*240),(byte)(225-v*110)):
            new((byte)(75+(v-.5f)*360),(byte)(220-(v-.5f)*310),(byte)(170-(v-.5f)*280));
    }
    public void Draw(RenderWindow w,Scenario scenario,Snapshot s,float speed,int fill,int capacity,long underruns,bool temperature,float target,string? failure)
    {
        w.Clear(new(10,17,26));
        for(int x=28;x<1300;x+=32) Line(w,x,190,x,610,1,new(17,28,40));
        for(int y=190;y<620;y+=32) Line(w,28,y,1260,y,1,new(17,28,40));
        Label(w,"FLUIDSIM",28,22,28); Label(w,"COMPRESSIBLE FLOW / ENGINE ACOUSTICS",230,33,12,muted); Label(w,scenario.Name,900,32,16,accent);
        string[] titles=["ENGINE SPEED","THROTTLE","TRANSMISSION","ROAD SPEED","PLAYBACK"];
        string[] values=[$"{s.Rpm:0} rpm",$"{s.Throttle*100:0}%",s.Gear==0?"NEUTRAL":$"GEAR {s.Gear}",$"{s.Speed*3.6f:0.0} km/h",$"{speed:0.00} x"];
        for(int i=0;i<5;i++) { float x=28+i*254; Box(w,x,83,240,87,new(19,31,44)); Label(w,titles[i],x+16,95,11,muted); Label(w,values[i],x+16,118,25,i==0?accent:ink); }
        Label(w,"01   LIVE FLOW NETWORK",28,191,12,muted); Label(w,temperature?"TEMPERATURE [K]  /  T TO CHANGE":"PRESSURE [bar]  /  T TO CHANGE",870,191,12,muted);
        foreach(var p in scenario.Pipes.Pipes)
        {
            for(int j=0;j<p.Count;j++)
            {
                float a=(float)j/p.Count,b=(float)(j+1)/p.Count;
                float v=temperature?(s.Temperature[p.Start+j]-280)/1100:(s.Pressure[p.Start+j]/100000-.45f)/1.5f;
                Line(w,p.X1+(p.X2-p.X1)*a,p.Y1+(p.Y2-p.Y1)*a,p.X1+(p.X2-p.X1)*b,p.Y1+(p.Y2-p.Y1)*b,12,Heat(v));
            }
            Label(w,p.Name,(p.X1+p.X2)*.5f-35,(p.Y1+p.Y2)*.5f-27,10,muted);
        }
        if(s.CylinderPressure.Length>0)
        {
            Box(w,310,290,40,65,new(35,62,71)); Label(w,"PLENUM",292,368,10,muted);
            Box(w,820,330,35,65,new(69,47,45)); Label(w,"MERGE",818,404,10,muted); Box(w,1020,337,25,50,new(45,53,65));
        }
        for(int i=0;i<s.CylinderPressure.Length;i++)
        {
            float y=i==0?231:441;
            Box(w,510,y,140,85,new(25,39,52)); Box(w,519,y+25,122,42,Heat(Math.Clamp(s.CylinderPressure[i]/5000000,0,1)));
            float piston=15+Math.Clamp((s.CylinderVolume[i]-.00003f)/.0003f,0,1)*60;
            Box(w,520+piston,y+26,12,40,new(193,204,211)); Label(w,$"CYL {i+1}  {s.CylinderPressure[i]/100000:0.0} bar",514,y-24,12);
            Box(w,516,y+5,50*s.IntakeLift[i]+2,5,accent); Box(w,586,y+5,50*s.ExhaustLift[i]+2,5,new(255,139,89));
        }
        for(int i=0;i<180;i++) Box(w,34+i*2,568,2,8,Heat(i/179f));
        Label(w,temperature?"280        830        1380 K":"0.45        1.2         1.95 bar",34,584,11,muted);
        Label(w,$"CFL SUBSTEPS {s.Substeps}   /   NUMERICAL REPAIRS {s.Repairs}",760,578,11,muted);
        Box(w,28,626,800,170,new(16,27,39)); Box(w,844,626,424,170,new(16,27,39)); Label(w,"02   AUDIO / LEFT CHANNEL HISTORY",44,640,11,muted);
        history[cursor]=s.AudioL; cursor=(cursor+1)%history.Length;
        for(int i=1;i<history.Length;i++) Line(w,44+(i-1)*1.72f,723-history[(cursor+i-1)%history.Length]*52,44+i*1.72f,723-history[(cursor+i)%history.Length]*52,1.5f,accent);
        Label(w,"03   AUDIO PIPELINE",862,640,11,muted); Label(w,"44,100 Hz  /  STEREO  /  512 FRAMES",862,671,13);
        Box(w,862,705,384,7,new(37,51,64)); Box(w,862,705,384*Math.Clamp((float)fill/capacity,0,1),7,accent);
        Label(w,$"QUEUE {fill} / {capacity}    UNDERRUNS {underruns}",862,725,12,muted); Label(w,$"TARGET {target*100:0}%    BRAKE {s.Brake*100:0}%",862,752,12,muted);
        Label(w,"W  THROTTLE     UP/DOWN  TARGET     LEFT/RIGHT  BRAKE     C  CLUTCH     Q/E  SHIFT",28,817,12,muted);
        Label(w,"WHEEL  PLAYBACK     SPACE  REAL TIME     T  HEATMAP     ESC  EXIT",28,843,12,muted);
        if(failure!=null) { Box(w,28,532,1220,28,new(98,28,38)); Label(w,failure,38,535,13); }
        w.Display();
    }
    public void Dispose() => font.Dispose();
}
