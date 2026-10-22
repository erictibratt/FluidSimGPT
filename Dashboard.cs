using FluidSim.Scenarios;
using SFML.Graphics;
using SFML.System;
namespace FluidSim;

public sealed class Dashboard : IDisposable
{
    private readonly Font font=new(Path.Combine(AppContext.BaseDirectory,"Assets","Inter.ttf"));
    private readonly Color ink=new(224,235,243), muted=new(121,145,165), accent=new(78,225,187);
    private readonly float[] history=new float[440]; private int cursor;
    private readonly Text text;
    private readonly RectangleShape rectangle=new(new Vector2f());
    private readonly Vertex[] grid,legend,wave=new Vertex[440];
    private Vertex[] pipeVertices=[];
    public Dashboard()
    {
        text=new Text(font);
        var vertices=new List<Vertex>();
        for(int x=28;x<1300;x+=32) { vertices.Add(new(new Vector2f(x,215),new Color(17,28,40))); vertices.Add(new(new Vector2f(x,560),new Color(17,28,40))); }
        for(int y=215;y<560;y+=32) { vertices.Add(new(new Vector2f(28,y),new Color(17,28,40))); vertices.Add(new(new Vector2f(1260,y),new Color(17,28,40))); }
        grid=vertices.ToArray(); legend=new Vertex[180*6];
        for(int i=0;i<180;i++) Quad(legend,i*6,new(34+i*2,568),new(36+i*2,568),new(36+i*2,576),new(34+i*2,576),Heat(i/179f));
    }
    private void Label(RenderWindow w,string value,float x,float y,uint size=14,Color? color=null)
    {
        text.DisplayedString=value; text.Position=new(x,y); text.CharacterSize=size; text.FillColor=color??ink; w.Draw(text);
    }
    private void Box(RenderWindow w,float x,float y,float width,float height,Color color)
    { rectangle.Size=new(width,height); rectangle.Position=new(x,y); rectangle.FillColor=color; w.Draw(rectangle); }
    private static void Quad(Vertex[] vertices,int i,Vector2f a,Vector2f b,Vector2f c,Vector2f d,Color color)
    { vertices[i]=new(a,color); vertices[i+1]=new(b,color); vertices[i+2]=new(c,color); vertices[i+3]=new(a,color); vertices[i+4]=new(c,color); vertices[i+5]=new(d,color); }
    private void PreparePipes(Scenario scenario)
    {
        pipeVertices=new Vertex[scenario.Pipes.Rho.Length*6];
        foreach(var p in scenario.Pipes.Pipes)
        {
            var delta=new Vector2f(p.X2-p.X1,p.Y2-p.Y1);
            float length=MathF.Sqrt(delta.X*delta.X+delta.Y*delta.Y);
            var normal=new Vector2f(-delta.Y/length*4,delta.X/length*4);
            for(int i=0;i<p.Count;i++)
            {
                var a=new Vector2f(p.X1,p.Y1)+delta*((float)i/p.Count);
                var b=new Vector2f(p.X1,p.Y1)+delta*((float)(i+1)/p.Count);
                Quad(pipeVertices,(p.Start+i)*6,a-normal,b-normal,b+normal,a+normal,Color.White);
            }
        }
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
        w.Draw(grid,PrimitiveType.Lines);
        Label(w,"FLUIDSIM",28,22,28); Label(w,"COMPRESSIBLE FLOW / ENGINE ACOUSTICS",230,33,12,muted); Label(w,scenario.Name,900,32,16,accent);
        string[] titles=["ENGINE SPEED","THROTTLE","TRANSMISSION","ROAD SPEED","PLAYBACK"];
        string[] values=[$"{s.Rpm:0} rpm",$"{s.Throttle*100:0}%",s.Gear==0?"NEUTRAL":$"GEAR {s.Gear}",$"{s.Speed*3.6f:0.0} km/h",$"{speed:0.00} x"];
        for(int i=0;i<5;i++) { float x=28+i*254; Box(w,x,83,240,87,new(19,31,44)); Label(w,titles[i],x+16,95,11,muted); Label(w,values[i],x+16,118,25,i==0?accent:ink); }
        Label(w,"01   LIVE FLOW NETWORK",28,191,12,muted); Label(w,temperature?"TEMPERATURE [K]  /  T TO CHANGE":"PRESSURE [bar]  /  T TO CHANGE",870,191,12,muted);
        Label(w,scenario.Description,28,215,11,muted);
        if(pipeVertices.Length!=s.Pressure.Length*6) PreparePipes(scenario);
        for(int i=0;i<s.Pressure.Length;i++)
        {
            float v=temperature?(s.Temperature[i]-280)/1100:(s.Pressure[i]/100000-.45f)/1.5f;
            var color=Heat(v); for(int k=0;k<6;k++) pipeVertices[i*6+k].Color=color;
        }
        w.Draw(pipeVertices,PrimitiveType.Triangles);
        foreach(var p in scenario.Pipes.Pipes)
        {
            if(s.CylinderPressure.Length<4 || p.Name.StartsWith("RUNNER") || p.Name.StartsWith("SECONDARY"))
                Label(w,p.Name,(p.X1+p.X2)*.5f-35,(p.Y1+p.Y2)*.5f-18,9,muted);
        }
        foreach(var v in scenario.VolumeVisuals)
        {
            Box(w,v.X,v.Y,v.Width,v.Height,new(35,53,65));
            Label(w,v.Name,v.X-8,v.Y+v.Height+7,9,muted);
        }
        for(int i=0;i<s.CylinderPressure.Length;i++)
        {
            var layout=scenario.CylinderVisuals[i]; float x=layout.X,y=layout.Y,width=layout.Width,height=layout.Height;
            Box(w,x,y,width,height,new(25,39,52)); Box(w,x+6,y+18,width-12,height-24,Heat(Math.Clamp(s.CylinderPressure[i]/5000000,0,1)));
            var c=scenario.Cylinders[i]; float clearance=c.Displacement/(c.Compression-1);
            float piston=8+Math.Clamp((s.CylinderVolume[i]-clearance)/c.Displacement,0,1)*(width-34);
            Box(w,x+piston,y+18,10,height-24,new(193,204,211)); Label(w,$"{i+1} / {s.CylinderPressure[i]/100000:0.0} bar",x+4,y-15,10);
            Box(w,x+6,y+5,40*s.IntakeLift[i]+2,4,accent); Box(w,x+width-48,y+5,40*s.ExhaustLift[i]+2,4,new(255,139,89));
        }
        w.Draw(legend,PrimitiveType.Triangles);
        Label(w,temperature?"280        830        1380 K":"0.45        1.2         1.95 bar",34,584,11,muted);
        Label(w,$"CFL SUBSTEPS {s.Substeps}   /   NUMERICAL REPAIRS {s.Repairs}",760,578,11,muted);
        Box(w,28,626,800,170,new(16,27,39)); Box(w,844,626,424,170,new(16,27,39)); Label(w,"02   AUDIO / LEFT CHANNEL HISTORY",44,640,11,muted);
        history[cursor]=s.AudioL; cursor=(cursor+1)%history.Length;
        for(int i=0;i<history.Length;i++) wave[i]=new(new Vector2f(44+i*1.72f,723-history[(cursor+i)%history.Length]*52),accent);
        w.Draw(wave,PrimitiveType.LineStrip);
        Label(w,"03   AUDIO PIPELINE",862,640,11,muted); Label(w,"44,100 Hz  /  STEREO  /  512 FRAMES",862,671,13);
        Box(w,862,705,384,7,new(37,51,64)); Box(w,862,705,384*Math.Clamp((float)fill/capacity,0,1),7,accent);
        Label(w,$"QUEUE {fill} / {capacity}    UNDERRUNS {underruns}",862,725,12,muted); Label(w,$"TARGET {target*100:0}%    BRAKE {s.Brake*100:0}%",862,752,12,muted);
        Label(w,"W  THROTTLE     UP/DOWN  TARGET     LEFT/RIGHT  BRAKE     C  CLUTCH     Q/E  SHIFT",28,817,12,muted);
        Label(w,"WHEEL  PLAYBACK     SPACE  REAL TIME     T  HEATMAP     ESC  EXIT",28,843,12,muted);
        if(failure!=null) { Box(w,28,532,1220,28,new(98,28,38)); Label(w,failure,38,535,13); }
        w.Display();
    }
    public void Dispose() { text.Dispose(); rectangle.Dispose(); font.Dispose(); }
}
