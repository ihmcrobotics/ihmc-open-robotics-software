package us.ihmc.plotting.shapes;

import java.awt.Graphics;

import us.ihmc.plotting.Artifact;
import us.ihmc.plotting.Pose;

public class RobotArtifact extends Artifact
{
   /**
    * 
    */
   private static final long serialVersionUID = 874940514060462114L;
   protected Pose _pose;

   public RobotArtifact(String id)
   {
      super(id);
      setType("robot");
      setLevel(4);
   }

   public void setPose(Pose pose)
   {
      _pose = pose;
   }

   public Pose getPose()
   {
      return _pose;
   }

   public String describe()
   {
      return this.getID() + " : " + this.getType() + " " + this._pose.getX() + "," + this._pose.getY() + "," + this._pose.getZ() + "," + this._pose.getYaw()
             + "," + this._pose.getPitch() + "," + this._pose.getRoll();
   }

   /**
    * Must provide a draw method for plotter to render artifact
    */
   public void draw(Graphics g, int Xcenter, int Ycenter, double scaleFactor)
   {
      int x = Xcenter + (new Double(_pose.getX() * scaleFactor).intValue());
      int y = Ycenter - (new Double(_pose.getY() * scaleFactor).intValue());

//    if(orientation == Plottable.X_Z){
//            x = Xcenter + (new Double(_pose.getX()* scaleFactor).intValue());
//            y = Ycenter - (new Double(_pose.getZ()* scaleFactor).intValue());
//    }
//    else if(orientation == Plottable.Y_Z){
//            x = Xcenter + (new Double(_pose.getY()* scaleFactor).intValue());
//            y = Ycenter - (new Double(_pose.getZ()* scaleFactor).intValue());
//    }

      int w1 = 16;
      double w2 = 16.0;
      int w12 = 8;
      double hdg = Math.toRadians(_pose.getYaw());
      int xHDG = x + new Double(Math.sin(hdg) * w2).intValue();
      int yHDG = y - new Double(Math.cos(hdg) * w2).intValue();

      g.setColor(color);
      g.drawOval((x - w12), (y - w12), w1, w1);
      g.drawLine(x, y, xHDG, yHDG);
   }

   public void drawLegend(Graphics g, int Xcenter, int Ycenter, double scaleFactor)
   {
   }

}
