package us.ihmc.plotting.shapes;

import java.awt.Graphics;

import us.ihmc.plotting.Artifact;
import us.ihmc.plotting.Coordinate;
import us.ihmc.plotting.Pose;

public class RobotArtifact extends Artifact
{
   private static final long serialVersionUID = 874940514060462114L;
   protected Pose _pose = new Pose(0.0, 0.0, 0.0, Coordinate.METER);
   protected double width = 16.0;

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

   public void setWidth(double width)
   {
      this.width = width;
   }

   public double getWidth()
   {
      return width;
   }


   /**
    * Must provide a draw method for plotter to render artifact
    */
   public void draw(Graphics g, int Xcenter, int Ycenter, double headingOffset, double scaleFactor)
   {
      int x = Xcenter + ((int)(Math.round(_pose.getX() * scaleFactor)));
      int y = Ycenter - ((int)(Math.round(_pose.getY() * scaleFactor)));

//    if(orientation == Plottable.X_Z){
//            x = Xcenter + (new Double(_pose.getX()* scaleFactor).intValue());
//            y = Ycenter - (new Double(_pose.getZ()* scaleFactor).intValue());
//    }
//    else if(orientation == Plottable.Y_Z){
//            x = Xcenter + (new Double(_pose.getY()* scaleFactor).intValue());
//            y = Ycenter - (new Double(_pose.getZ()* scaleFactor).intValue());
//    }

      int w1 = Double.valueOf(width).intValue();
      int w12 = Double.valueOf(width / 2.0).intValue();
      double hdg = Math.toRadians(_pose.getYaw());
      int xHDG = x + (int)Math.round(Math.sin(hdg - headingOffset) * width);
      int yHDG = y - (int)Math.round(Math.cos(hdg - headingOffset) * width);

      g.setColor(color);
      g.drawOval((x - w12), (y - w12), w1, w1);
      g.drawLine(x, y, xHDG, yHDG);
   }

   public void drawLegend(Graphics g, int Xcenter, int Ycenter, double scaleFactor)
   {
   }
   
   public void drawHistory(Graphics g, int Xcenter, int Ycenter, double scaleFactor)
   {
      throw new RuntimeException("Not implemented!");
   }
   
   public void takeHistorySnapshot()
   {
      throw new RuntimeException("Not implemented!");
   }

}
