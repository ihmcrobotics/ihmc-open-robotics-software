package us.ihmc.plotting.shapes;

import java.awt.Graphics;

import us.ihmc.plotting.Artifact;
import us.ihmc.plotting.Pose;

public class PioneerRobotArtifact extends Artifact
{
   private static final long serialVersionUID = -7196775480859274435L;
   protected Pose _pose;
   protected double width = 0.4826;
   protected double length = 0.5334;

   public PioneerRobotArtifact(String id)
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

   // public void draw(Graphics g, int Xcenter, int Ycenter, double
   // scaleFactor)
   // {
   // int x = Xcenter + (new Double(_pose.getX() * scaleFactor).intValue());
   // int y = Ycenter - (new Double(_pose.getY() * scaleFactor).intValue());
   //
   // // if(orientation == Plottable.X_Z){
   // // x = Xcenter + (new Double(_pose.getX()* scaleFactor).intValue());
   // // y = Ycenter - (new Double(_pose.getZ()* scaleFactor).intValue());
   // // }
   // // else if(orientation == Plottable.Y_Z){
   // // x = Xcenter + (new Double(_pose.getY()* scaleFactor).intValue());
   // // y = Ycenter - (new Double(_pose.getZ()* scaleFactor).intValue());
   // // }
   //
   // int w1 = 16;
   // double w2 = 16.0;
   // int w12 = 8;
   // double hdg = Math.toRadians(_pose.getYaw());
   // int xHDG = x + new Double(Math.sin(hdg) * w2).intValue();
   // int yHDG = y - new Double(Math.cos(hdg) * w2).intValue();
   //
   // g.setColor(color);
   // g.drawOval((x - w12), (y - w12), w1, w1);
   // g.drawLine(x, y, xHDG, yHDG);
   // }
   public void draw(Graphics g, int Xcenter, int Ycenter, double headingOffset, double scaleFactor)
   {
      int x = Xcenter + ((int)Math.round(_pose.getX() * scaleFactor));
      int y = Ycenter - ((int)Math.round(_pose.getY() * scaleFactor));

      g.setColor(color);
      int w = (int) ((this.width * scaleFactor) / 2.0);
      int l = (int) ((this.length * scaleFactor) / 2.0);

      _pose.getYaw();

      int xc1, yc1, xc2, yc2, xc3, yc3, xc4, yc4;

      xc1 = x - (w / 2);
      yc1 = y - (l / 2);

      xc2 = x + (w / 2);
      yc2 = y - (l / 2);

      xc3 = x + (w / 2);
      yc3 = y + (l / 2);

      xc4 = x - (w / 2);
      yc4 = y + (l / 2);

      int rxc1, ryc1, rxc2, ryc2, rxc3, ryc3, rxc4, ryc4;

      rxc1 = (int) (x + (Math.cos(Math.toRadians(_pose.getYaw())) * (xc1 - x) - Math.sin(Math.toRadians(_pose.getYaw())) * (yc1 - y)));
      ryc1 = (int) (y + (Math.sin(Math.toRadians(_pose.getYaw())) * (xc1 - x) + Math.cos(Math.toRadians(_pose.getYaw())) * (yc1 - y)));

      rxc2 = (int) (x + (Math.cos(Math.toRadians(_pose.getYaw())) * (xc2 - x) - Math.sin(Math.toRadians(_pose.getYaw())) * (yc2 - y)));
      ryc2 = (int) (y + (Math.sin(Math.toRadians(_pose.getYaw())) * (xc2 - x) + Math.cos(Math.toRadians(_pose.getYaw())) * (yc2 - y)));

      rxc3 = (int) (x + (Math.cos(Math.toRadians(_pose.getYaw())) * (xc3 - x) - Math.sin(Math.toRadians(_pose.getYaw())) * (yc3 - y)));
      ryc3 = (int) (y + (Math.sin(Math.toRadians(_pose.getYaw())) * (xc3 - x) + Math.cos(Math.toRadians(_pose.getYaw())) * (yc3 - y)));

      rxc4 = (int) (x + (Math.cos(Math.toRadians(_pose.getYaw())) * (xc4 - x) - Math.sin(Math.toRadians(_pose.getYaw())) * (yc4 - y)));
      ryc4 = (int) (y + (Math.sin(Math.toRadians(_pose.getYaw())) * (xc4 - x) + Math.cos(Math.toRadians(_pose.getYaw())) * (yc4 - y)));

      int[] pointsX = {rxc1, rxc2, rxc3, rxc4};
      int[] pointsY = {ryc1, ryc2, ryc3, ryc4};

      g.fillPolygon(pointsX, pointsY, 4);



      // g.fillRoundRect(x - (w / 2), y - (l / 2), w, l, 1, 1);

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
