package us.ihmc.plotting.shapes;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;

import javax.vecmath.Point2d;

import us.ihmc.plotting.Artifact;
import us.ihmc.plotting.Pose;

public class DroneArtifact extends Artifact
{
   private static final long serialVersionUID = -7196775480859274435L;
   protected Pose _pose;
   protected double width = 0.3048;
   protected double length = 0.3048;

   public DroneArtifact(String id)
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
      return this.getID() + " : " + this.getType() + "X-Y-Z (" + this._pose.getX() + "," + this._pose.getY() + "," + this._pose.getZ() + ")  YAW-PITCH-ROLL(" + this._pose.getYaw()
             + "," + this._pose.getPitch() + "," + this._pose.getRoll();
   }

   public void draw(Graphics g, int Xcenter, int Ycenter, double headingOffset, double scaleFactor)
   {
      if (_pose != null)
      {
         int x = Xcenter + ((int) Math.round(_pose.getX() * scaleFactor));
         int y = Ycenter - ((int) Math.round(_pose.getY() * scaleFactor));

         g.setColor(color);
         int w = (int) ((this.width * scaleFactor) / 2.0);
         int l = (int) ((this.length * scaleFactor) / 2.0);

         double yaw = -_pose.getYaw() + 90.0;

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

         rxc1 = (int) (x + (Math.cos(Math.toRadians(yaw)) * (xc1 - x) - Math.sin(Math.toRadians(yaw)) * (yc1 - y)));
         ryc1 = (int) (y + (Math.sin(Math.toRadians(yaw)) * (xc1 - x) + Math.cos(Math.toRadians(yaw)) * (yc1 - y)));

         rxc2 = (int) (x + (Math.cos(Math.toRadians(yaw)) * (xc2 - x) - Math.sin(Math.toRadians(yaw)) * (yc2 - y)));
         ryc2 = (int) (y + (Math.sin(Math.toRadians(yaw)) * (xc2 - x) + Math.cos(Math.toRadians(yaw)) * (yc2 - y)));

         rxc3 = (int) (x + (Math.cos(Math.toRadians(yaw)) * (xc3 - x) - Math.sin(Math.toRadians(yaw)) * (yc3 - y)));
         ryc3 = (int) (y + (Math.sin(Math.toRadians(yaw)) * (xc3 - x) + Math.cos(Math.toRadians(yaw)) * (yc3 - y)));

         rxc4 = (int) (x + (Math.cos(Math.toRadians(yaw)) * (xc4 - x) - Math.sin(Math.toRadians(yaw)) * (yc4 - y)));
         ryc4 = (int) (y + (Math.sin(Math.toRadians(yaw)) * (xc4 - x) + Math.cos(Math.toRadians(yaw)) * (yc4 - y)));

         Graphics2D g2d = (Graphics2D) g;

         g2d.setStroke(new BasicStroke(3));

         Point2d point = new Point2d(rxc1, ryc1);
         Point2d point2 = new Point2d(rxc2, ryc2);
         int radius = new Double(point.distance(point2)).intValue();

         g2d.setColor(Color.red);
         g2d.drawOval(rxc1 - radius / 2, ryc1 - radius / 2, radius, radius);
         g2d.drawOval(rxc2 - radius / 2, ryc2 - radius / 2, radius, radius);
         g2d.drawOval(rxc3 - radius / 2, ryc3 - radius / 2, radius, radius);
         g2d.drawOval(rxc4 - radius / 2, ryc4 - radius / 2, radius, radius);

         int midxFront = (rxc1 + rxc2) / 2;
         int midyFront = (ryc1 + ryc2) / 2;

         int midxright = (rxc2 + rxc3) / 2;
         int midyright = (ryc2 + ryc3) / 2;
         int midxleft = (rxc1 + rxc4) / 2;
         int midyleft = (ryc1 + ryc4) / 2;

         g2d.setColor(Color.green);
         g2d.setStroke(new BasicStroke(3));

         g2d.drawLine(midxFront, midyFront, midxright, midyright);
         g2d.drawLine(midxFront, midyFront, midxleft, midyleft);
         g2d.drawLine(midxright, midyright, midxleft, midyleft);
      }
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
