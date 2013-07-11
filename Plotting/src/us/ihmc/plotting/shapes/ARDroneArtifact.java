package us.ihmc.plotting.shapes;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.geom.AffineTransform;

import us.ihmc.plotting.Artifact;
import us.ihmc.plotting.Pose;

public class ARDroneArtifact extends Artifact
{
   private static final long serialVersionUID = -7196775480859274435L;
   protected Pose _pose;

   protected double propellerOffset = 0.1397;
   protected double noseOffset = 0.3397;
   protected double width = propellerOffset * 4;
   protected double length = propellerOffset * 4;


   public ARDroneArtifact(String id)
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
      return this.getID() + " : " + this.getType() + "X-Y-Z (" + this._pose.getX() + "," + this._pose.getY() + "," + this._pose.getZ() + ")  YAW-PITCH-ROLL("
             + this._pose.getYaw() + "," + this._pose.getPitch() + "," + this._pose.getRoll();
   }

   public void draw(Graphics g, int Xcenter, int Ycenter, double headingOffset, double scaleFactor)
   {
      if (_pose != null)
      {
         int x = Xcenter + ((int) Math.round(_pose.getX() * scaleFactor));
         int y = Ycenter - ((int) Math.round(-_pose.getY() * scaleFactor));

         double lengthPercentage = (-Math.abs(_pose.getPitch()) + 90) / 90;
         double widthPercentage = (-Math.abs(_pose.getRoll()) + 90) / 90;

         double transformX = Xcenter + ((int) Math.round((_pose.getX()) * scaleFactor));
         double transformY = Ycenter + ((int) Math.round((-_pose.getY()) * scaleFactor));;


         int frontNosex = ((int) Math.round(((noseOffset * lengthPercentage)) * scaleFactor));
         int frontNosey = 0;


         int frontRightx = +((int) Math.round((+(propellerOffset * lengthPercentage)) * scaleFactor));
         int frontRighty = -((int) Math.round((+(propellerOffset * widthPercentage)) * scaleFactor));

         int frontLeftx = +((int) Math.round((+(propellerOffset * lengthPercentage)) * scaleFactor));
         int frontLefty = -((int) Math.round((-(propellerOffset * widthPercentage)) * scaleFactor));

         int backRightx = +((int) Math.round((-(propellerOffset * lengthPercentage)) * scaleFactor));
         int backRighty = -((int) Math.round((+(propellerOffset * widthPercentage)) * scaleFactor));

         int backLeftx = +((int) Math.round((-(propellerOffset * lengthPercentage)) * scaleFactor));
         int backLefty = -((int) Math.round((-(propellerOffset * widthPercentage)) * scaleFactor));

         g.setColor(color);
         int w = new Double(((this.width * scaleFactor) / 2.0) * widthPercentage).intValue();
         int l = new Double(((this.length * scaleFactor) / 2.0) * lengthPercentage).intValue();

         double yaw = -_pose.getYaw();

         Graphics2D g2d = (Graphics2D) g;

         g2d.setStroke(new BasicStroke(4));

         // Point2d point = new Point2d(rxc1, ryc1);
         // Point2d point2 = new Point2d(rxc2, ryc2);
         // int radius = new Double(point.distance(point2)).intValue();
         AffineTransform af = g2d.getTransform();

         // AffineTransform af = new AffineTransform();
         af.translate(transformX, transformY);
         af.rotate(Math.toRadians(yaw));

         // af.translate(-Xcenter, -Ycenter);
         // af.translate(transformX, transformY);
         g2d.setTransform(af);

         g2d.setColor(Color.red);
         g2d.drawOval(frontRightx - l / 2, frontRighty - w / 2, l, w);
         g2d.drawOval(frontLeftx - l / 2, frontLefty - w / 2, l, w);
         g2d.drawOval(backRightx - l / 2, backRighty - w / 2, l, w);
         g2d.drawOval(backLeftx - l / 2, backLefty - w / 2, l, w);
         g2d.drawLine(0, 0, frontNosex, 0);
         af.rotate(-Math.toRadians(yaw));
         af.translate(-transformX, -transformY);
         g2d.setTransform(af);

         // g2d.drawOval(0 - radius / 2, 0 - radius / 2, width, length);
         // g2d.drawOval(0 - radius / 2, 0 - radius / 2, width, length);
         // g2d.drawOval(0 - radius / 2, 0 - radius / 2, width, length);

//       int midxFront = (rxc1 + rxc2) / 2;
//       int midyFront = (ryc1 + ryc2) / 2;
//
//       int midxright = (rxc2 + rxc3) / 2;
//       int midyright = (ryc2 + ryc3) / 2;
//       int midxleft = (rxc1 + rxc4) / 2;
//       int midyleft = (ryc1 + ryc4) / 2;

//       g2d.setColor(Color.green);
//       g2d.setStroke(new BasicStroke(3));
//

//       g2d.drawLine(midxFront, midyFront, midxleft, midyleft);
//       g2d.drawLine(midxright, midyright, midxleft, midyleft);
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
