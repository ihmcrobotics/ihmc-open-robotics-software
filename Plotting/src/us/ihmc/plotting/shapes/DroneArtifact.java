package us.ihmc.plotting.shapes;

import java.awt.BasicStroke;
import java.awt.Graphics;
import java.awt.Graphics2D;

import javax.swing.JFrame;

import us.ihmc.plotting.Artifact;
import us.ihmc.plotting.PlotterPanel;
import us.ihmc.plotting.Pose;

public class DroneArtifact extends Artifact
{
   private static final long serialVersionUID = 874940514060462114L;
   protected Pose _pose;

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
      return this.getID() + " : " + this.getType() + " " + this._pose.getX() + "," + this._pose.getY() + "," + this._pose.getZ() + "," + this._pose.getYaw()
             + "," + this._pose.getPitch() + "," + this._pose.getRoll();
   }

   /**
    * Must provide a draw method for plotter to render artifact
    */
   public void draw(Graphics g, int Xcenter, int Ycenter, double scaleFactor)
   {
      int x = Xcenter + ((int) Math.round(_pose.getX() * scaleFactor));
      int y = Ycenter - ((int) Math.round(_pose.getY() * scaleFactor));

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
      int xHDG = x + (int) Math.round(Math.sin(hdg) * w2);
      int yHDG = y - (int) Math.round(Math.cos(hdg) * w2);

      
      Graphics2D g2d = (Graphics2D)g;
      int width = 3;
      g2d.setStroke(new BasicStroke(width));
      
      g2d.setColor(color);
      
      g2d.drawOval((x - w12 - w12), (y + w12 - w12), w1, w1);
      g2d.drawOval((x + w12 - w12), (y + w12 - w12), w1, w1);
      g2d.drawOval((x - w12 - w12), (y - w12 - w12), w1, w1);
      g2d.drawOval((x + w12 - w12), (y - w12 - w12), w1, w1);

      g2d.drawOval((x - w12 - w12), (y + w12 - w12), w1, w1);
      g2d.drawOval((x + w12 - w12), (y + w12 - w12), w1, w1);
      g2d.drawOval((x - w12 - w12), (y - w12 - w12), w1, w1);
      g2d.drawOval((x + w12 - w12), (y - w12 - w12), w1, w1);
       width = 4;
      g2d.setStroke(new BasicStroke(width));
      //g.drawOval((x - w12 / 2), (y - w12 / 2), w12, w1 * 2);
      g2d.drawLine(x, y, xHDG, yHDG);
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

   public static void main(String[] args)
   {
      JFrame frame = new JFrame();
      PlotterPanel panel = new PlotterPanel();
      frame.getContentPane().add(panel);
      frame.setVisible(true);
      DroneArtifact d = new DroneArtifact("asdf");
      d.setPose(new Pose(0, 0, 0, Pose.METER));
      panel.getPlotter().addArtifact(d);
   }
}
