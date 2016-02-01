package us.ihmc.plotting.shapes;

import java.awt.Graphics;

import javax.vecmath.Point2d;

import us.ihmc.plotting.Artifact;

/**
 * Last updated by: mjohnson
 * On: 6/10/11 9:46 AM
 */
public class MarkerArtifact extends Artifact
{
   protected Point2d location = new Point2d();
   protected Point2d offset = new Point2d();
   protected double range = 0.0;
   protected double bearing = 0.0;
   protected double edgeLength = 0.2;

   public MarkerArtifact(String id)
   {
      super(id);
      setType("marker");
   }

   public void setLocation(Point2d location)
   {
      this.location = location;
   }

   public void setOffset(Point2d offset)
   {
      this.offset = offset;
   }

   public void setRange(double range)
   {
      this.range = range;
   }

   public void setBearing(double bearing)
   {
      this.bearing = bearing;
   }

   public String describe()
   {
      return this.getID() + " : " + this.getType() + " " + location + "; " + range + "; " + bearing;
   }

   /**
    * Must provide a draw method for plotter to render artifact
    */
   public void draw(Graphics g, int Xcenter, int Ycenter, double headingOffset, double scaleFactor)
   {
      // find scaled location
      int x = Xcenter + ((int)Math.round(location.getX() * scaleFactor));
      int y = Ycenter - ((int)Math.round(location.getY() * scaleFactor));

      // scale size
      int scaledEdge = ((int)Math.round(edgeLength * scaleFactor));

      // draw marker box
      g.setColor(color);
      g.drawRect((x - (scaledEdge/2)), (y - (scaledEdge/2)), scaledEdge, scaledEdge);

      // draw line to location of camera
      double hdg = Math.toRadians(bearing);
      int scaledRange = (int)(range * scaleFactor);
      int xHDG = x + (int)Math.round(Math.sin(hdg) * scaledRange);
      int yHDG = y - (int)Math.round(Math.cos(hdg) * scaledRange);
      g.drawLine(x, y, xHDG, yHDG);

      // draw point for estimated position
      int xOffset = xHDG + ((int)Math.round(-offset.getX() * scaleFactor));
      int yOffset = yHDG - ((int)Math.round(-offset.getY() * scaleFactor));
      g.fillOval(xOffset, yOffset, scaledEdge / 2, scaledEdge / 2);


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
