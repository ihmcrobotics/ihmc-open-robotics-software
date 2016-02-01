package us.ihmc.plotting.shapes;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Stroke;

import us.ihmc.plotting.Artifact;

public class ArcArtifact extends Artifact
{
   private static final long serialVersionUID = -8129731194060916363L;
   private double x;
   private double y;
   private double diameter;
   private double startAngle;
   private double arcAngle;
   private int lineThickness = 1;

   public ArcArtifact(String id, double x, double y, double diameter, double startAngle, double arcAngle)
   {
      super(id);
      setLevel(1);
      this.x = x;
      this.y = y;
      this.diameter = diameter;
      this.startAngle = startAngle;
      this.arcAngle = arcAngle;
   }

   public ArcArtifact(String string, double x2, double y2, double diameter, double startAngle, double arcAngle, Color color)
   {
      this(string, x2, y2, diameter, startAngle, arcAngle);
      this.setColor(color);
   }

   public void setPosition(double x, double y)
   {
      this.x = x;
      this.y = y;
   }

   public void setDiameter(double diameter)
   {
      this.diameter = diameter;
   }

   public void setLineThickness(int lineThickness)
   {
      this.lineThickness = lineThickness;
   }

   public double getX()
   {
      return x;
   }


   public double getY()
   {
      return y;
   }

   public void draw(Graphics g, int Xcenter, int Ycenter, double headingOffset, double scaleFactor)
   {
      int x = Xcenter + ((int) Math.round(this.x * scaleFactor));
      int y = Ycenter - ((int) Math.round(this.y * scaleFactor));

      g.setColor(color);
      int d = (int) ((this.diameter * scaleFactor));

      Graphics2D g2d = (Graphics2D) g;
      Stroke currentStroke = g2d.getStroke();
      g2d.setStroke(new BasicStroke(lineThickness));
      g.drawArc((x - (d / 2)), (y - (d / 2)), d, d, (int) startAngle, (int) arcAngle);
      g2d.setStroke(currentStroke);

   }

   public void drawLegend(Graphics g, int Xcenter, int Ycenter, double scaleFactor)
   {
      throw new RuntimeException("Not implemented!");
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
