package us.ihmc.plotting.artifact;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Stroke;

import us.ihmc.plotting.Graphics2DAdapter;

public class ArcArtifact extends Artifact
{
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

   @Override
   public void draw(Graphics2DAdapter graphics2d, int centerX, int centerY, double headingOffset, double scaleFactor)
   {
      int x = centerX + ((int) Math.round(this.x * scaleFactor));
      int y = centerY - ((int) Math.round(this.y * scaleFactor));

      graphics2d.setColor(color);
      int d = (int) ((this.diameter * scaleFactor));

      Stroke currentStroke = graphics2d.getStroke();
      graphics2d.setStroke(new BasicStroke(lineThickness));
      graphics2d.drawArc((x - (d / 2)), (y - (d / 2)), d, d, (int) startAngle, (int) arcAngle);
      graphics2d.setStroke(currentStroke);
   }

   @Override
   public void drawLegend(Graphics2DAdapter graphics2d, int centerX, int centerY, double scaleFactor)
   {
      throw new RuntimeException("Not implemented!");
   }

   @Override
   public void drawHistory(Graphics2DAdapter graphics2d, int centerX, int centerY, double scaleFactor)
   {
      throw new RuntimeException("Not implemented!");
   }

   @Override
   public void takeHistorySnapshot()
   {
      throw new RuntimeException("Not implemented!");
   }
}
