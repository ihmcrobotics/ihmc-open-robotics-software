package us.ihmc.graphicsDescription.plotting.artifact;

import java.awt.BasicStroke;
import java.awt.Color;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

import us.ihmc.graphicsDescription.plotting.Graphics2DAdapter;
import us.ihmc.graphicsDescription.plotting.Plotter2DAdapter;

public class ArcArtifact extends Artifact
{
   private static final BasicStroke STROKE = new BasicStroke(1.0f);
   
   private double x;
   private double y;
   private double diameter;
   private double startAngle;
   private double arcAngle;
   
   private final Point2d tempPoint = new Point2d();
   private final Vector2d tempRadii = new Vector2d();

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

   public double getX()
   {
      return x;
   }

   public double getY()
   {
      return y;
   }

   @Override
   public void draw(Graphics2DAdapter graphics)
   {
      graphics.setColor(color);
      graphics.setStroke(STROKE);
      
      tempPoint.set(x, y);
      tempRadii.set(diameter / 2.0, diameter / 2.0);
      graphics.drawArc(tempPoint, tempRadii, startAngle, arcAngle);
   }

   @Override
   public void drawLegend(Plotter2DAdapter graphics, Point2d origin)
   {
      throw new RuntimeException("Not implemented!");
   }

   @Override
   public void drawHistory(Graphics2DAdapter graphics)
   {
      throw new RuntimeException("Not implemented!");
   }

   @Override
   public void takeHistorySnapshot()
   {
      throw new RuntimeException("Not implemented!");
   }
}
