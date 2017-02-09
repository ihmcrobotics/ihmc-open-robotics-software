package us.ihmc.graphicsDescription.plotting.artifact;

import java.awt.BasicStroke;
import java.awt.Color;
import java.io.BufferedReader;
import java.io.PrintWriter;
import java.util.StringTokenizer;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

import us.ihmc.graphicsDescription.plotting.Graphics2DAdapter;
import us.ihmc.graphicsDescription.plotting.Plotter2DAdapter;

public class CircleArtifact extends Artifact
{
   private static final BasicStroke STROKE = new BasicStroke(1.0f);
   private static final double LEGEND_RADIUS = 20.0;
   
   private double x;
   private double y;
   private double diameter;
   private boolean fill = true;
   
   private final Point2d tempPoint = new Point2d();
   private final Vector2d tempRadii = new Vector2d();

   public CircleArtifact(String id, double x, double y, double diameter, boolean fill)
   {
      super(id);
      setLevel(1);
      this.x = x;
      this.y = y;
      this.diameter = diameter;
      this.fill = fill;
   }

   public CircleArtifact(String string, double x2, double y2, double d, boolean b, Color color)
   {
      this(string, x2, y2, d, b);
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

   /**
    * Must provide a draw method for plotter to render artifact
    */
   @Override
   public void draw(Graphics2DAdapter graphics)
   {
      graphics.setColor(color);
      graphics.setStroke(STROKE);
      tempPoint.set(x, y);
      tempRadii.set(diameter / 2.0, diameter / 2.0);

      if (fill)
      {
         graphics.drawOvalFilled(tempPoint, tempRadii);
      }
      else
      {
         graphics.drawOval(tempPoint, tempRadii);
      }
   }

   @Override
   public void drawLegend(Plotter2DAdapter graphics, Point2d origin)
   {
      graphics.setColor(color);
      graphics.setStroke(STROKE);
      tempPoint.set(origin);
      tempRadii.set(LEGEND_RADIUS, LEGEND_RADIUS);
      if (fill)
      {
         graphics.drawOvalFilled(graphics.getScreenFrame(), tempPoint, tempRadii);
      }
      else
      {
         graphics.drawOval(graphics.getScreenFrame(), tempPoint, tempRadii);
      }
   }


   public void save(PrintWriter printWriter)
   {
      printWriter.println(x + " " + y + " " + diameter + " " + fill + " " + id);
   }

   public static CircleArtifact load(BufferedReader bufferedReader)
   {
      CircleArtifact circleArtifact = null;
      try
      {
         String line = bufferedReader.readLine();
         if (line == null)
            return null;
         StringTokenizer s = new StringTokenizer(line, " ");
         double x = Double.parseDouble(s.nextToken());
         double y = Double.parseDouble(s.nextToken());
         double diameter = Double.parseDouble(s.nextToken());
         boolean fill = Boolean.parseBoolean(s.nextToken());
         String id = s.nextToken();
         circleArtifact = new CircleArtifact(id, x, y, diameter, fill);
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }

      return circleArtifact;
   }

   public CircleArtifact getCopy()
   {
      CircleArtifact cirlceCopy = new CircleArtifact(this.getID(), x, y, diameter, fill);
      cirlceCopy.setColor(this.getColor());

      return cirlceCopy;
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
