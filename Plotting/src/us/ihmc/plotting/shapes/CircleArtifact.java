package us.ihmc.plotting.shapes;

import java.awt.Color;
import java.awt.Graphics;
import java.io.BufferedReader;
import java.io.PrintWriter;
import java.util.StringTokenizer;

import us.ihmc.plotting.Artifact;

public class CircleArtifact extends Artifact
{
   private static final long serialVersionUID = -8129731194060916363L;
   private double x;
   private double y;
   private double diameter;
   private boolean fill = true;

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
   public void draw(Graphics g, int Xcenter, int Ycenter, double headingOffset, double scaleFactor)
   {
      int x = Xcenter + ((int)Math.round(this.x * scaleFactor));
      int y = Ycenter - ((int)Math.round(this.y * scaleFactor));

      g.setColor(color);
      int d = (int) ((this.diameter * scaleFactor));
      if (fill)
      {
         g.fillOval((x - (d / 2)), (y - (d / 2)), d, d);
      }
      else
      {
         g.drawOval((x - (d / 2)), (y - (d / 2)), d, d);
      }
   }

   public void drawLegend(Graphics g, int Xcenter, int Ycenter, double scaleFactor)
   {
      int x = Xcenter;
      int y = Ycenter;

      g.setColor(color);
      int d = (int) ((this.diameter * scaleFactor));
      if (fill)
      {
         g.fillOval((x - (d / 2)), (y - (d / 2)), d, d);
      }
      else
      {
         g.drawOval((x - (d / 2)), (y - (d / 2)), d, d);
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
   
   public void drawHistory(Graphics g, int Xcenter, int Ycenter, double scaleFactor)
   {
      throw new RuntimeException("Not implemented!");
   }

   public void takeHistorySnapshot()
   {
      throw new RuntimeException("Not implemented!");
   }
}
