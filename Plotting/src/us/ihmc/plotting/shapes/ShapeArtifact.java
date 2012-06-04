package us.ihmc.plotting.shapes;

import java.awt.Graphics;
import java.io.BufferedReader;
import java.io.PrintWriter;
import java.util.StringTokenizer;

import us.ihmc.plotting.Artifact;
import us.ihmc.plotting.Coordinate;
import us.ihmc.plotting.Pose;

public class ShapeArtifact extends Artifact
{
   private static final long serialVersionUID = 2446850336045691305L;
   private Pose pose;
   private double height;
   private double width;

   public ShapeArtifact(String id, String type, double height, double width, Pose pose)
   {
      super(id);
      setType(type);
      setLevel(1);
      this.pose = pose;
      this.height = height;
      this.width = width;
   }
   
   public void setPose(Pose pose)
   {
      this.pose = pose;
   }

   public Coordinate getCoordinate()
   {
      return pose;
   }


   public Pose getPose()
   {
      return pose;
   }

   /**
    * Must provide a draw method for plotter to render artifact
    */
   public void draw(Graphics g, int Xcenter, int Ycenter, double headingOffset, double scaleFactor)
   {
      if (pose == null)
      {
         System.out.println("problem...shape with null pose:" + this.getID());

         return;
      }

      int x = Xcenter + ((int)Math.round(pose.getX() * scaleFactor));
      int y = Ycenter - ((int)Math.round(pose.getY() * scaleFactor));

      g.setColor(color);
      int w = (int) (width * scaleFactor);
      int h = (int) (height * scaleFactor);
      if (getType().equals("fillcircle"))
      {
         g.fillOval((x - (w / 2)), (y - (h / 2)), w, h);
      }
      else if (getType().equals("circle"))
      {
         g.drawOval((x - (w / 2)), (y - (h / 2)), w, h);
      }
      else if (getType().equals("fillrectangle"))
      {
         g.fillRect((x - (w / 2)), (y - (h / 2)), w, h);
      }
      else if (getType().equals("rectangle"))
      {
         g.drawRect((x - (w / 2)), (y - (h / 2)), w, h);
      }

   }

   public void drawLegend(Graphics g, int Xcenter, int Ycenter, double scaleFactor)
   {
      int x = Xcenter;
      int y = Ycenter;

      g.setColor(color);
      int w = (int) (width * scaleFactor);
      int h = (int) (height * scaleFactor);
      if (getType().equals("fillcircle"))
      {
         g.fillOval((x - (w / 2)), (y - (h / 2)), w, h);
      }
      else if (getType().equals("circle"))
      {
         g.drawOval((x - (w / 2)), (y - (h / 2)), w, h);
      }
      else if (getType().equals("fillrectangle"))
      {
         g.fillRect((x - (w / 2)), (y - (h / 2)), w, h);
      }
      else if (getType().equals("rectangle"))
      {
         g.drawRect((x - (w / 2)), (y - (h / 2)), w, h);
      }
   }



   public void save(PrintWriter printWriter)
   {
      printWriter.println(pose.getX() + " " + pose.getY() + " " + width + " " + height + " " + getType() + " " + id);
   }

   public static ShapeArtifact load(BufferedReader bufferedReader)
   {
      ShapeArtifact shapeArtifact = null;
      try
      {
         String line = bufferedReader.readLine();
         if (line == null)
            return null;
         StringTokenizer s = new StringTokenizer(line, " ");
         double x = Double.parseDouble(s.nextToken());
         double y = Double.parseDouble(s.nextToken());
         Pose pose = new Pose(x, y, 0.0, Coordinate.METER);
         double width = Double.parseDouble(s.nextToken());
         double height = Double.parseDouble(s.nextToken());
         String type = s.nextToken();
         String id = s.nextToken();
         shapeArtifact = new ShapeArtifact(id, type, height, width, pose);
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }

      return shapeArtifact;
   }

   public ShapeArtifact getCopy()
   {
      ShapeArtifact shapeCopy = new ShapeArtifact(this.getID(), this.getType(), height, width, this.getPose());
      shapeCopy.setColor(this.getColor());

      return shapeCopy;
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
