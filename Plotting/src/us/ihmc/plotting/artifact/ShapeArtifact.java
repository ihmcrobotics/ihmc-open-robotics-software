package us.ihmc.plotting.artifact;

import java.io.BufferedReader;
import java.io.PrintWriter;
import java.util.StringTokenizer;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

import us.ihmc.plotting.Coordinate;
import us.ihmc.plotting.Graphics2DAdapter;
import us.ihmc.plotting.Pose;

public class ShapeArtifact extends Artifact
{
   private Pose pose;
   private double height;
   private double width;
   
   private final Point2d tempCenter = new Point2d();
   private final Vector2d tempRadii = new Vector2d();

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
   @Override
   public void draw(Graphics2DAdapter graphics)
   {
      if (pose == null)
      {
         System.out.println("problem...shape with null pose:" + this.getID());

         return;
      }

      graphics.setColor(color);
      tempCenter.set(pose.getX(), pose.getY());
      tempRadii.set(width / 2.0, height / 2.0);
      
      if (getType().equals("fillcircle"))
      {
         graphics.drawOvalFilled(tempCenter, tempRadii);
      }
      else if (getType().equals("circle"))
      {
         graphics.drawOval(tempCenter, tempRadii);
      }
      else if (getType().equals("fillrectangle"))
      {
         graphics.drawSquareFilled(tempCenter, tempRadii);
      }
      else if (getType().equals("rectangle"))
      {
         graphics.drawSquare(tempCenter, tempRadii);
      }
   }

   @Override
   public void drawLegend(Graphics2DAdapter graphics, int centerX, int centerY)
   {
      graphics.setColor(color);
      int w = 40;
      int h = 40;
      if (getType().equals("fillcircle"))
      {
         graphics.drawOvalFilled((centerX - (w / 2)), (centerY - (h / 2)), w, h);
      }
      else if (getType().equals("circle"))
      {
         graphics.drawOval((centerX - (w / 2)), (centerY - (h / 2)), w, h);
      }
      else if (getType().equals("fillrectangle"))
      {
         graphics.drawRectangleFilled((centerX - (w / 2)), (centerY - (h / 2)), w, h);
      }
      else if (getType().equals("rectangle"))
      {
         graphics.drawRectangle((centerX - (w / 2)), (centerY - (h / 2)), w, h);
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
