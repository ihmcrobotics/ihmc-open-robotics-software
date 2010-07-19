package us.ihmc.plotting.shapes;

import java.awt.*;
import java.io.*;

import java.util.StringTokenizer;
import us.ihmc.plotting.Artifact;
import us.ihmc.plotting.Pose;
import us.ihmc.plotting.Coordinate;

public class ShapeArtifact extends Artifact
{
   /**
    * 
    */
   private static final long serialVersionUID = 2446850336045691305L;
   private Pose _pose;
   private double _height;
   private double _width;

   public ShapeArtifact(String id, String type, double height, double width, Pose pose)
   {
      super(id);
      setType(type);
      setLevel(1);
      _pose = pose;
      _height = height;
      _width = width;
   }

   public void setPose(Pose pose)
   {
      _pose = pose;
   }

   public Coordinate getCoordinate()
   {
      return _pose;
   }


   public Pose getPose()
   {
      return _pose;
   }

   /**
    * Must provide a draw method for plotter to render artifact
    */
   public void draw(Graphics g, int Xcenter, int Ycenter, double scaleFactor)
   {
      if (_pose == null)
      {
         System.out.println("problem...shape with null pose:" + this.getID());

         return;
      }

      int x = Xcenter + ((int)Math.round(_pose.getX() * scaleFactor));
      int y = Ycenter - ((int)Math.round(_pose.getY() * scaleFactor));

      g.setColor(color);
      int w = (int) (_width * scaleFactor);
      int h = (int) (_height * scaleFactor);
      if (super.getType().equals("fillcircle"))
      {
         g.fillOval((x - (w / 2)), (y - (h / 2)), w, h);
      }
      else if (super.getType().equals("circle"))
      {
         g.drawOval((x - (w / 2)), (y - (h / 2)), w, h);
      }
      else if (super.getType().equals("fillrectangle"))
      {
         g.fillRect((x - (w / 2)), (y - (h / 2)), w, h);
      }
      else if (super.getType().equals("rectangle"))
      {
         g.drawRect((x - (w / 2)), (y - (h / 2)), w, h);
      }

   }

   public void drawLegend(Graphics g, int Xcenter, int Ycenter, double scaleFactor)
   {
      int x = Xcenter;
      int y = Ycenter;

      g.setColor(color);
      int w = (int) (_width * scaleFactor);
      int h = (int) (_height * scaleFactor);
      if (super.getType().equals("fillcircle"))
      {
         g.fillOval((x - (w / 2)), (y - (h / 2)), w, h);
      }
      else if (super.getType().equals("circle"))
      {
         g.drawOval((x - (w / 2)), (y - (h / 2)), w, h);
      }
      else if (super.getType().equals("fillrectangle"))
      {
         g.fillRect((x - (w / 2)), (y - (h / 2)), w, h);
      }
      else if (super.getType().equals("rectangle"))
      {
         g.drawRect((x - (w / 2)), (y - (h / 2)), w, h);
      }
   }



   public void save(PrintWriter printWriter)
   {
      printWriter.println(_pose.getX() + " " + _pose.getY() + " " + _width + " " + _height + " " + getType() + " " + id);
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
      ShapeArtifact shapeCopy = new ShapeArtifact(this.getID(), this.getType(), _height, _width, this.getPose());
      shapeCopy.setColor(this.getColor());

      return shapeCopy;
   }

}
