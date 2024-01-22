package us.ihmc.rdx.ui.widgets;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;

import java.util.ArrayList;

/**
 * Utility to convert SVG vertices to a format pastable into
 * custom ImGui widget classes.
 */
public class ImGuiSVGWidgetNormalizer
{
   public ImGuiSVGWidgetNormalizer()
   {
      // Paste SVG path n here
      String pathDString =
            """
            M 54.998117,120.67206 V 149.66372 L 58.944634,157.70852 65.775127,158.77105 74.27529,159.07461 82.016528,157.25316 85.659448,152.6995 89.302385,144.5029 92.793522,133.87768 93.552467,128.4133 87.632698,126.28825 85.052301,128.71686 84.596924,134.78841 84.293357,138.58313 V 115.96661 111.41295 L 82.927252,107.77002 77.462863,109.4397 77.614647,112.77904 77.311066,131.29727 77.007486,109.4397 V 106.85929 L 75.489598,104.43066 72.150257,105.3414 70.328789,109.4397 70.784153,130.53833 V 109.74327 L 67.90016,103.67172 63.498295,104.58246 63.194715,107.77002 63.498295,129.47581 62.891135,109.89506 61.525046,106.70749 57.882109,107.01107 55.757061,111.86832 Z        
            """;

      String[] commands = pathDString.split("\\s+");

      ArrayList<FramePoint2D> vertices = new ArrayList<>();

      ReferenceFrame drawFrame = null;
      String command = null;
      for (int i = 0; i < commands.length; i++)
      {
         if (!commands[i].contains(",")) // Command stays the same until changed
         {
            command = commands[i];

            if (command.equalsIgnoreCase("m")) // Move to
            {
               String[] coordinates = commands[++i].split(",");
               drawFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("drawFrame", ReferenceFrame.getWorldFrame(),
                                new RigidBodyTransform(new RotationMatrix(), new Point3D(Double.parseDouble(coordinates[0]),
                                                                                         Double.parseDouble(coordinates[1]),
                                                                                         0.0)));
            }
            else if (command.equals("Z")) // Close path
            {
               vertices.add(new FramePoint2D(vertices.get(0)));
            }

            continue;
         }

         if (command.equalsIgnoreCase("m")) // A vertex
         {
            String[] coordinates = commands[i].split(",");
            vertices.add(new FramePoint2D(drawFrame, Double.parseDouble(coordinates[0]), Double.parseDouble(coordinates[1])));
         }
         else if (command.equals("L")) // Line to
         {
            String[] coordinates = commands[i].split(",");
            FramePoint2D newPoint = new FramePoint2D(ReferenceFrame.getWorldFrame(), Double.parseDouble(coordinates[0]), Double.parseDouble(coordinates[1]));
            vertices.add(newPoint);
         }
         else if (command.equals("H")) // Horizontal line to
         {
            FramePoint2D lastPoint = vertices.get(vertices.size() - 1);
//            lastPoint.changeFrame(ReferenceFrame.getWorldFrame());
            FramePoint2D newPoint = new FramePoint2D(drawFrame, lastPoint.getX32() + Double.parseDouble(commands[i]), lastPoint.getY32());
            vertices.add(newPoint);
         }
         else if (command.equals("V")) // Vertical line to
         {
            FramePoint2D lastPoint = vertices.get(vertices.size() - 1);
//            lastPoint.changeFrame(ReferenceFrame.getWorldFrame());
            FramePoint2D newPoint = new FramePoint2D(drawFrame, lastPoint.getX32(), lastPoint.getY32() + Double.parseDouble(commands[i]));
            vertices.add(newPoint);
         }
      }

      double xMin = Double.MAX_VALUE;
      double xMax = Double.MIN_VALUE;
      double yMin = Double.MAX_VALUE;
      double yMax = Double.MIN_VALUE;
      for (FramePoint2D vertex : vertices)
      {
         xMin = Math.min(xMin, vertex.getX());
         xMax = Math.max(xMax, vertex.getX());
         yMin = Math.min(yMin, vertex.getY());
         yMax = Math.max(yMax, vertex.getY());
      }

      double width = xMax - xMin;
      double height = yMax - yMin;

      double maxDimension = Math.max(width, height);


      ReferenceFrame centerFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("centerFrame", ReferenceFrame.getWorldFrame(),
                              new RigidBodyTransform(new RotationMatrix(),
                                                     new Point3D(xMin + width / 2.0, yMin + height / 2.0, 0.0)));

      for (FramePoint2D vertex : vertices)
      {
         vertex.changeFrame(centerFrame);
         vertex.scale(1.0 / maxDimension);
      }

      System.out.println("private final ArrayList<Point2D32> vertices = new ArrayList<>();\n{");
      for (FramePoint2D vertex : vertices)
      {
         System.out.printf("   vertices.add(new Point2D32(%.3ff, %.3ff));%n", vertex.getX(), vertex.getY());
      }
      System.out.println("}");
   }

   public static void main(String[] args)
   {
      new ImGuiSVGWidgetNormalizer();
   }
}
