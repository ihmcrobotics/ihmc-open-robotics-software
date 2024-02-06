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
            M 75.186018,102.25503 73.21275,110.75519 80.650436,116.37138 90.820271,114.09455 95.070368,118.79999 101.29367,113.18382 96.436443,109.3891 100.99011,100.88892 94.463176,92.540541 82.471889,93.147694 83.5344,96.942414 93.248855,98.612091 92.338159,107.11227 80.195059,110.29983 79.132551,101.79966 Z        
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
            ReferenceFrame vertexFrame = ReferenceFrame.getWorldFrame(); // When using absolute mode this is world apparently
            vertices.add(new FramePoint2D(vertexFrame, Double.parseDouble(coordinates[0]), Double.parseDouble(coordinates[1])));
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

      for (FramePoint2D vertex : vertices)
      {
         vertex.changeFrame(ReferenceFrame.getWorldFrame());
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
