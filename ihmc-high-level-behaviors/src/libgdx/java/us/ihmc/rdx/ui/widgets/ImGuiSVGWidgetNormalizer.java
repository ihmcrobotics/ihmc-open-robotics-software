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
 *
 * In Inksscape, set SVG Ouput in preferences to Path string format: "Absolute"
 *
 * FIXME: This class is still not feature complete.
 */
public class ImGuiSVGWidgetNormalizer
{
   public ImGuiSVGWidgetNormalizer()
   {
      // Paste SVG path n here
      String pathDString = """
                  M 99.303402,65.896674
                  V 63.783598
                  L 95.971674,63.727697 94.741843,62.911534 94.663579,61.122686
                  H 96.262363
                  L 96.273542,60.284163 93.444927,56.494044 90.593951,60.317705 90.616311,61.077966 92.192733,61.122686 92.103291,62.531405 92.46106,64.532679 94.071023,65.773691 96.620133,65.941394 Z
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

            if (command.equals("m")) // Move to
            {
               String[] coordinates = commands[++i].split(",");
               drawFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("drawFrame",
                                                                                             ReferenceFrame.getWorldFrame(),
                                                                                             new RigidBodyTransform(new RotationMatrix(),
                                                                                                                    new Point3D(Double.parseDouble(coordinates[0]),
                                                                                                                                Double.parseDouble(coordinates[1]),
                                                                                                                                0.0)));
            }
            else if (command.equalsIgnoreCase("z")) // Close path
            {
               vertices.add(new FramePoint2D(vertices.get(0)));
            }

            continue;
         }

         if (command.equals("M")) // A vertex
         {
            String[] coordinates = commands[i].split(",");
            ReferenceFrame vertexFrame = ReferenceFrame.getWorldFrame(); // When using absolute mode this is world apparently
            vertices.add(new FramePoint2D(vertexFrame, Double.parseDouble(coordinates[0]), Double.parseDouble(coordinates[1])));
         }
         else if (command.equals("m")) // relative mode TODO fix
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

      ReferenceFrame centerFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("centerFrame",
                                                                                                     ReferenceFrame.getWorldFrame(),
                                                                                                     new RigidBodyTransform(new RotationMatrix(),
                                                                                                                            new Point3D(xMin + width / 2.0,
                                                                                                                                        yMin + height / 2.0,
                                                                                                                                        0.0)));

      for (FramePoint2D vertex : vertices)
      {
         vertex.changeFrame(centerFrame);
         vertex.scale(1.0 / maxDimension);
      }

      System.out.println("private final ArrayList<Point2D32> vertices = new ArrayList<>();\n{");
      for (FramePoint2D vertex : vertices)
      {
         System.out.printf("   new ImVec2(%.3ff, %.3ff),%n", vertex.getX(), vertex.getY());
      }
      System.out.println("}");
   }

   public static void main(String[] args)
   {
      new ImGuiSVGWidgetNormalizer();
   }
}
