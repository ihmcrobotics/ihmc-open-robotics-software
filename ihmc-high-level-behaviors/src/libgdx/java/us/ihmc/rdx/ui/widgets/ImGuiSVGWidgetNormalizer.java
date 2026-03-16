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
                  M 51.144162,108.02267 20.922612,108.02267 30.841479,108.02267 30.22155,19.527771 51.764091,19.372789 51.764091,26.346992 37.970666,26.19201 38.435612,107.86769 Z
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

      System.out.println("private final ImVec2[] polygon = new ImVec2[]\n{");
      for (FramePoint2D vertex : vertices)
      {
         System.out.printf("   new ImVec2(%.3ff, %.3ff),%n", vertex.getX(), vertex.getY());
      }
      System.out.println("};");
   }

   public static void main(String[] args)
   {
      new ImGuiSVGWidgetNormalizer();
   }
}
