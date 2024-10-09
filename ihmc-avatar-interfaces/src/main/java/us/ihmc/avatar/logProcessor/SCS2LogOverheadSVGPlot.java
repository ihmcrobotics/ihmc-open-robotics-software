package us.ihmc.avatar.logProcessor;

import org.jfree.svg.SVGGraphics2D;
import org.jfree.svg.SVGHints;
import org.jfree.svg.SVGUnits;
import us.ihmc.commonWalkingControlModules.desiredFootStep.FootstepListVisualizer;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.log.LogTools;

import java.awt.*;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

public class SCS2LogOverheadSVGPlot
{
   /** Square document size in meters. */
   private static final double DOCUMENT_SIZE = Double.parseDouble(System.getProperty("document.size", "15.0"));

   private final SCS2LogLocomotionData locomotionData;
   private final Path svgPath;
   private SVGGraphics2D svgGraphics2D;

   public SCS2LogOverheadSVGPlot(Path logPath, SCS2LogLocomotionData locomotionData)
   {
      this.locomotionData = locomotionData;

      svgPath = logPath.resolve(logPath.getFileName().toString() + "_OverheadPlot.svg");
   }

   public void drawSVG()
   {
      double documentSizeMillimeters = convertToMillimeters(DOCUMENT_SIZE);
      svgGraphics2D = new SVGGraphics2D(documentSizeMillimeters, documentSizeMillimeters, SVGUnits.MM);
      svgGraphics2D.setFontSizeUnits(SVGUnits.MM);

      svgGraphics2D.setRenderingHint(SVGHints.KEY_BEGIN_GROUP, "Legend");
      Point2D legendOrigin = new Point2D(locomotionData.getRobotStartLocation());
      legendOrigin.add(2.0, -2.0);
      drawText("Legend", legendOrigin.getX(), legendOrigin.getY());
      double verticalCaret = 0.1;
      double lineIndent = 0.7;
      double lineLength = 0.2;
      drawText("Center of Mass", legendOrigin.getX(), legendOrigin.getY() - verticalCaret);
      comStroke();
      drawLine(new Point2D[] {new Point2D(legendOrigin.getX() + lineIndent,
                                          legendOrigin.getY() - verticalCaret),
                              new Point2D(legendOrigin.getX() + lineIndent + lineLength,
                                          legendOrigin.getY() - verticalCaret)
      });
      verticalCaret += 0.1;
      drawText("Capture Point", legendOrigin.getX(), legendOrigin.getY() - verticalCaret);
      icpStroke();
      drawLine(new Point2D[] {new Point2D(legendOrigin.getX() + lineIndent,
                                          legendOrigin.getY() - verticalCaret),
                              new Point2D(legendOrigin.getX() + lineIndent + lineLength,
                                          legendOrigin.getY() - verticalCaret)
      });
      svgGraphics2D.setRenderingHint(SVGHints.KEY_END_GROUP, "Legend");

      int walk = 1;
      for (SCS2LogWalk logWalk : locomotionData.getLogWalks())
      {
         String walkName = "Walk %d".formatted(walk);
         svgGraphics2D.setRenderingHint(SVGHints.KEY_BEGIN_GROUP, walkName);
         drawText(walkName, logWalk.getWalkStart().getX(), logWalk.getWalkStart().getY() + 0.3);

         String groupName = "Footsteps %d".formatted(walk);
         svgGraphics2D.setRenderingHint(SVGHints.KEY_BEGIN_GROUP, groupName);

         int footstepIndex = 0;
         for (SCS2LogFootstep footstep : logWalk.getFootsteps())
         {
            Color color = FootstepListVisualizer.defaultFeetColors.get(footstep.getSide());
            svgGraphics2D.setColor(color);

            double[] polygon = footstep.getPolygon();
            LogTools.info("Drawing step at {} {}", new Point2D(polygon[0], polygon[4]), new Point2D(metersToMMX(polygon[0]), metersToMMY(polygon[4])));
            svgGraphics2D.drawPolygon(new int[] {metersToMMX(polygon[0]),
                                                 metersToMMX(polygon[1]),
                                                 metersToMMX(polygon[2]),
                                                 metersToMMX(polygon[3])},
                                      new int[] {metersToMMY(polygon[4]),
                                                 metersToMMY(polygon[5]),
                                                 metersToMMY(polygon[6]),
                                                 metersToMMY(polygon[7])},
                                      4);
            
            Point2D center = new Point2D(polygon[0], polygon[4]);
            center.add(polygon[1], polygon[5]);
            center.add(polygon[2], polygon[6]);
            center.add(polygon[3], polygon[7]);
            center.scale(0.25);
            textStroke();
            svgGraphics2D.setColor(Color.GRAY);
            svgGraphics2D.setFont(new Font("Arial", Font.PLAIN, 10));
            svgGraphics2D.drawString("%d".formatted(footstepIndex), metersToMMX(center.getX() - 0.012), metersToMMY(center.getY() - 0.012));

            ++footstepIndex;
         }

         svgGraphics2D.setRenderingHint(SVGHints.KEY_END_GROUP, groupName);

         comStroke();
         plot(logWalk.getComs(), "Coms");

         icpStroke();
         plot(logWalk.getIcps(), "ICPs");

         if (logWalk.isEndedWithFall())
         {
            Point2D fallLocation;
            if (logWalk.getFootsteps().isEmpty())
            {
               fallLocation = logWalk.getWalkStart();
            }
            else
            {
               double[] lastStepPolygon = logWalk.getFootsteps().get(logWalk.getFootsteps().size() - 1).getPolygon();
               fallLocation = new Point2D(lastStepPolygon[0], lastStepPolygon[4]);
            }

            drawText("Fall %d".formatted(walk), fallLocation.getX(), fallLocation.getY() + 0.3);
         }

         svgGraphics2D.setRenderingHint(SVGHints.KEY_END_GROUP, walkName);

         ++walk;
      }


      LogTools.info("Saving SVG to {}", svgPath);

      try (FileWriter writer = new FileWriter(svgPath.toFile()))
      {
         String svgDocument = svgGraphics2D.getSVGDocument();
         // Add viewBox attribute to the SVG element, to make it load correctly in Inkscape
         svgDocument = svgDocument.replace("<svg", "<svg viewBox=\"0 0 %f %f\"".formatted(documentSizeMillimeters, documentSizeMillimeters));
         writer.write(svgDocument);
      }
      catch (IOException e)
      {
         throw new RuntimeException(e);
      }
   }

   private void drawText(String text, double x, double y)
   {
      textStroke();
      svgGraphics2D.drawString(text, metersToMMX(x), metersToMMY(y));
   }

   private void textStroke()
   {
      svgGraphics2D.setColor(Color.BLACK);
      svgGraphics2D.setStroke(new BasicStroke(5));
      svgGraphics2D.setFont(new Font("Arial", Font.PLAIN, 20));
   }

   private void icpStroke()
   {
      svgGraphics2D.setColor(Color.BLUE);
      svgGraphics2D.setStroke(new BasicStroke(2.5f, BasicStroke.CAP_BUTT, BasicStroke.JOIN_MITER, 10.0f, new float[] {10.0f}, 0.0f));
   }

   private void comStroke()
   {
      svgGraphics2D.setColor(Color.BLACK);
      svgGraphics2D.setStroke(new BasicStroke(2.5f));
   }

   private void drawLine(Point2D[] line)
   {
      svgGraphics2D.drawLine(metersToMMX(line[0].getX()),
                             metersToMMY(line[0].getY()),
                             metersToMMX(line[1].getX()),
                             metersToMMY(line[1].getY()));
   }

   private void plot(List<Point2D> points, String name)
   {
      if (!points.isEmpty())
      {
         int[] xs = new int[points.size()];
         int[] ys = new int[points.size()];
         for (int i = 0; i < points.size(); i++)
         {
            xs[i] = metersToMMX(points.get(i).getX());
            ys[i] = metersToMMY(points.get(i).getY());
         }
         svgGraphics2D.setRenderingHint(SVGHints.KEY_BEGIN_GROUP, name);
         svgGraphics2D.drawPolyline(xs, ys, xs.length);
         svgGraphics2D.setRenderingHint(SVGHints.KEY_END_GROUP, name);
      }
   }

   private int metersToMMX(double x)
   {
      double fromStart = x - locomotionData.getRobotStartLocation().getX();
      int halfDocumentMM = (int) convertToMillimeters(DOCUMENT_SIZE / 2.0);
      int mmFromStart = (int) convertToMillimeters(fromStart);
      return mmFromStart + halfDocumentMM;
   }

   private int metersToMMY(double y)
   {
      double fromStart = y - locomotionData.getRobotStartLocation().getY();
      int halfDocumentMM = (int) convertToMillimeters(DOCUMENT_SIZE / 2.0);
      int mmFromStart = (int) -convertToMillimeters(fromStart);
      return mmFromStart + halfDocumentMM;
   }

   private long convertToMillimeters(double meters)
   {
      return (long) (meters * 1000.0);
   }
}
