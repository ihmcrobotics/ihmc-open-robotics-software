package us.ihmc.graphicsDescription.yoGraphics.plotting;

import java.awt.BasicStroke;
import java.awt.Color;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.graphicsDescription.plotting.Graphics2DAdapter;
import us.ihmc.graphicsDescription.plotting.Plotter2DAdapter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameLineSegment2D;
import us.ihmc.yoVariables.variable.YoFramePoint2D;

public class YoArtifactLineSegment2d extends YoArtifact
{
   private static final BasicStroke STROKE = new BasicStroke(2);

   private final YoFrameLineSegment2D lineSegment;
   
   private final Point2D tempFirstEndpoint = new Point2D();
   private final LineSegment2D tempLineSegment = new LineSegment2D();
   private final ConvexPolygon2D tempArrowPolygon = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(new double[][] {{0.0, 0.1}, {0.1, 0.0}, {0.1, 0.1}}));
   
   private final boolean drawArrow;

   // Arrow only object
   private double arrowHeadWidth;
   private double arrowHeadHeight;
   private Vector2D arrowHeadVector;
   private Vector2D arrowHeadLateralVector;
   
   public YoArtifactLineSegment2d(String name, YoFramePoint2D startPoint, YoFramePoint2D endPoint, Color color, double arrowHeadWidth, double arrowHeadHeight)
   {
      this(name, startPoint.getYoX(), startPoint.getYoY(), endPoint.getYoX(), endPoint.getYoY(), color, arrowHeadWidth, arrowHeadHeight);
   }

   private YoArtifactLineSegment2d(String name, YoDouble startX, YoDouble startY, YoDouble endX, YoDouble endY, Color color, double arrowHeadWidth, double arrowHeadHeight)
   {
      this(name, new YoFrameLineSegment2D(startX, startY, endX, endY, ReferenceFrame.getWorldFrame()), color, arrowHeadWidth, arrowHeadHeight);
   }
   
   public YoArtifactLineSegment2d(String name, YoFramePoint2D start, YoFramePoint2D end, Color color)
   {
      this(name, new YoFrameLineSegment2D(start.getYoX(), start.getYoY(), end.getYoX(), end.getYoY(), ReferenceFrame.getWorldFrame()), color);
   }
   
   public YoArtifactLineSegment2d(String name, YoFrameLineSegment2D lineSegment, Color color, double arrowHeadWidth, double arrowHeadHeight)
   {
      super(name, new double[0], color,
            lineSegment.getYoFirstEndpointX(), lineSegment.getYoFirstEndpointY(), lineSegment.getYoSecondEndpointX(), lineSegment.getYoSecondEndpointY());
      this.lineSegment = lineSegment;
      this.drawArrow = true;
      instatiateArrowObjects(arrowHeadWidth, arrowHeadHeight);
   }

   public YoArtifactLineSegment2d(String name, YoFrameLineSegment2D lineSegment, Color color)
   {
      super(name, new double[0], color,
            lineSegment.getYoFirstEndpointX(), lineSegment.getYoFirstEndpointY(), lineSegment.getYoSecondEndpointX(), lineSegment.getYoSecondEndpointY());
      this.lineSegment = lineSegment;
      this.drawArrow = false;
   }

   public void instatiateArrowObjects(double arrowHeadWidth, double arrowHeadHeight)
   {
      this.arrowHeadWidth = arrowHeadWidth;
      this.arrowHeadHeight = arrowHeadHeight;
      arrowHeadVector = new Vector2D();
      arrowHeadLateralVector = new Vector2D();
   }
   
   @Override
   public void draw(Graphics2DAdapter graphics)
   {
      graphics.setColor(color);
      graphics.setStroke(STROKE);

      if (lineSegment.getFirstEndpoint().equals(lineSegment.getSecondEndpoint()))
      {
         tempFirstEndpoint.set(lineSegment.getFirstEndpointX(), lineSegment.getFirstEndpointY());
         graphics.drawPoint(tempFirstEndpoint);
      }
      else
      {
         tempLineSegment.set(lineSegment);
         graphics.drawLineSegment(tempLineSegment);
      }

      if (drawArrow)
      {
         arrowHeadVector.set(lineSegment.getSecondEndpointX() - lineSegment.getFirstEndpointX(), lineSegment.getSecondEndpointY() - lineSegment.getFirstEndpointY());
         arrowHeadVector.normalize();
         arrowHeadLateralVector.set(arrowHeadVector.getY(), -arrowHeadVector.getX());
         
         arrowHeadVector.scale(arrowHeadHeight);
         arrowHeadLateralVector.scale(arrowHeadWidth / 2.0);
         
         ((Tuple2DBasics) tempArrowPolygon.getVertex(0)).set(lineSegment.getSecondEndpointX(), lineSegment.getSecondEndpointY());
         
         ((Tuple2DBasics) tempArrowPolygon.getVertex(1)).set(lineSegment.getSecondEndpointX(), lineSegment.getSecondEndpointY());
         ((Tuple2DBasics) tempArrowPolygon.getVertex(1)).sub(arrowHeadVector);
         ((Tuple2DBasics) tempArrowPolygon.getVertex(1)).sub(arrowHeadLateralVector);
         
         ((Tuple2DBasics) tempArrowPolygon.getVertex(2)).set(lineSegment.getSecondEndpointX(), lineSegment.getSecondEndpointY());
         ((Tuple2DBasics) tempArrowPolygon.getVertex(2)).sub(arrowHeadVector);
         ((Tuple2DBasics) tempArrowPolygon.getVertex(2)).add(arrowHeadLateralVector);
         
         graphics.drawPolygonFilled(tempArrowPolygon);
      }
   }

   @Override
   public void drawLegend(Plotter2DAdapter graphics, Point2D origin)
   {
      graphics.setColor(color);
      graphics.setStroke(STROKE);

      graphics.drawLineSegment(graphics.getScreenFrame(), -20.0 + origin.getX(), -5.0 + origin.getY(), 20.0 + origin.getX(), 5.0 + origin.getY());
   }

   @Override
   public void drawHistoryEntry(Graphics2DAdapter graphics, double[] entry)
   {
      tempLineSegment.set(entry[0], entry[1], entry[2], entry[3]);
   }

   @Override
   public YoArtifact duplicate(YoVariableRegistry newRegistry)
   {
      return new YoArtifactLineSegment2d(getName(), lineSegment.duplicate(newRegistry), color, arrowHeadWidth, arrowHeadHeight);
   }
}
