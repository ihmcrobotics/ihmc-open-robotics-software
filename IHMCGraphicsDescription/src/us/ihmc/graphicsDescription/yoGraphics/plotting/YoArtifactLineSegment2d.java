package us.ihmc.graphicsDescription.yoGraphics.plotting;

import java.awt.BasicStroke;
import java.awt.Color;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.graphicsDescription.plotting.Graphics2DAdapter;
import us.ihmc.graphicsDescription.plotting.Plotter2DAdapter;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.LineSegment2d;
import us.ihmc.robotics.math.frames.YoFrameLineSegment2d;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class YoArtifactLineSegment2d extends YoArtifact
{
   private static final BasicStroke STROKE = new BasicStroke(2);

   private final YoFrameLineSegment2d lineSegment;
   
   private final Point2D tempFirstEndpoint = new Point2D();
   private final LineSegment2d tempLineSegment = new LineSegment2d();
   private final ConvexPolygon2d tempArrowPolygon = new ConvexPolygon2d(new double[][] {{0.0, 0.0}, {0.0, 0.0}, {0.0, 0.0}});
   
   private final boolean drawArrow;

   // Arrow only object
   private double arrowHeadWidth;
   private double arrowHeadHeight;
   private Vector2D arrowHeadVector;
   private Vector2D arrowHeadLateralVector;
   
   public YoArtifactLineSegment2d(String name, YoFramePoint2d startPoint, YoFramePoint2d endPoint, Color color, double arrowHeadWidth, double arrowHeadHeight)
   {
      this(name, startPoint.getYoX(), startPoint.getYoY(), endPoint.getYoX(), endPoint.getYoY(), color, arrowHeadWidth, arrowHeadHeight);
   }

   private YoArtifactLineSegment2d(String name, DoubleYoVariable startX, DoubleYoVariable startY, DoubleYoVariable endX, DoubleYoVariable endY, Color color, double arrowHeadWidth, double arrowHeadHeight)
   {
      this(name, new YoFrameLineSegment2d(startX, startY, endX, endY, ReferenceFrame.getWorldFrame()), color, arrowHeadWidth, arrowHeadHeight);
   }
   
   public YoArtifactLineSegment2d(String name, YoFramePoint2d start, YoFramePoint2d end, Color color)
   {
      this(name, new YoFrameLineSegment2d(start.getYoX(), start.getYoY(), end.getYoX(), end.getYoY(), ReferenceFrame.getWorldFrame()), color);
   }
   
   public YoArtifactLineSegment2d(String name, YoFrameLineSegment2d lineSegment, Color color, double arrowHeadWidth, double arrowHeadHeight)
   {
      super(name, new double[0], color,
            lineSegment.getYoFirstEndpointX(), lineSegment.getYoFirstEndpointY(), lineSegment.getYoSecondEndpointX(), lineSegment.getYoSecondEndpointY());
      this.lineSegment = lineSegment;
      this.drawArrow = true;
      instatiateArrowObjects(arrowHeadWidth, arrowHeadHeight);
   }

   public YoArtifactLineSegment2d(String name, YoFrameLineSegment2d lineSegment, Color color)
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

      if (lineSegment.areEndpointsTheSame())
      {
         tempFirstEndpoint.set(lineSegment.getFirstEndpointX(), lineSegment.getFirstEndpointY());
         graphics.drawPoint(tempFirstEndpoint);
      }
      else
      {
         lineSegment.getFrameLineSegment2d().get(tempLineSegment);
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
   public RemoteGraphicType getRemoteGraphicType()
   {
      return RemoteGraphicType.LINE_SEGMENT_2D_ARTIFACT;
   }
}
