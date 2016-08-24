package us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting;

import java.awt.BasicStroke;
import java.awt.Color;
import java.util.ArrayList;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearanceRGBColor;
import us.ihmc.plotting.Graphics2DAdapter;
import us.ihmc.plotting.PlotterGraphics;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.LineSegment2d;
import us.ihmc.robotics.math.frames.YoFrameLineSegment2d;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class YoArtifactLineSegment2d extends YoArtifact
{
   private static final BasicStroke STROKE = new BasicStroke(2);
   private static final double[] CONSTANTS = new double[] {};

   private final YoFrameLineSegment2d lineSegment;
   
   private final LineSegment2d tempLineSegment = new LineSegment2d();
   
   private final PlotterGraphics plotterGraphics = new PlotterGraphics();
   private final Color color;

   private final boolean drawArrow;
   private double arrowHeadWidth;
   private double arrowHeadHeight;

   private ArrayList<Point2d> arrowHeadPoints = null;
   private Vector2d arrowHeadVector = null;
   private Vector2d arrowHeadLateralVector = null;

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
      super(name, lineSegment.getYoX0(), lineSegment.getYoY0(), lineSegment.getYoX1(), lineSegment.getYoY1());
      this.lineSegment = lineSegment;
      this.color = color;
      this.drawArrow = true;
      this.arrowHeadWidth = arrowHeadWidth;
      this.arrowHeadHeight = arrowHeadHeight;
   }

   public YoArtifactLineSegment2d(String name, YoFrameLineSegment2d lineSegment, Color color)
   {
      super(name, lineSegment.getYoX0(), lineSegment.getYoY0(), lineSegment.getYoX1(), lineSegment.getYoY1());
      this.lineSegment = lineSegment;
      this.color = color;
      this.drawArrow = false;
   }

   @Override
   public void draw(Graphics2DAdapter graphics, int Xcenter, int Ycenter, double headingOffset, double scaleFactor)
   {
      if (isVisible)
      {
         graphics.setColor(color);

         if (STROKE != null)
            graphics.setStroke(STROKE);

         
         
         plotterGraphics.setCenter(Xcenter, Ycenter);
         plotterGraphics.setScale(scaleFactor);

         plotterGraphics.drawLineSegment(graphics, lineSegment.getX0(), lineSegment.getY0(), lineSegment.getX1(), lineSegment.getY1());

         if (drawArrow)
         {
            computeArrowHeadPoints(arrowHeadPoints, lineSegment.getX0(), lineSegment.getY0(), lineSegment.getX1(), lineSegment.getY1());
            plotterGraphics.fillPolygon(graphics, arrowHeadPoints);
         }
      }
   }

   private void computeArrowHeadPoints(ArrayList<Point2d> arrowHeadPointsToPack, double x0, double y0, double x1, double y1)
   {
      if (arrowHeadPoints == null)
      {
         arrowHeadPoints = new ArrayList<>();
         arrowHeadVector = new Vector2d();
         arrowHeadLateralVector = new Vector2d();
      }

      arrowHeadVector.set(x1 - x0, y1 - y0);
      arrowHeadVector.normalize();
      arrowHeadLateralVector.set(arrowHeadVector.getY(), -arrowHeadVector.getX());

      arrowHeadVector.scale(arrowHeadHeight);
      arrowHeadLateralVector.scale(arrowHeadWidth);

      int i = 0;
      Point2d arrowHeadTopCorner = getOrCreate(arrowHeadPointsToPack, i++);
      arrowHeadTopCorner.set(x1, y1);

      Point2d arrowHeadLeftCorner = getOrCreate(arrowHeadPointsToPack, i++);
      arrowHeadLeftCorner.set(x1, y1);
      arrowHeadLeftCorner.sub(arrowHeadVector);
      arrowHeadLeftCorner.add(arrowHeadLateralVector);

      Point2d arrowHeadRightCorner = getOrCreate(arrowHeadPointsToPack, i++);
      arrowHeadRightCorner.set(x1, y1);
      arrowHeadRightCorner.sub(arrowHeadVector);
      arrowHeadRightCorner.sub(arrowHeadLateralVector);
   }

   private Point2d getOrCreate(ArrayList<Point2d> arrowHeadPointsToPack, int index)
   {
      while (index >= arrowHeadPoints.size())
         arrowHeadPoints.add(new Point2d());
      return arrowHeadPoints.get(index);
   }

   @Override
   public void drawLegend(Graphics2DAdapter graphics, int Xcenter, int Ycenter, double scaleFactor)
   {
      graphics.setColor(color);
      if (STROKE != null)
         graphics.setStroke(STROKE);

      graphics.drawLine(-20 + Xcenter, -5 + Ycenter, 20 + Xcenter, 5 + Ycenter);
   }

   @Override
   public void drawHistory(Graphics2DAdapter graphics2d, int Xcenter, int Ycenter, double scaleFactor)
   {
      throw new RuntimeException("Not implemented!");
   }

   @Override
   public void takeHistorySnapshot()
   {
      throw new RuntimeException("Not implemented!");
   }

   @Override
   public RemoteGraphicType getRemoteGraphicType()
   {
      return RemoteGraphicType.LINE_SEGMENT_2D_ARTIFACT;
   }

   @Override
   public double[] getConstants()
   {
      return CONSTANTS;
   }

   @Override
   public AppearanceDefinition getAppearance()
   {
      return new YoAppearanceRGBColor(color, 0.0);
   }
}
