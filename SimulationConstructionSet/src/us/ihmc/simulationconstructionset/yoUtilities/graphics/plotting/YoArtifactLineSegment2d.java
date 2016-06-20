package us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.util.ArrayList;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearanceRGBColor;
import us.ihmc.plotting.Artifact;
import us.ihmc.plotting.PlotterGraphics;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.frames.YoFrameLineSegment2d;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.RemoteYoGraphic;

public class YoArtifactLineSegment2d extends Artifact implements RemoteYoGraphic
{
   private static final double[] CONSTANTS = new double[] {};

   private static final long serialVersionUID = -2067732984899803547L;

   private final DoubleYoVariable[] variables = new DoubleYoVariable[4];
   private final DoubleYoVariable startX;
   private final DoubleYoVariable endX;
   private final DoubleYoVariable startY;
   private final DoubleYoVariable endY;
   private final PlotterGraphics plotterGraphics = new PlotterGraphics();
   private final Color color;
   private static final int pixels = 2;
   private static final BasicStroke stroke = new BasicStroke(pixels);

   private boolean drawArrow = false;
   private final double arrowHeadWidth;
   private final double arrowHeadHeight;

   public YoArtifactLineSegment2d(String name, YoFrameLineSegment2d yoFrameLineSegment2d, Color color)
   {
      this(name, yoFrameLineSegment2d, color, Double.NaN, Double.NaN);
   }

   public YoArtifactLineSegment2d(String name, YoFrameLineSegment2d yoFrameLineSegment2d, Color color, double arrowHeadWidth, double arrowHeadHeight)
   {
      this(name, yoFrameLineSegment2d.getYoX0(), yoFrameLineSegment2d.getYoY0(), yoFrameLineSegment2d.getYoX1(), yoFrameLineSegment2d.getYoY1(), color, arrowHeadWidth, arrowHeadHeight);
   }

   public YoArtifactLineSegment2d(String name, YoFramePoint2d startPoint, YoFramePoint2d endPoint, Color color)
   {
      this(name, startPoint.getYoX(), startPoint.getYoY(), endPoint.getYoX(), endPoint.getYoY(), color, Double.NaN, Double.NaN);
   }

   public YoArtifactLineSegment2d(String name, YoFramePoint2d startPoint, YoFramePoint2d endPoint, Color color, double arrowHeadWidth, double arrowHeadHeight)
   {
      this(name, startPoint.getYoX(), startPoint.getYoY(), endPoint.getYoX(), endPoint.getYoY(), color, arrowHeadWidth, arrowHeadHeight);
   }

   public YoArtifactLineSegment2d(String name, DoubleYoVariable startX, DoubleYoVariable startY, DoubleYoVariable endX, DoubleYoVariable endY, Color color, double arrowHeadWidth, double arrowHeadHeight)
   {
      super(name);
      this.startX = startX;
      this.endX = endX;
      this.startY = startY;
      this.endY = endY;
      this.color = color;
      this.arrowHeadWidth = arrowHeadWidth;
      this.arrowHeadHeight = arrowHeadHeight;

      variables[0] = startX;
      variables[1] = endX;
      variables[2] = startY;
      variables[3] = endY;

      drawArrow = true;
   }

   @Override
   public void draw(Graphics graphics, int Xcenter, int Ycenter, double headingOffset, double scaleFactor)
   {
      if (isVisible)
      {
         graphics.setColor(color);

         if (stroke != null)
            ((Graphics2D) graphics).setStroke(stroke);

         plotterGraphics.setCenter(Xcenter, Ycenter);
         plotterGraphics.setScale(scaleFactor);

         double x0 = startX.getDoubleValue();
         double x1 = endX.getDoubleValue();
         double y0 = startY.getDoubleValue();
         double y1 = endY.getDoubleValue();

         if (startX.isNaN() || endX.isNaN() || startY.isNaN() || endY.isNaN())
            return;

         plotterGraphics.drawLineSegment(graphics, x0, y0, x1, y1);

         if (drawArrow)
         {
            computeArrowHeadPoints(arrowHeadPoints, x0, y0, x1, y1);
            plotterGraphics.fillPolygon(graphics, arrowHeadPoints);
         }
      }
   }

   private ArrayList<Point2d> arrowHeadPoints = null;
   private Vector2d arrowHeadVector = null;
   private Vector2d arrowHeadLateralVector = null;

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
   public void drawLegend(Graphics graphics, int Xcenter, int Ycenter, double scaleFactor)
   {
      graphics.setColor(color);
      if (stroke != null)
         ((Graphics2D) graphics).setStroke(stroke);

//      plotterGraphics.setCenter(Xcenter, Ycenter);
//      plotterGraphics.setScale(scaleFactor);
//      plotterGraphics.drawLineSegment(graphics, 0.0, 0.0, 0.1, 0.1);
      graphics.drawLine(-20 + Xcenter, -5 + Ycenter, 20 + Xcenter, 5 + Ycenter);
   }

   @Override
   public void drawHistory(Graphics g, int Xcenter, int Ycenter, double scaleFactor)
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
   public DoubleYoVariable[] getVariables()
   {
      return variables;
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
