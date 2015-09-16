package us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting;

import java.awt.Color;
import java.awt.Graphics;
import java.util.ArrayList;

import javax.vecmath.Color3f;
import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

import us.ihmc.plotting.Artifact;
import us.ihmc.plotting.PlotterGraphics;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class YoArtifactVector extends Artifact
{
   private static final long serialVersionUID = 374256294846738354L;

   private final PlotterGraphics plotterGraphics = new PlotterGraphics();
   private final Color3f color;
   private final YoFramePoint2d basePoint;
   private final YoFrameVector2d vector;
   private final Point2d endPoint = new Point2d();
   ArrayList<Point2d> arrowHeadPoints = new ArrayList<Point2d>();

   private final static double ARROW_HEAD_WIDTH = 0.08;
   private final static double ARROW_HEAD_HEIGHT = 0.12;

   public YoArtifactVector(String name, YoFramePoint2d basePoint, YoFrameVector2d vector, Color3f color)
   {
      super(name);
      this.basePoint = basePoint;
      this.vector = vector;
      this.color = color;
   }

   public YoArtifactVector(String name, DoubleYoVariable basePointX, DoubleYoVariable basePointY, DoubleYoVariable vectorX, DoubleYoVariable vectorY, Color3f color)
   {
      super(name);
      this.basePoint = new YoFramePoint2d(basePointX, basePointY, ReferenceFrame.getWorldFrame());
      this.vector = new YoFrameVector2d(vectorX, vectorY, ReferenceFrame.getWorldFrame());
      this.color = color;
   }

   public void draw(Graphics graphics, int Xcenter, int Ycenter, double headingOffset, double scaleFactor)
   {
      graphics.setColor(new Color(color.getX(), color.getY(), color.getZ()));

      basePoint.get(endPoint);
      endPoint.x += vector.getX();
      endPoint.y += vector.getY();

      arrowHeadPoints = getArrowHeadPoints(vector.getFrameVector2dCopy().getVectorCopy(), endPoint);

      plotterGraphics.setCenter(Xcenter, Ycenter);
      plotterGraphics.setScale(scaleFactor);

      plotterGraphics.drawLineSegment(graphics, basePoint.getX(), basePoint.getY(), endPoint.getX(), endPoint.getY());
      plotterGraphics.fillPolygon(graphics, arrowHeadPoints);
   }

   private ArrayList<Point2d> getArrowHeadPoints(Vector2d vector, Point2d endPoint)
   {
      ArrayList<Point2d> ret = new ArrayList<Point2d>();

      Vector2d arrowHeadVector = new Vector2d(vector);
      arrowHeadVector.normalize();
      arrowHeadVector.scale(ARROW_HEAD_HEIGHT);

      Vector2d arrowHeadLateralVector = new Vector2d(vector.getY(), -vector.getX());
      arrowHeadLateralVector.normalize();
      arrowHeadLateralVector.scale(ARROW_HEAD_WIDTH);

      Point2d arrowHeadTopCorner = new Point2d(endPoint);
      arrowHeadTopCorner.add(arrowHeadVector);
      ret.add(arrowHeadTopCorner);

      Point2d arrowHeadLeftCorner = new Point2d(endPoint);
      arrowHeadLeftCorner.add(arrowHeadLateralVector);
      ret.add(arrowHeadLeftCorner);

      Point2d arrowHeadRightCorner = new Point2d(endPoint);
      arrowHeadRightCorner.sub(arrowHeadLateralVector);
      ret.add(arrowHeadRightCorner);

      return ret;
   }

   public void drawLegend(Graphics graphics, int Xcenter, int Ycenter, double scaleFactor)
   {
      graphics.setColor(new Color(color.getX(), color.getY(), color.getZ()));

      graphics.drawLine(Xcenter, Ycenter, Xcenter + 20, Ycenter);

      int[] x = { Xcenter + 20, Xcenter + 20, Xcenter + 25 };
      int[] y = { Ycenter + 5, Ycenter - 5, Ycenter };
      graphics.fillPolygon(x, y, 3);
   }

   public void drawHistory(Graphics g, int Xcenter, int Ycenter, double scaleFactor)
   {
      throw new RuntimeException("Not implemented!");
   }

   public void takeHistorySnapshot()
   {
      throw new RuntimeException("Not implemented!");
   }

}