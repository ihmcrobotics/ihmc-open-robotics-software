package us.ihmc.simulationconstructionset.plotting;

import java.awt.Color;
import java.awt.Graphics;
import java.util.ArrayList;

import javax.vecmath.Color3f;
import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearanceRGBColor;
import us.ihmc.plotting.Artifact;
import us.ihmc.plotting.PlotterGraphics;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicVector;


public class DynamicGraphicVectorArtifact extends Artifact
{
   private static final long serialVersionUID = 2379549723591744368L;
   private final YoGraphicVector dynamicGraphicVector;
   private final PlotterGraphics plotterGraphics = new PlotterGraphics();
   private final Color3f color = new Color3f();
   private final Point3d basePoint3d = new Point3d();
   private final Vector3d vector3d = new Vector3d();
   private final Point2d basePoint = new Point2d();
   private final Vector2d vector = new Vector2d();
   private final Point2d endPoint = new Point2d();
   ArrayList<Point2d> arrowHeadPoints = new ArrayList<Point2d>();

   private final static double ARROW_HEAD_WIDTH = 0.02;
   private final static double ARROW_HEAD_HEIGHT = 0.03;

   public DynamicGraphicVectorArtifact(String name, YoGraphicVector dynamicGraphicVector)
   {
      super(name);
      this.dynamicGraphicVector = dynamicGraphicVector;

//      dynamicGraphicVector.getAppearance().getMaterial().getAmbientColor(color);
      AppearanceDefinition appearance = dynamicGraphicVector.getAppearance();
      if(appearance instanceof YoAppearanceRGBColor)
      {
         color.set(((YoAppearanceRGBColor) appearance).getRed(), ((YoAppearanceRGBColor) appearance).getGreen(), ((YoAppearanceRGBColor) appearance).getBlue());
      }
   }

   public void draw(Graphics graphics, int Xcenter, int Ycenter, double headingOffset, double scaleFactor)
   {
      graphics.setColor(new Color(color.getX(), color.getY(), color.getZ()));

      dynamicGraphicVector.getBasePosition(basePoint3d);
      dynamicGraphicVector.getVector(vector3d);

      basePoint.set(basePoint3d.getX(), basePoint3d.getY());
      vector.set(vector3d.getX(), vector3d.getY());

      endPoint.set(basePoint);
      endPoint.add(vector);
      
      arrowHeadPoints = getArrowHeadPoints(vector, endPoint);

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