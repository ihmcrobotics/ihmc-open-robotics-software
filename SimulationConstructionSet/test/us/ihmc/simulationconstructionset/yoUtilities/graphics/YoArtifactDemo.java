package us.ihmc.simulationconstructionset.yoUtilities.graphics;

import java.awt.Color;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.graphicsDescription.plotting.artifact.LineArtifact;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition.GraphicType;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactLine2d;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactLineSegment2d;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactOval;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPolygon;
import us.ihmc.graphicsDescription.yoGraphics.plotting.YoArtifactPosition;
import us.ihmc.plotting.Plotter;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFrameLine2d;
import us.ihmc.robotics.math.frames.YoFrameLineSegment2d;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class YoArtifactDemo
{
   public void showPlotter()
   {
      Plotter plotter = new Plotter();
      plotter.setPreferredSize(800, 600);
      
      plotter.setViewRange(10.0);
      plotter.setXYZoomEnabled(true);
      plotter.setRotationEnabled(true);
//      plotter.setViewRange(1.0);
      plotter.setShowLabels(true);
      
      YoVariableRegistry registry = new YoVariableRegistry("plotterDemo");
      
      YoFramePoint2d center = new YoFramePoint2d("center", ReferenceFrame.getWorldFrame(), registry);
      YoFrameVector2d radii = new YoFrameVector2d("radii", ReferenceFrame.getWorldFrame(), registry);
      center.set(-1.0, -1.0);
      radii.set(1.0, 0.7);
      
      YoFrameLineSegment2d lineSegment = new YoFrameLineSegment2d("segmentGuy", "", ReferenceFrame.getWorldFrame(), registry);
      lineSegment.getYoFirstEndpointX().set(1.0);
      lineSegment.getYoFirstEndpointY().set(1.0);
      lineSegment.getYoSecondEndpointX().set(2.0);
      lineSegment.getYoSecondEndpointY().set(2.0);
      
      YoFrameLine2d line = new YoFrameLine2d("line", "", ReferenceFrame.getWorldFrame(), registry);
      line.getYoPointX().set(-1.0);
      line.getYoPointY().set(1.0);
      line.getYoVectorX().set(-0.5);
      line.getYoVectorY().set(1.0);
      
      YoFrameConvexPolygon2d polygon = new YoFrameConvexPolygon2d("polygon", ReferenceFrame.getWorldFrame(), 5, registry);
      
      YoFrameConvexPolygon2d polygon2 = new YoFrameConvexPolygon2d("polygon1", ReferenceFrame.getWorldFrame(), 5, registry);
      YoFramePoint2d point1 = new YoFramePoint2d("point1", ReferenceFrame.getWorldFrame(), registry);
      YoFramePoint2d polyPoint1 = point1;
      polyPoint1.set(1.5, 2.0);
      polygon2.setConvexPolygon2d(new FramePoint2d[] {polyPoint1.getFrameTuple2d()});
      
      YoFrameConvexPolygon2d polygon3 = new YoFrameConvexPolygon2d("polygon2", ReferenceFrame.getWorldFrame(), 5, registry);
      YoFramePoint2d polyPoint2 = new YoFramePoint2d("point2", ReferenceFrame.getWorldFrame(), registry);
      polyPoint2.set(2.0, 2.5);
      YoFramePoint2d polyPoint3 = new YoFramePoint2d("point3", ReferenceFrame.getWorldFrame(), registry);
      polyPoint3.set(1.0, 3.0);
      polygon3.setConvexPolygon2d(new FramePoint2d[] {polyPoint1.getFrameTuple2d(), polyPoint2.getFrameTuple2d(), polyPoint3.getFrameTuple2d()});
      
      YoFramePoint2d pointZ = new YoFramePoint2d("pointZ", ReferenceFrame.getWorldFrame(), registry);
      pointZ.set(-2.2, 3.0);
      YoFramePoint2d point4 = new YoFramePoint2d("point4", ReferenceFrame.getWorldFrame(), registry);
      point4.set(-2.0, 3.0);
      YoFramePoint2d point5 = new YoFramePoint2d("point5", ReferenceFrame.getWorldFrame(), registry);
      point5.set(-1.8, 3.0);
      YoFramePoint2d point6 = new YoFramePoint2d("point6", ReferenceFrame.getWorldFrame(), registry);
      point6.set(-1.6, 3.0);
      YoFramePoint2d point7 = new YoFramePoint2d("point7", ReferenceFrame.getWorldFrame(), registry);
      point7.set(-1.4, 3.0);
      YoFramePoint2d point8 = new YoFramePoint2d("point8", ReferenceFrame.getWorldFrame(), registry);
      point8.set(-1.2, 3.0);
      YoFramePoint2d point9 = new YoFramePoint2d("point9", ReferenceFrame.getWorldFrame(), registry);
      point9.set(-1.0, 3.0);
      YoFramePoint2d point10 = new YoFramePoint2d("point10", ReferenceFrame.getWorldFrame(), registry);
      point10.set(-0.8, 3.0);
      YoFramePoint2d point11 = new YoFramePoint2d("point11", ReferenceFrame.getWorldFrame(), registry);
      point11.set(-0.6, 3.0);
      YoFramePoint2d point12 = new YoFramePoint2d("point12", ReferenceFrame.getWorldFrame(), registry);
      point12.set(-0.4, 3.0);
      YoFramePoint2d point13 = new YoFramePoint2d("point13", ReferenceFrame.getWorldFrame(), registry);
      point13.set(-0.2, 3.0);
      
      
      plotter.addArtifact(new LineArtifact("01", new Point2D(0, 0), new Point2D(1, 1)));
      plotter.addArtifact(new LineArtifact("02", new Point2D(1, 1), new Point2D(2, 0)));
      plotter.addArtifact(new LineArtifact("03", new Point2D(2, 0), new Point2D(3, 1)));
      plotter.addArtifact(new YoArtifactOval("circle", center, radii, Color.RED));
      plotter.addArtifact(new YoArtifactLineSegment2d("lineSegment1", lineSegment, Color.DARK_GRAY, 0.1, 0.1));
      plotter.addArtifact(new YoArtifactLine2d("line1", line, Color.GREEN));
      plotter.addArtifact(new YoArtifactPolygon("emptyPolygon1", polygon, Color.MAGENTA, false)); 
      plotter.addArtifact(new YoArtifactPolygon("onePointPolygon", polygon2, Color.MAGENTA, false)); 
      plotter.addArtifact(new YoArtifactPolygon("twoPointPolygon", polygon3, Color.BLUE, true)); 
      plotter.addArtifact(new YoArtifactPosition("pointZ", pointZ, GraphicType.BALL, Color.DARK_GRAY, 0.1)); 
      plotter.addArtifact(new YoArtifactPosition("point4", point4, GraphicType.BALL_WITH_CROSS, Color.DARK_GRAY, 0.1)); 
      plotter.addArtifact(new YoArtifactPosition("point5", point5, GraphicType.BALL_WITH_ROTATED_CROSS, Color.DARK_GRAY, 0.1)); 
      plotter.addArtifact(new YoArtifactPosition("point6", point6, GraphicType.CROSS, Color.DARK_GRAY, 0.1)); 
      plotter.addArtifact(new YoArtifactPosition("point7", point7, GraphicType.DIAMOND, Color.DARK_GRAY, 0.1)); 
      plotter.addArtifact(new YoArtifactPosition("point8", point8, GraphicType.DIAMOND_WITH_CROSS, Color.DARK_GRAY, 0.1)); 
      plotter.addArtifact(new YoArtifactPosition("point9", point9, GraphicType.ELLIPSOID, Color.DARK_GRAY, 0.1)); 
      plotter.addArtifact(new YoArtifactPosition("point10", point10, GraphicType.ROTATED_CROSS, Color.DARK_GRAY, 0.1)); 
      plotter.addArtifact(new YoArtifactPosition("point11", point11, GraphicType.SOLID_BALL, Color.DARK_GRAY, 0.1)); 
      plotter.addArtifact(new YoArtifactPosition("point12", point12, GraphicType.SQUARE, Color.DARK_GRAY, 0.1)); 
      plotter.addArtifact(new YoArtifactPosition("point13", point13, GraphicType.SQUARE_WITH_CROSS, Color.DARK_GRAY, 0.1)); 
      
      plotter.showInNewWindow("plotterDemo", true);
      
//      plotter.setScale(40.0, 20.0);
//      plotter.setFocusPointX(2.5);
//      plotter.setFocusPointY(-3.0);
   }
   
   public static void main(String[] args)
   {
      YoArtifactDemo plotterDemo = new YoArtifactDemo();
      plotterDemo.showPlotter();
   }
}
