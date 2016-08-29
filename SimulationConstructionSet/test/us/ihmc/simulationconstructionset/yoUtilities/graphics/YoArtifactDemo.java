package us.ihmc.simulationconstructionset.yoUtilities.graphics;

import java.awt.Color;

import javax.vecmath.Point2d;

import us.ihmc.plotting.Plotter;
import us.ihmc.plotting.artifact.LineArtifact;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFrameLine2d;
import us.ihmc.robotics.math.frames.YoFrameLineSegment2d;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting.YoArtifactLine2d;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting.YoArtifactLineSegment2d;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting.YoArtifactOval;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting.YoArtifactPolygon;

public class YoArtifactDemo
{
   public void showPlotter()
   {
      Plotter plotter = new Plotter();
      plotter.setPreferredSize(800, 600);
      
      plotter.setViewRange(10.0);
      plotter.setXYZoomEnabled(true);
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
      YoFramePoint2d polyPoint1 = new YoFramePoint2d("point1", ReferenceFrame.getWorldFrame(), registry);
      polyPoint1.set(1.5, 2.0);
      polygon2.setConvexPolygon2d(new FramePoint2d[] {polyPoint1.getFrameTuple2d()});
      
      YoFrameConvexPolygon2d polygon3 = new YoFrameConvexPolygon2d("polygon2", ReferenceFrame.getWorldFrame(), 5, registry);
      YoFramePoint2d polyPoint2 = new YoFramePoint2d("point2", ReferenceFrame.getWorldFrame(), registry);
      polyPoint2.set(2.0, 2.5);
      YoFramePoint2d polyPoint3 = new YoFramePoint2d("point3", ReferenceFrame.getWorldFrame(), registry);
      polyPoint3.set(1.0, 3.0);
      polygon3.setConvexPolygon2d(new FramePoint2d[] {polyPoint1.getFrameTuple2d(), polyPoint2.getFrameTuple2d(), polyPoint3.getFrameTuple2d()});
      
      plotter.addArtifact(new LineArtifact("01", new Point2d(0, 0), new Point2d(1, 1)));
      plotter.addArtifact(new LineArtifact("02", new Point2d(1, 1), new Point2d(2, 0)));
      plotter.addArtifact(new LineArtifact("03", new Point2d(2, 0), new Point2d(3, 1)));
      plotter.addArtifact(new YoArtifactOval("circle", center, radii, Color.RED));
      plotter.addArtifact(new YoArtifactLineSegment2d("lineSegment1", lineSegment, Color.DARK_GRAY, 0.1, 0.1));
      plotter.addArtifact(new YoArtifactLine2d("line1", line, Color.GREEN));
      plotter.addArtifact(new YoArtifactPolygon("emptyPolygon1", polygon, Color.MAGENTA, false)); 
      plotter.addArtifact(new YoArtifactPolygon("onePointPolygon", polygon2, Color.MAGENTA, false)); 
      plotter.addArtifact(new YoArtifactPolygon("twoPointPolygon", polygon3, Color.BLUE, true)); 
      
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
