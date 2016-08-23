package us.ihmc.simulationconstructionset.yoUtilities.graphics;

import java.awt.Color;

import javax.vecmath.Point2d;

import us.ihmc.plotting.Plotter;
import us.ihmc.plotting.artifact.LineArtifact;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.math.frames.YoFramePoint2d;
import us.ihmc.robotics.math.frames.YoFrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.plotting.YoArtifactCircle;

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
      
      plotter.addArtifact(new LineArtifact("01", new Point2d(0, 0), new Point2d(1, 1)));
      plotter.addArtifact(new LineArtifact("02", new Point2d(1, 1), new Point2d(2, 0)));
      plotter.addArtifact(new LineArtifact("03", new Point2d(2, 0), new Point2d(3, 1)));
      plotter.addArtifact(new YoArtifactCircle("circle", center, radii, Color.RED));
      
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
