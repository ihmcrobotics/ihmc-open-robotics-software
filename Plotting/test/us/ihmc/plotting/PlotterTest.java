package us.ihmc.plotting;

import javax.vecmath.Point2d;

import org.junit.Test;

import us.ihmc.plotting.shapes.LineArtifact;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

public class PlotterTest
{
   @DeployableTestMethod(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testBasicPlotter()
   {
      BasicPlotter plotter = new BasicPlotter(600, 400);
      
      plotter.setShowGridLines(true);

      plotter.addArtifact(new LineArtifact("01", new Point2d(0, 0), new Point2d(1, 1)));
      plotter.addArtifact(new LineArtifact("02", new Point2d(1, 1), new Point2d(2, 0)));
      plotter.addArtifact(new LineArtifact("03", new Point2d(2, 0), new Point2d(3, 1)));
      
      plotter.showInNewWindow();
   }
   
   @DeployableTestMethod(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testPlotter()
   {
      Plotter plotter = new Plotter();

      plotter.addArtifact(new LineArtifact("01", new Point2d(0, 0), new Point2d(1, 1)));
      plotter.addArtifact(new LineArtifact("02", new Point2d(1, 1), new Point2d(2, 0)));
      plotter.addArtifact(new LineArtifact("03", new Point2d(2, 0), new Point2d(3, 1)));
      
      plotter.showInNewWindow();
   }
}
