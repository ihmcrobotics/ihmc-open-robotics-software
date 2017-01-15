package us.ihmc.plotting;

import static org.junit.Assert.assertEquals;

import javax.vecmath.Point2d;
import javax.vecmath.Vector2d;

import org.junit.Test;

import us.ihmc.graphics3DDescription.plotting.PlotterPoint2d;
import us.ihmc.graphics3DDescription.plotting.artifact.LineArtifact;
import us.ihmc.graphics3DDescription.plotting.frames.MetersReferenceFrame;
import us.ihmc.graphics3DDescription.plotting.frames.PixelsReferenceFrame;
import us.ihmc.graphics3DDescription.plotting.frames.PlotterFrameSpace;
import us.ihmc.graphics3DDescription.plotting.frames.PlotterSpaceConverter;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.testing.JUnitTools;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.tools.continuousIntegration.IntegrationCategory;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationTools;

@ContinuousIntegrationPlan(categories = IntegrationCategory.UI)
public class PlotterTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 300000)
   public void testPlotter()
   {
      Plotter plotter = new Plotter();

      plotter.addArtifact(new LineArtifact("01", new Point2d(0, 0), new Point2d(1, 1)));
      plotter.addArtifact(new LineArtifact("02", new Point2d(1, 1), new Point2d(2, 0)));
      plotter.addArtifact(new LineArtifact("03", new Point2d(2, 0), new Point2d(3, 1)));

      plotter.showInNewWindow();
      
      if (!ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer())
      {
         ThreadTools.sleepForever();
      }
   }

   @SuppressWarnings("serial")
   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testPlotterReferenceFrames()
   {
      PlotterSpaceConverter spaceConverter = new PlotterSpaceConverter()
      {
         private Vector2d scaleVector = new Vector2d();

         @Override
         public Vector2d getConversionToSpace(PlotterFrameSpace plotterFrameType)
         {
            if (plotterFrameType == PlotterFrameSpace.METERS)
            {
               scaleVector.set(0.1, 0.2);
            }
            else
            {
               scaleVector.set(10.0, 5.0);
            }
            return scaleVector;
         }
      };

      PixelsReferenceFrame pixelsFrame = new PixelsReferenceFrame("pixelsFrame", ReferenceFrame.getWorldFrame(), spaceConverter)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
         }
      };

      PixelsReferenceFrame screenFrame = new PixelsReferenceFrame("screenFrame", pixelsFrame, spaceConverter)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.setTranslation(50.0, 100.0, 0.0);
            transformToParent.applyRotationZ(Math.PI);
         }
      };

      MetersReferenceFrame metersFrame = new MetersReferenceFrame("metersFrame", ReferenceFrame.getWorldFrame(), spaceConverter)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
         }
      };

      pixelsFrame.update();
      metersFrame.update();
      screenFrame.update();

      PlotterPoint2d point = new PlotterPoint2d(metersFrame);

      point.set(1.0, 5.0);

      System.out.println(point);
      JUnitTools.assertPoint2dEquals("Point not equal", new Point2d(1.0, 5.0), point.getPoint(), 1e-7);

      point.changeFrame(pixelsFrame);

      System.out.println(point);
      JUnitTools.assertPoint2dEquals("Point not equal", new Point2d(10.0, 25.0), point.getPoint(), 1e-7);

      point.changeFrame(screenFrame);

      System.out.println(point);
      JUnitTools.assertPoint2dEquals("Point not equal", new Point2d(40.0, 75.0), point.getPoint(), 1e-7);

      point.changeFrame(metersFrame);

      System.out.println(point);
      JUnitTools.assertPoint2dEquals("Point not equal", new Point2d(1.0, 5.0), point.getPoint(), 1e-7);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testBuildAPlotterAndCallSomeStuff()
   {
      Plotter plotter = new Plotter();
      plotter.setPreferredSize(800, 600);

      plotter.setScale(10.0, 10.0);
      plotter.setShowLabels(true);

      //      plotter.addArtifact(new LineArtifact("01", new Point2d(0, 0), new Point2d(1, 1)));
      //      plotter.addArtifact(new LineArtifact("02", new Point2d(1, 1), new Point2d(2, 0)));
      //      plotter.addArtifact(new LineArtifact("03", new Point2d(2, 0), new Point2d(3, 1)));

      plotter.showInNewWindow();

      plotter.setScale(40.0, 20.0);
      plotter.setFocusPointX(-5.0);
      plotter.setFocusPointY(10.0);

      assertEquals("focus point x not correct", -5.0, plotter.getFocusPointX(), 1e-7);
      assertEquals("focus point y not correct", 10.0, plotter.getFocusPointY(), 1e-7);
   }
}
