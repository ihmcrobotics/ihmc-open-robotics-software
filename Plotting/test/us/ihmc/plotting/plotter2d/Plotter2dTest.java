package us.ihmc.plotting.plotter2d;

import javax.vecmath.Point2d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.plotting.plotter2d.frames.MetersReferenceFrame;
import us.ihmc.plotting.plotter2d.frames.PixelsReferenceFrame;
import us.ihmc.plotting.plotter2d.frames.PlotterFrameSpace;
import us.ihmc.plotting.plotter2d.frames.PlotterSpaceConverter;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.Transform3d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.testing.JUnitTools;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

public class Plotter2dTest
{
   @DeployableTestMethod(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testPlotter2d()
   {
      Transform3d pixelsToWorld = new Transform3d();
      pixelsToWorld.setScale(0.5, 0.5, 0.5);
      Transform3d worldToPixels = new Transform3d();
      worldToPixels.invert(pixelsToWorld);
      
      FramePoint2d point = new FramePoint2d(ReferenceFrame.getWorldFrame());
      
      point.set(1.0, 0.0);
      
      System.out.println(point);
      
      Vector3d vector3d = new Vector3d(point.getX(), point.getY(), 0.0);
      worldToPixels.transform(vector3d);
      point.set(vector3d.getX(), vector3d.getY());
      
      System.out.println(point);
   }
   
   @SuppressWarnings("serial")
   @DeployableTestMethod(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testReferenceFrames()
   {
      ReferenceFrame pixelFrame = new ReferenceFrame("pixelFrame", ReferenceFrame.getWorldFrame(), new Transform3d(), false, false, false)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            ((Transform3d) transformToParent).setScale(0.5, 0.5, 0.5);
         }
      };
      
      //      ReferenceFrame screenFrame = new ReferenceFrame("screenFrame", pixelFrame, transform3d, false, false, false)
//      {
//         @Override
//         protected void updateTransformToParent(RigidBodyTransform transformToParent)
//         {
////            ((Transform3d) transformToParent).setRotationEulerAndZeroTranslation(0.0, 0.0, Math.PI);
//            ((Transform3d) transformToParent).setTranslation(1.0, 0.0, 0.0);
//         }
//      };
      
      pixelFrame.update();
//      screenFrame.update();
      
      FramePoint2d point = new FramePoint2d(ReferenceFrame.getWorldFrame());
      
      point.set(1.0, 0.0);
      
      System.out.println(point);
      
      point.changeFrame(pixelFrame);
      
      System.out.println(point);
      
//      point.changeFrame(screenFrame);
      
      System.out.println(point);
   }
   
   @SuppressWarnings("serial")
   @DeployableTestMethod(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testSeperateReferenceFrames()
   {
      ReferenceFrame screenFrame = new ReferenceFrame("screenFrame", ReferenceFrame.getWorldFrame())
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.setTranslation(50.0, 100.0, 0.0);
            transformToParent.applyRotationZ(Math.PI);
         }
      };
      
      screenFrame.update();
      
      FramePoint2d point = new FramePoint2d(ReferenceFrame.getWorldFrame());
      
      point.set(1.0, 0.0);
      
      System.out.println(point);
      
      changeToPixelFrame(point);
      
      System.out.println(point);
      
      point.changeFrame(screenFrame);
      
      System.out.println(point);
   }
   
   private void changeToPixelFrame(FramePoint2d point)
   {
      point.changeFrame(ReferenceFrame.getWorldFrame());
      point.scale(5.0);
   }
   
   @SuppressWarnings("serial")
   @DeployableTestMethod(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testNewWayOfThings()
   {
      PlotterSpaceConverter spaceConverter = new PlotterSpaceConverter()
      {
         @Override
         public double getConversionToSpace(PlotterFrameSpace plotterFrameType)
         {
            return plotterFrameType == PlotterFrameSpace.METERS ? 0.1 : 10.0;
         }
      };
      
      PixelsReferenceFrame pixelsFrame = new PixelsReferenceFrame("pixelsFrame", ReferenceFrame.getWorldFrame(), spaceConverter)
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
         }
      };
      
      PixelsReferenceFrame screenFrame = new PixelsReferenceFrame("screenFrame", ReferenceFrame.getWorldFrame(), spaceConverter)
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
      JUnitTools.assertPoint2dEquals("Point not equal", new Point2d(10.0, 50.0), point.getPoint(), 1e-7);
      
      point.changeFrame(screenFrame);
      
      System.out.println(point);
      JUnitTools.assertPoint2dEquals("Point not equal", new Point2d(40.0, 50.0), point.getPoint(), 1e-7);
      
      point.changeFrame(metersFrame);
      
      System.out.println(point);
      JUnitTools.assertPoint2dEquals("Point not equal", new Point2d(1.0, 5.0), point.getPoint(), 1e-7);
   }
}
