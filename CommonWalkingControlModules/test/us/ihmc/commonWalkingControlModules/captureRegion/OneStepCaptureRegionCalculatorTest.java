package us.ihmc.commonWalkingControlModules.captureRegion;

import java.awt.Color;
import java.util.ArrayList;

import javax.media.j3d.Transform3D;
import javax.vecmath.Point2d;
import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;

import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FrameGeometry2dPlotter;
import us.ihmc.utilities.math.geometry.FrameGeometryTestFrame;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public class OneStepCaptureRegionCalculatorTest
{
   private static final boolean PLOT_RESULTS = true;
   private static final boolean WAIT_FOR_BUTTON_PUSH = true;
   
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   
   private final ReferenceFrame leftAnkleZUpFrame = new SimpleAnkleZUpReferenceFrame("leftAnkleZUp");
   private final ReferenceFrame rightAnkleZUpFrame = new SimpleAnkleZUpReferenceFrame("rightAnkleZUp");
   private final SideDependentList<ReferenceFrame> ankleZUpFrames = new SideDependentList<ReferenceFrame>(leftAnkleZUpFrame, rightAnkleZUpFrame);
   
   private final double midFootAnkleXOffset = 0.0;
   private final double footWidth = 0.5;
   private final double kineamaticStepRange = 5.0;
   
   private final YoVariableRegistry registry = new YoVariableRegistry("CaptureRegionCalculatorTest");
   private final DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry = new DynamicGraphicObjectsListRegistry();
   
   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }
   
   @After
   public void showMemoryUsageAfterTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
   
   @SuppressWarnings("unused")
   @Test
   public void testConstructor()
   {
      OneStepCaptureRegionCalculator captureRegionCalculator = new OneStepCaptureRegionCalculator(
            midFootAnkleXOffset, footWidth, kineamaticStepRange, ankleZUpFrames, registry, dynamicGraphicObjectsListRegistry);
   }
   
   //@Test
   public void testCalculation()
   {
      OneStepCaptureRegionCalculator captureRegionCalculator = new OneStepCaptureRegionCalculator(
            midFootAnkleXOffset, footWidth, kineamaticStepRange, ankleZUpFrames, registry, dynamicGraphicObjectsListRegistry);
      
      RobotSide swingSide = RobotSide.RIGHT;
      double swingTimeRemaining = 0.5;
      FramePoint2d icp = new FramePoint2d(worldFrame, 1.0, 1.0);
      double omega0 = 3.0;
      ArrayList<Point2d> listOfPoints = new ArrayList<Point2d>();
      listOfPoints.add(new Point2d(0.0, 0.0));
      listOfPoints.add(new Point2d(footWidth, 0.0));
      listOfPoints.add(new Point2d(0.0, 1.0));
      listOfPoints.add(new Point2d(footWidth, 1.0));
      FrameConvexPolygon2d supportFootPolygon = new FrameConvexPolygon2d(worldFrame, listOfPoints);
      
      captureRegionCalculator.calculateCaptureRegion(swingSide, swingTimeRemaining, icp, omega0, supportFootPolygon);
      FrameConvexPolygon2d captureRegion = captureRegionCalculator.getCaptureRegion();
      
      // do some tests here
      
      if (PLOT_RESULTS)
      {
         // Plot:
         FrameGeometryTestFrame testFrame = new FrameGeometryTestFrame(-5, 10, -5, 10);
         FrameGeometry2dPlotter plotter = testFrame.getFrameGeometry2dPlotter();
         plotter.setDrawPointsLarge();
         plotter.addPolygon(captureRegion);
//         plotter.addFramePoint2d(frontmostLeft, Color.green);
//         plotter.addFramePoint2d(frontmostRight, Color.orange);
//         plotter.addFramePoint2d(backmostRight, Color.red);
//         plotter.addFramePoint2d(backmostLeft, Color.black);

         waitForButtonOrPause(testFrame);
      }
   }
   
   private class  SimpleAnkleZUpReferenceFrame extends ReferenceFrame
   {
      private static final long serialVersionUID = -2855876641425187923L;
      private final Vector3d offset = new Vector3d();
      
      public SimpleAnkleZUpReferenceFrame(String name)
      {
         super(name, ReferenceFrame.getWorldFrame());
      }

      public void updateTransformToParent(Transform3D transformToParent)
      {
         transformToParent.setIdentity();
         transformToParent.setTranslation(offset);
      }
   }
   
   private void waitForButtonOrPause(FrameGeometryTestFrame testFrame)
   {
      if (WAIT_FOR_BUTTON_PUSH)
         testFrame.waitForButtonPush();
      else
         pauseOneSecond();
   }
   
   private void pauseOneSecond()
   {
      try
      {
         Thread.sleep(1000);
      }
      catch (InterruptedException ex)
      {
      }
   }
}
