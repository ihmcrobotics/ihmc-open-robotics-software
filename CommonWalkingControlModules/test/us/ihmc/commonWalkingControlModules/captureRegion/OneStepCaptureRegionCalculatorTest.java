package us.ihmc.commonWalkingControlModules.captureRegion;


import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.awt.Color;
import java.util.ArrayList;

import javax.media.j3d.Transform3D;
import javax.vecmath.Point2d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.geometry.FrameConvexPolygon2d;
import us.ihmc.utilities.math.geometry.FrameGeometry2dPlotter;
import us.ihmc.utilities.math.geometry.FrameGeometryTestFrame;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class OneStepCaptureRegionCalculatorTest
{
   private static final boolean PLOT_RESULTS = false;
   private static final boolean WAIT_FOR_BUTTON_PUSH = true;
   
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   
   private final ReferenceFrame leftAnkleZUpFrame = new SimpleAnkleZUpReferenceFrame("leftAnkleZUp");
   private final ReferenceFrame rightAnkleZUpFrame = new SimpleAnkleZUpReferenceFrame("rightAnkleZUp");
   private final SideDependentList<ReferenceFrame> ankleZUpFrames = new SideDependentList<ReferenceFrame>(leftAnkleZUpFrame, rightAnkleZUpFrame);
   
   private final YoVariableRegistry registry = new YoVariableRegistry("CaptureRegionCalculatorTest");
   
   @SuppressWarnings("unused")
   @Test
   public void testConstructor()
   {
      double midFootAnkleXOffset = 0.0;
      double footWidth = 0.5;
      double kineamaticStepRange = 3.0;
      
      OneStepCaptureRegionCalculator captureRegionCalculator = new OneStepCaptureRegionCalculator(
            midFootAnkleXOffset, footWidth, kineamaticStepRange, ankleZUpFrames, registry, null);
   }
   
   @Test
   public void testPointsInsideCaptureRegion()
   {
      // do not change parameters
      // expected results are pre-calculated
      double midFootAnkleXOffset = 0.1;
      double footWidth = 0.5;
      double footLength = 1.0;
      double kineamaticStepRange = 3.0;
      
      RobotSide swingSide = RobotSide.RIGHT;
      double swingTimeRemaining = 0.2;
      double omega0 = 3.0;
      
      OneStepCaptureRegionCalculator captureRegionCalculator = new OneStepCaptureRegionCalculator(
            midFootAnkleXOffset, footWidth, kineamaticStepRange, ankleZUpFrames, registry, null);
      
      ArrayList<Point2d> listOfPoints = new ArrayList<Point2d>();
      listOfPoints.add(new Point2d(-footLength/2.0, -footWidth/2.0));
      listOfPoints.add(new Point2d(-footLength/2.0, footWidth/2.0));
      listOfPoints.add(new Point2d(footLength/2.0, -footWidth/2.0));
      listOfPoints.add(new Point2d(footLength/2.0, footWidth/2.0));
      FrameConvexPolygon2d supportFootPolygon = new FrameConvexPolygon2d(worldFrame, listOfPoints);
      
      FramePoint2d icp = new FramePoint2d(worldFrame, 0.6, -0.5);
      captureRegionCalculator.calculateCaptureRegion(swingSide, swingTimeRemaining, icp, omega0, supportFootPolygon);
      FrameConvexPolygon2d captureRegion = captureRegionCalculator.getCaptureRegion();
      
      ArrayList<FramePoint2d> expectedPointsOnBorder = new ArrayList<FramePoint2d>();
      ReferenceFrame supportAnkleFrame = ankleZUpFrames.get(swingSide.getOppositeSide());
      
      // Points that are used to construct a approximation of the expected capture region
      expectedPointsOnBorder.add(new FramePoint2d(supportAnkleFrame, 1.504330, -0.705530));  // norm: 1.66156
      expectedPointsOnBorder.add(new FramePoint2d(supportAnkleFrame, 0.682212, -0.705530));  // norm: 0.98142
      expectedPointsOnBorder.add(new FramePoint2d(supportAnkleFrame, 0.682212, -1.116590));  // norm: 1.30851
      expectedPointsOnBorder.add(new FramePoint2d(supportAnkleFrame, 2.716116, -1.273857));
      expectedPointsOnBorder.add(new FramePoint2d(supportAnkleFrame, 2.085382, -2.156660));
      expectedPointsOnBorder.add(new FramePoint2d(supportAnkleFrame, 1.564096, -2.559988));
      
      double movePointsFactor = 0.01;
      FrameConvexPolygon2d expectedPolygonInside = new FrameConvexPolygon2d(expectedPointsOnBorder);
      expectedPolygonInside = FrameConvexPolygon2d.shrinkConstantDistanceInto(movePointsFactor, expectedPolygonInside);
      
      assertTrue(expectedPolygonInside.getConvexPolygon2d().isCompletelyInside(captureRegion.getConvexPolygon2d()));
      
      ArrayList<FramePoint2d> expectedPointsOutside = new ArrayList<FramePoint2d>();
      movePointsFactor = 1.03;
      expectedPointsOutside.add(new FramePoint2d(supportAnkleFrame, movePointsFactor * 2.716116, movePointsFactor * -1.273857));
      expectedPointsOutside.add(new FramePoint2d(supportAnkleFrame, movePointsFactor * 2.085382, movePointsFactor * -2.156660));
      expectedPointsOutside.add(new FramePoint2d(supportAnkleFrame, movePointsFactor * 1.564096, movePointsFactor * -2.559988));
      movePointsFactor = 0.97;
      expectedPointsOutside.add(new FramePoint2d(supportAnkleFrame, movePointsFactor * 1.504330, movePointsFactor * -0.705530));
      expectedPointsOutside.add(new FramePoint2d(supportAnkleFrame, movePointsFactor * 0.682212, movePointsFactor * -0.705530));
      expectedPointsOutside.add(new FramePoint2d(supportAnkleFrame, movePointsFactor * 0.682212, movePointsFactor * -1.116590));
      
      for(int i = 0; i < expectedPointsOutside.size(); i++)
      {
         assertFalse(captureRegion.isPointInside(expectedPointsOutside.get(i)));
      }
      
      if (PLOT_RESULTS)
      {
         FrameGeometryTestFrame testFrame = new FrameGeometryTestFrame(-5, 5, -5, 5);
         FrameGeometry2dPlotter plotter = testFrame.getFrameGeometry2dPlotter();
         plotter.setDrawPointsLarge();
         plotter.addPolygon(supportFootPolygon, Color.black);
         plotter.addPolygon(captureRegion, Color.green);
         plotter.addFramePoint2d(icp, Color.blue);
         for(int i = 0; i < expectedPointsOnBorder.size(); i++)
         {
            plotter.addFramePoint2d(expectedPointsOnBorder.get(i), Color.red);
         }
         plotter.addPolygon(expectedPolygonInside, Color.red);
         for(int i = 0; i < expectedPointsOutside.size(); i++)
         {
            plotter.addFramePoint2d(expectedPointsOutside.get(i), Color.cyan);
         }
         waitForButtonOrPause(testFrame);
      }
   }
   
   @Test
   public void testProjectedFootCorners()
   {
      // do not change parameters
      // expected results are pre-calculated
      double midFootAnkleXOffset = 0.1;
      double footWidth = 0.5;
      double footLength = 1.0;
      double kineamaticStepRange = 3.0;
      
      RobotSide swingSide = RobotSide.RIGHT;
      double swingTimeRemaining = 0.2;
      double omega0 = 3.0;
      
      OneStepCaptureRegionCalculator captureRegionCalculator = new OneStepCaptureRegionCalculator(
            midFootAnkleXOffset, footWidth, kineamaticStepRange, ankleZUpFrames, registry, null);
      
      ArrayList<Point2d> listOfPoints = new ArrayList<Point2d>();
      listOfPoints.add(new Point2d(-footLength/2.0, -footWidth/2.0));
      listOfPoints.add(new Point2d(-footLength/2.0, footWidth/2.0));
      listOfPoints.add(new Point2d(footLength/2.0, -footWidth/2.0));
      listOfPoints.add(new Point2d(footLength/2.0, footWidth/2.0));
      FrameConvexPolygon2d supportFootPolygon = new FrameConvexPolygon2d(worldFrame, listOfPoints);
      
      FramePoint2d icp = new FramePoint2d(worldFrame, 0.6, -0.5);
      captureRegionCalculator.calculateCaptureRegion(swingSide, swingTimeRemaining, icp, omega0, supportFootPolygon);
      FrameConvexPolygon2d captureRegion = captureRegionCalculator.getCaptureRegion();
      
      ArrayList<FramePoint2d> expectedPointsOnBorder = new ArrayList<FramePoint2d>();
      ArrayList<FramePoint2d> expectedPointsInside = new ArrayList<FramePoint2d>();
      ReferenceFrame supportAnkleFrame = ankleZUpFrames.get(swingSide.getOppositeSide());
      
      // Points that are expected to be vertexes on the capture region border
      expectedPointsOnBorder.add(new FramePoint2d(supportAnkleFrame, 1.50433, -0.705530));
      expectedPointsOnBorder.add(new FramePoint2d(supportAnkleFrame, 0.682212, -0.705530));
      expectedPointsOnBorder.add(new FramePoint2d(supportAnkleFrame, 0.682212, -1.11659));
      
      for(int i = 0; i < expectedPointsOnBorder.size(); i++)
      {
         FramePoint2d closestVertex = captureRegion.getClosestVertexCopy(expectedPointsOnBorder.get(i));
         assertTrue(closestVertex.epsilonEquals(expectedPointsOnBorder.get(i), 10e-7));
      }
      
      // Points that are expected to be inside the capture region
      expectedPointsInside.add(new FramePoint2d(supportAnkleFrame, 1.50433, -1.11659));
      
      for(int i = 0; i < expectedPointsInside.size(); i++)
      {
         assertTrue(captureRegion.isPointInside(expectedPointsInside.get(i)));
      }
      
      if (PLOT_RESULTS)
      {
         FrameGeometryTestFrame testFrame = new FrameGeometryTestFrame(-5, 5, -5, 5);
         FrameGeometry2dPlotter plotter = testFrame.getFrameGeometry2dPlotter();
         plotter.setDrawPointsLarge();
         plotter.addPolygon(supportFootPolygon, Color.black);
         plotter.addPolygon(captureRegion, Color.green);
         plotter.addFramePoint2d(icp, Color.blue);
         
         for(int i = 0; i < expectedPointsOnBorder.size(); i++)
         {
            plotter.addFramePoint2d(expectedPointsOnBorder.get(i), Color.red);
         }
         for(int i = 0; i < expectedPointsInside.size(); i++)
         {
            plotter.addFramePoint2d(expectedPointsInside.get(i), Color.magenta);
         }
         
         waitForButtonOrPause(testFrame);
      }
   }
   
   @Test
   public void testCalculationWithICPInFootPolygon()
   {
      double midFootAnkleXOffset = 0.2;
      double footWidth = 0.5;
      double footLength = 1.0;
      double kineamaticStepRange = 3.0;
      
      RobotSide swingSide = RobotSide.LEFT;
      double swingTimeRemaining = 0.2;
      double omega0 = 3.0;
      
      OneStepCaptureRegionCalculator captureRegionCalculator = new OneStepCaptureRegionCalculator(
            midFootAnkleXOffset, footWidth, kineamaticStepRange, ankleZUpFrames, registry, null);
      // set the cutoff angle such that the reachable region should be a half circle
      captureRegionCalculator.setReachableRegionCutoffAngle(1.0);
      
      ArrayList<Point2d> listOfPoints = new ArrayList<Point2d>();
      listOfPoints.add(new Point2d(-footLength/2.0, -footWidth/2.0));
      listOfPoints.add(new Point2d(-footLength/2.0, footWidth/2.0));
      listOfPoints.add(new Point2d(footLength/2.0, -footWidth/2.0));
      listOfPoints.add(new Point2d(footLength/2.0, footWidth/2.0));
      FrameConvexPolygon2d supportFootPolygon = new FrameConvexPolygon2d(worldFrame, listOfPoints);
      
      // set the icp to be inside the foot polygon
      FramePoint2d icp = new FramePoint2d(worldFrame, 0.0, 0.0);
      captureRegionCalculator.calculateCaptureRegion(swingSide, swingTimeRemaining, icp, omega0, supportFootPolygon);
      FrameConvexPolygon2d captureRegion = captureRegionCalculator.getCaptureRegion();
      
      // check that capture region is reachable region
      ArrayList<FramePoint2d> testPointsInside = new ArrayList<FramePoint2d>();
      ArrayList<FramePoint2d> testPointsOutside = new ArrayList<FramePoint2d>();
      ReferenceFrame supportAnkleFrame = ankleZUpFrames.get(swingSide.getOppositeSide());
      double sign = swingSide == RobotSide.LEFT ? 1.0 : -1.0;
      
      testPointsInside.add(new FramePoint2d(supportAnkleFrame, kineamaticStepRange*0.95 + midFootAnkleXOffset, sign*0.1));
      testPointsInside.add(new FramePoint2d(supportAnkleFrame, -kineamaticStepRange*0.95 + midFootAnkleXOffset, sign*0.1));
      testPointsInside.add(new FramePoint2d(supportAnkleFrame,  midFootAnkleXOffset, sign*kineamaticStepRange*0.95));
      
      testPointsOutside.add(new FramePoint2d(supportAnkleFrame, kineamaticStepRange*1.05 + midFootAnkleXOffset, sign*0.1));
      testPointsOutside.add(new FramePoint2d(supportAnkleFrame, -kineamaticStepRange*1.05 + midFootAnkleXOffset, sign*0.1));
      testPointsOutside.add(new FramePoint2d(supportAnkleFrame, midFootAnkleXOffset, -sign*0.1));
      testPointsOutside.add(new FramePoint2d(supportAnkleFrame, midFootAnkleXOffset, sign*kineamaticStepRange*1.05));
      
      for(int i = 0; i < testPointsInside.size(); i++)
      {
         assertTrue(captureRegion.isPointInside(testPointsInside.get(i)));
      }
      
      for(int i = 0; i < testPointsOutside.size(); i++)
      {
         assertFalse(captureRegion.isPointInside(testPointsOutside.get(i)));
      }
      
      if (PLOT_RESULTS)
      {
         FrameGeometryTestFrame testFrame = new FrameGeometryTestFrame(-5, 5, -5, 5);
         FrameGeometry2dPlotter plotter = testFrame.getFrameGeometry2dPlotter();
         plotter.setDrawPointsLarge();
         plotter.addPolygon(supportFootPolygon, Color.black);
         plotter.addPolygon(captureRegion, Color.green);
         for(int i = 0; i < testPointsInside.size(); i++)
         {
            plotter.addFramePoint2d(testPointsInside.get(i), Color.green);
         }
         for(int i = 0; i < testPointsOutside.size(); i++)
         {
            plotter.addFramePoint2d(testPointsOutside.get(i), Color.red);
         }
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
