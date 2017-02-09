package us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.smoothICPGenerator;

import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.Random;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePointTest;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.Line2d;
import us.ihmc.robotics.lists.FrameTupleArrayList;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.tools.testing.JUnitTools;

public class CapturePointToolsTest
{
   int nTests = 20;
   Random random = new Random();
   YoVariableRegistry registry = new YoVariableRegistry("");

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testComputeConstantCMPPointsWithBeginningAndEndBetweenFeetWith2Steps()
   {
      int nFootsteps = 2;
      FrameTupleArrayList<FramePoint> footstepList = FrameTupleArrayList.createFramePointArrayList(nFootsteps);
      ArrayList<YoFramePoint> arrayToPack = new ArrayList<YoFramePoint>();
      PoseReferenceFrame poseReferenceFrame = new PoseReferenceFrame("test", new FramePose());

      int numberFootstepsToConsider = 4;
      for (int i = 0; i < numberFootstepsToConsider; i++)
      {
         arrayToPack.add(new YoFramePoint("test" + Integer.toString(i), ReferenceFrame.getWorldFrame(), registry));
      }
      for (int i = 0; i < nFootsteps; i++)
      {
         FramePose pose = new FramePose(ReferenceFrame.getWorldFrame(), new Point3d(random.nextDouble(), random.nextDouble(), random.nextDouble()),
               new Quat4d(random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble()));

         poseReferenceFrame.setPoseAndUpdate(pose);

         Point3d pointToPack = new Point3d();
         Quat4d orientation = new Quat4d();
         poseReferenceFrame.getPosition(pointToPack);
         poseReferenceFrame.getOrientation(orientation);
         FramePoint footstepData = footstepList.getAndGrowIfNeeded(i);
         footstepData.setIncludingFrame(poseReferenceFrame.getRootFrame(), pointToPack);
      }

      boolean startStanding = true;
      boolean endStanding = true;
      int lastFootstepIndex = Math.min(footstepList.size(), numberFootstepsToConsider) - 1;
      CapturePointTools.computeConstantCMPs(arrayToPack, footstepList, 0, lastFootstepIndex, startStanding, endStanding);

      Point3d pointBetweenFeet = new Point3d();
      pointBetweenFeet.set(footstepList.get(0).getPointCopy());
      pointBetweenFeet.add(footstepList.get(1).getPointCopy());
      pointBetweenFeet.scale(0.5);

      for (int i = 0; i < footstepList.size(); i++)
      {
         JUnitTools.assertPoint3dEquals("", arrayToPack.get(i).getPoint3dCopy(), pointBetweenFeet, 1e-10);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testComputeConstantCMPPointsOnFeetAndEndBetweenFeetWith2Steps()
   {
      int nFootsteps = 2;
      FrameTupleArrayList<FramePoint> footstepList = FrameTupleArrayList.createFramePointArrayList(nFootsteps);
      ArrayList<YoFramePoint> arrayToPack = new ArrayList<YoFramePoint>();
      PoseReferenceFrame poseReferenceFrame = new PoseReferenceFrame("test", new FramePose());

      int numberFootstepsToConsider = 4;
      for (int i = 0; i < numberFootstepsToConsider; i++)
      {
         arrayToPack.add(new YoFramePoint("test" + Integer.toString(i), ReferenceFrame.getWorldFrame(), registry));
      }
      for (int i = 0; i < nFootsteps; i++)
      {
         FramePose pose = new FramePose(ReferenceFrame.getWorldFrame(), new Point3d(random.nextDouble(), random.nextDouble(), random.nextDouble()),
               new Quat4d(random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble()));

         poseReferenceFrame.setPoseAndUpdate(pose);

         Point3d pointToPack = new Point3d();
         Quat4d orientation = new Quat4d();
         poseReferenceFrame.getPosition(pointToPack);
         poseReferenceFrame.getOrientation(orientation);
         FramePoint footstepData = footstepList.getAndGrowIfNeeded(i);
         footstepData.setIncludingFrame(poseReferenceFrame.getRootFrame(), pointToPack);
      }

      boolean startStanding = false;
      boolean endStanding = true;
      int lastFootstepIndex = Math.min(footstepList.size(), numberFootstepsToConsider) - 1;
      CapturePointTools.computeConstantCMPs(arrayToPack, footstepList, 0, lastFootstepIndex, startStanding, endStanding);

      Point3d pointBetweenFeet = new Point3d();
      pointBetweenFeet.set(footstepList.get(0).getPointCopy());
      pointBetweenFeet.add(footstepList.get(1).getPointCopy());
      pointBetweenFeet.scale(0.5);

      JUnitTools.assertPoint3dEquals("", footstepList.get(0).getPointCopy(), arrayToPack.get(0).getPoint3dCopy(), 1e-10);

      for (int i = 1; i < footstepList.size(); i++)
      {
         JUnitTools.assertPoint3dEquals("", arrayToPack.get(i).getPoint3dCopy(), pointBetweenFeet, 1e-10);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testComputeConstantCentersOfPressuresOnFeet()
   {
      int nFootsteps = 10;
      FrameTupleArrayList<FramePoint> footstepList = FrameTupleArrayList.createFramePointArrayList(nFootsteps);
      ArrayList<YoFramePoint> arrayToPack = new ArrayList<YoFramePoint>();
      PoseReferenceFrame poseReferenceFrame = new PoseReferenceFrame("test", new FramePose());

      int numberFootstepsToConsider = 3;
      for (int i = 0; i < numberFootstepsToConsider; i++)
      {
         arrayToPack.add(new YoFramePoint("test" + Integer.toString(i), ReferenceFrame.getWorldFrame(), registry));
      }

      for (int j = 0; j < nTests; j++)
      {
         for (int i = 0; i < nFootsteps; i++)
         {
            FramePose pose = new FramePose(ReferenceFrame.getWorldFrame(), new Point3d(random.nextDouble(), random.nextDouble(), random.nextDouble()),
                  new Quat4d(random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble()));

            poseReferenceFrame.setPoseAndUpdate(pose);

            Point3d pointToPack = new Point3d();
            Quat4d orientation = new Quat4d();
            poseReferenceFrame.getPosition(pointToPack);
            poseReferenceFrame.getOrientation(orientation);
            FramePoint footstepData = footstepList.getAndGrowIfNeeded(i);
            footstepData.setIncludingFrame(poseReferenceFrame.getRootFrame(), pointToPack);
         }

         CapturePointTools.computeConstantCMPsOnFeet(arrayToPack, footstepList, 0, numberFootstepsToConsider - 1);

         for (int i = 0; i < arrayToPack.size(); i++)
         {
            JUnitTools.assertPoint3dEquals("", arrayToPack.get(i).getPoint3dCopy(), footstepList.get(i).getPointCopy(), 1e-10);
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testComputeConstantCentersOfPressureWithStartBetweenFeetAndRestOnFeet()
   {
      int nFootsteps = 10;
      FrameTupleArrayList<FramePoint> footstepList = FrameTupleArrayList.createFramePointArrayList(nFootsteps);
      ArrayList<YoFramePoint> arrayToPack = new ArrayList<YoFramePoint>();
      PoseReferenceFrame poseReferenceFrame = new PoseReferenceFrame("test", new FramePose());

      int numberFootstepsToConsider = 3;
      for (int i = 0; i < numberFootstepsToConsider; i++)
      {
         arrayToPack.add(new YoFramePoint("test" + Integer.toString(i), ReferenceFrame.getWorldFrame(), registry));
      }

      for (int j = 0; j < nTests; j++)
      {
         for (int i = 0; i < nFootsteps; i++)
         {
            FramePose pose = new FramePose(ReferenceFrame.getWorldFrame(), new Point3d(random.nextDouble(), random.nextDouble(), random.nextDouble()),
                  new Quat4d(random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble()));

            poseReferenceFrame.setPoseAndUpdate(pose);

            Point3d pointToPack = new Point3d();
            Quat4d orientation = new Quat4d();
            poseReferenceFrame.getPosition(pointToPack);
            poseReferenceFrame.getOrientation(orientation);
            FramePoint footstepData = footstepList.getAndGrowIfNeeded(i);
            footstepData.setIncludingFrame(poseReferenceFrame.getRootFrame(), pointToPack);
         }

         boolean startStanding = true;
         boolean endStanding = false;
         int lastFootstepIndex = Math.min(footstepList.size(), numberFootstepsToConsider) - 1;
         CapturePointTools.computeConstantCMPs(arrayToPack, footstepList, 0, lastFootstepIndex, startStanding, endStanding);

         Point3d copPos1 = new Point3d();
         copPos1.set(footstepList.get(0).getPointCopy());
         copPos1.add(footstepList.get(1).getPointCopy());
         copPos1.scale(0.5);
         JUnitTools.assertPoint3dEquals("", copPos1, arrayToPack.get(0).getPoint3dCopy(), 1e-10);
         for (int i = 1; i < arrayToPack.size() - 1; i++)
         {
            JUnitTools.assertPoint3dEquals("", arrayToPack.get(i).getPoint3dCopy(), footstepList.get(i).getPointCopy(), 1e-10);
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testComputeConstantCentersOfPressureWithEndBetweenFeetAndRestOnFeet()
   {
      int nFootsteps = 10;
      FrameTupleArrayList<FramePoint> footstepList = FrameTupleArrayList.createFramePointArrayList(nFootsteps);
      ArrayList<YoFramePoint> arrayToPack = new ArrayList<YoFramePoint>();
      PoseReferenceFrame poseReferenceFrame = new PoseReferenceFrame("test", new FramePose());

      int numberFootstepsToConsider = 3;
      for (int i = 0; i < numberFootstepsToConsider; i++)
      {
         arrayToPack.add(new YoFramePoint("test" + Integer.toString(i), ReferenceFrame.getWorldFrame(), registry));
      }

      for (int j = 0; j < nTests; j++)
      {
         for (int i = 0; i < nFootsteps; i++)
         {
            FramePose pose = new FramePose(ReferenceFrame.getWorldFrame(), new Point3d(random.nextDouble(), random.nextDouble(), random.nextDouble()),
                  new Quat4d(random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble()));

            poseReferenceFrame.setPoseAndUpdate(pose);

            Point3d pointToPack = new Point3d();
            Quat4d orientation = new Quat4d();
            poseReferenceFrame.getPosition(pointToPack);
            poseReferenceFrame.getOrientation(orientation);
            FramePoint footstepData = footstepList.getAndGrowIfNeeded(i);
            footstepData.setIncludingFrame(poseReferenceFrame.getRootFrame(), pointToPack);
         }

         boolean startStanding = false;
         boolean endStanding = true;
         int lastFootstepIndex = Math.min(footstepList.size(), numberFootstepsToConsider) - 1;
         CapturePointTools.computeConstantCMPs(arrayToPack, footstepList, 0, lastFootstepIndex, startStanding, endStanding);

         Point3d copPos1 = new Point3d();
         copPos1.set(footstepList.get(numberFootstepsToConsider - 1).getPointCopy());
         copPos1.add(footstepList.get(numberFootstepsToConsider - 2).getPointCopy());
         copPos1.scale(0.5);
         JUnitTools.assertPoint3dEquals("", copPos1, arrayToPack.get(numberFootstepsToConsider - 1).getPoint3dCopy(), 1e-10);
         for (int i = 0; i < arrayToPack.size() - 2; i++)
         {
            JUnitTools.assertPoint3dEquals("", arrayToPack.get(i).getPoint3dCopy(), footstepList.get(i).getPointCopy(), 1e-10);
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testComputeConstantCentersOfPressureWithEndAndBeginningBetweenFeet()
   {
      int nFootsteps = 10;
      FrameTupleArrayList<FramePoint> footstepList = FrameTupleArrayList.createFramePointArrayList(nFootsteps);
      ArrayList<YoFramePoint> arrayToPack = new ArrayList<YoFramePoint>();
      PoseReferenceFrame poseReferenceFrame = new PoseReferenceFrame("test", new FramePose());

      int numberFootstepsToConsider = 3;
      for (int i = 0; i < numberFootstepsToConsider; i++)
      {
         arrayToPack.add(new YoFramePoint("test" + Integer.toString(i), ReferenceFrame.getWorldFrame(), registry));
      }

      for (int j = 0; j < nTests; j++)
      {
         for (int i = 0; i < nFootsteps; i++)
         {
            FramePose pose = new FramePose(ReferenceFrame.getWorldFrame(), new Point3d(random.nextDouble(), random.nextDouble(), random.nextDouble()),
                  new Quat4d(random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble()));

            poseReferenceFrame.setPoseAndUpdate(pose);

            Point3d pointToPack = new Point3d();
            Quat4d orientation = new Quat4d();
            poseReferenceFrame.getPosition(pointToPack);
            poseReferenceFrame.getOrientation(orientation);
            FramePoint footstepData = footstepList.getAndGrowIfNeeded(i);
            footstepData.setIncludingFrame(poseReferenceFrame.getRootFrame(), pointToPack);
         }

         boolean startStanding = true;
         boolean endStanding = true;
         int lastFootstepIndex = Math.min(footstepList.size(), numberFootstepsToConsider) - 1;
         CapturePointTools.computeConstantCMPs(arrayToPack, footstepList, 0, lastFootstepIndex, startStanding, endStanding);

         Point3d copPos1 = new Point3d();
         copPos1.set(footstepList.get(numberFootstepsToConsider - 1).getPointCopy());
         copPos1.add(footstepList.get(numberFootstepsToConsider - 2).getPointCopy());
         copPos1.scale(0.5);
         JUnitTools.assertPoint3dEquals("", copPos1, arrayToPack.get(numberFootstepsToConsider - 1).getPoint3dCopy(), 1e-10);

         copPos1.set(footstepList.get(0).getPointCopy());
         copPos1.add(footstepList.get(1).getPointCopy());
         copPos1.scale(0.5);
         JUnitTools.assertPoint3dEquals("", copPos1, arrayToPack.get(0).getPoint3dCopy(), 1e-10);

         for (int i = 1; i < arrayToPack.size() - 2; i++)
         {
            JUnitTools.assertPoint3dEquals("", arrayToPack.get(i).getPoint3dCopy(), footstepList.get(i).getPointCopy(), 1e-10);
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testComputeDesiredEndOfStepCapturePointLocations()
   {
      int nFootsteps = 10;
      FrameTupleArrayList<FramePoint> footstepList = FrameTupleArrayList.createFramePointArrayList(nFootsteps);
      ArrayList<YoFramePoint> constantCentersOfPressures = new ArrayList<YoFramePoint>();
      ArrayList<YoFramePoint> capturePointsToPack = new ArrayList<YoFramePoint>();
      YoFramePoint icpToCheck = new YoFramePoint("icpToCheck", ReferenceFrame.getWorldFrame(), registry);
      PoseReferenceFrame poseReferenceFrame = new PoseReferenceFrame("test", new FramePose());
      FramePoint2d p1 = new FramePoint2d();
      FramePoint2d p2 = new FramePoint2d();

      int numberFootstepsToConsider = random.nextInt(((nFootsteps - 3) + 1)) + 3;
      for (int i = 0; i < numberFootstepsToConsider; i++)
      {
         constantCentersOfPressures.add(new YoFramePoint("test" + Integer.toString(i), ReferenceFrame.getWorldFrame(), registry));
      }
      int lastFootstepIndex = numberFootstepsToConsider - 1;
      for (int i = 0; i < lastFootstepIndex; i++)
      {
         capturePointsToPack.add(new YoFramePoint("testICP" + Integer.toString(i), ReferenceFrame.getWorldFrame(), registry));
      }

      for (int j = 0; j < nTests; j++)
      {
         for (int i = 0; i < numberFootstepsToConsider; i++)
         {
            FramePose pose = new FramePose(ReferenceFrame.getWorldFrame(), new Point3d(random.nextDouble(), random.nextDouble(), random.nextDouble()),
                  new Quat4d(random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble()));

            poseReferenceFrame.setPoseAndUpdate(pose);

            Point3d pointToPack = new Point3d();
            Quat4d orientation = new Quat4d();
            poseReferenceFrame.getPosition(pointToPack);
            poseReferenceFrame.getOrientation(orientation);
            FramePoint footstepData = footstepList.getAndGrowIfNeeded(i);
            footstepData.setIncludingFrame(poseReferenceFrame.getRootFrame(), pointToPack);
         }

         CapturePointTools.putConstantCMPBetweenFeet(constantCentersOfPressures.get(0), footstepList.get(0), footstepList.get(1));
         CapturePointTools.computeConstantCMPsOnFeet(constantCentersOfPressures, footstepList, 1, lastFootstepIndex);
         CapturePointTools.putConstantCMPBetweenFeet(constantCentersOfPressures.get(lastFootstepIndex), footstepList.get(lastFootstepIndex - 1),
               footstepList.get(lastFootstepIndex));
         double omega0 = 0.5;
         double time = 0.5;
         CapturePointTools.computeDesiredCornerPoints(capturePointsToPack, constantCentersOfPressures, false, time, omega0);

         CapturePointTools.computeDesiredCapturePointPosition(omega0, 0, capturePointsToPack.get(0), constantCentersOfPressures.get(0), icpToCheck);

         FramePointTest.assertFramePointEquals(capturePointsToPack.get(0).getFramePointCopy(), icpToCheck.getFramePointCopy(), 1e-8);

         for (int i = 0; i < constantCentersOfPressures.size() - 2; i++)
         {
            p1.set(constantCentersOfPressures.get(i).getFramePoint2dCopy());
            p2.set(capturePointsToPack.get(i).getFramePoint2dCopy());
            Line2d line = new Line2d(p1.getPointCopy(), p2.getPointCopy());
            p1.set(capturePointsToPack.get(i + 1).getFramePoint2dCopy());
            boolean isPointOnLine = line.isPointOnLine(p1.getPoint());
            assertTrue(isPointOnLine);
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testComputeDesiredCapturePointLocations()
   {
      int nFootsteps = 10;
      FrameTupleArrayList<FramePoint> footstepList = FrameTupleArrayList.createFramePointArrayList(nFootsteps);
      ArrayList<YoFramePoint> constantCentersOfPressures = new ArrayList<YoFramePoint>();
      ArrayList<YoFramePoint> capturePointsToPack = new ArrayList<YoFramePoint>();
      PoseReferenceFrame poseReferenceFrame = new PoseReferenceFrame("test", new FramePose());
      FramePoint2d p1 = new FramePoint2d();
      FramePoint2d p2 = new FramePoint2d();
      YoFramePoint desiredICP = new YoFramePoint("", ReferenceFrame.getWorldFrame(), registry);

      int numberFootstepsToConsider = random.nextInt(((nFootsteps - 3) + 1)) + 3;
      for (int i = 0; i < numberFootstepsToConsider; i++)
      {
         constantCentersOfPressures.add(new YoFramePoint("test" + Integer.toString(i), ReferenceFrame.getWorldFrame(), registry));
      }
      for (int i = 0; i < numberFootstepsToConsider - 1; i++)
      {
         capturePointsToPack.add(new YoFramePoint("testICP" + Integer.toString(i), ReferenceFrame.getWorldFrame(), registry));
      }

      for (int j = 0; j < nTests; j++)
      {
         for (int i = 0; i < nFootsteps; i++)
         {
            FramePose pose = new FramePose(ReferenceFrame.getWorldFrame(), new Point3d(random.nextDouble(), random.nextDouble(), random.nextDouble()),
                  new Quat4d(random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble()));

            poseReferenceFrame.setPoseAndUpdate(pose);

            Point3d pointToPack = new Point3d();
            Quat4d orientation = new Quat4d();
            poseReferenceFrame.getPosition(pointToPack);
            poseReferenceFrame.getOrientation(orientation);
            FramePoint footstepData = footstepList.getAndGrowIfNeeded(i);
            footstepData.setIncludingFrame(poseReferenceFrame.getRootFrame(), pointToPack);
         }

         int lastFootstepIndex = numberFootstepsToConsider - 1;
         CapturePointTools.putConstantCMPBetweenFeet(constantCentersOfPressures.get(0), footstepList.get(0), footstepList.get(1));
         CapturePointTools.computeConstantCMPsOnFeet(constantCentersOfPressures, footstepList, 1, lastFootstepIndex);
         CapturePointTools.putConstantCMPBetweenFeet(constantCentersOfPressures.get(lastFootstepIndex), footstepList.get(lastFootstepIndex - 1),
               footstepList.get(lastFootstepIndex));
         double omega0 = 0.5;
         double time = 0.5;
         CapturePointTools.computeDesiredCornerPoints(capturePointsToPack, constantCentersOfPressures, false, time, omega0);

         for (int i = 0; i < constantCentersOfPressures.size() - 2; i++)
         {
            double timeIntoStep = random.nextDouble() * time;
            CapturePointTools.computeDesiredCapturePointPosition(omega0, timeIntoStep, capturePointsToPack.get(i), constantCentersOfPressures.get(i),
                  desiredICP);

            p1.set(constantCentersOfPressures.get(i).getFramePoint2dCopy());
            p2.set(capturePointsToPack.get(i).getFramePoint2dCopy());
            Line2d line = new Line2d(p1.getPointCopy(), p2.getPointCopy());
            p1.set(desiredICP.getFramePoint2dCopy());
            boolean isPointOnLine = line.isPointOnLine(p1.getPoint());
            assertTrue(isPointOnLine);
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testComputeDesiredCapturePointVelocity()
   {
      YoFramePoint initialCapturePointPosition = new YoFramePoint("", ReferenceFrame.getWorldFrame(), registry);
      YoFramePoint computedCapturePoint1 = new YoFramePoint("1", ReferenceFrame.getWorldFrame(), registry);
      YoFramePoint computedCapturePoint2 = new YoFramePoint("2", ReferenceFrame.getWorldFrame(), registry);
      YoFramePoint initialCenterOfPressure = new YoFramePoint("3", ReferenceFrame.getWorldFrame(), registry);
      YoFrameVector differentiatedCapturePointPosition = new YoFrameVector("4", ReferenceFrame.getWorldFrame(), registry);
      YoFrameVector computedCapturePointVelocity = new YoFrameVector("5", ReferenceFrame.getWorldFrame(), registry);

      for (int i = 0; i < nTests; i++)
      {
         initialCapturePointPosition.set(random.nextDouble(), random.nextDouble(), 0);
         initialCenterOfPressure.set(initialCapturePointPosition.getX() + 0.02, initialCapturePointPosition.getY() + 0.01, 0);

         double deltaT = 0.001;
         double time = random.nextDouble() * 0.1 + 0.05;
         double omega0 = 0.5;

         CapturePointTools.computeDesiredCapturePointPosition(omega0, time, initialCapturePointPosition, initialCenterOfPressure, computedCapturePoint1);
         CapturePointTools.computeDesiredCapturePointPosition(omega0, time + deltaT, initialCapturePointPosition, initialCenterOfPressure,
               computedCapturePoint2);

         differentiatedCapturePointPosition.set(computedCapturePoint2);
         differentiatedCapturePointPosition.sub(computedCapturePoint1);
         differentiatedCapturePointPosition.scale(1 / deltaT);

         CapturePointTools.computeDesiredCapturePointVelocity(omega0, time + deltaT, initialCapturePointPosition, initialCenterOfPressure,
               computedCapturePointVelocity);

         JUnitTools.assertPoint3dEquals("", computedCapturePointVelocity.getPoint3dCopy(), differentiatedCapturePointPosition.getPoint3dCopy(), 1e-3);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testComputeDesiredCapturePointAcceleration()
   {
      YoFramePoint initialCapturePointPosition = new YoFramePoint("", ReferenceFrame.getWorldFrame(), registry);
      YoFramePoint initialCenterOfPressure = new YoFramePoint("3", ReferenceFrame.getWorldFrame(), registry);
      YoFrameVector computedCapturePointVelocity = new YoFrameVector("5", ReferenceFrame.getWorldFrame(), registry);
      YoFrameVector desiredCapturePointAcceleration = new YoFrameVector("6", ReferenceFrame.getWorldFrame(), registry);

      for (int i = 0; i < nTests; i++)
      {
         initialCapturePointPosition.set(random.nextDouble(), random.nextDouble(), 0);
         initialCenterOfPressure.set(initialCapturePointPosition.getX() + 0.02, initialCapturePointPosition.getY() + 0.01, 0);

         double time = random.nextDouble() * 0.1 + 0.05;
         double omega0 = 0.5;

         CapturePointTools.computeDesiredCapturePointVelocity(omega0, time, initialCapturePointPosition, initialCenterOfPressure, computedCapturePointVelocity);
         CapturePointTools.computeDesiredCapturePointAcceleration(omega0, time, initialCapturePointPosition, initialCenterOfPressure,
               desiredCapturePointAcceleration);
         computedCapturePointVelocity.scale(omega0);

         JUnitTools.assertPoint3dEquals("", computedCapturePointVelocity.getPoint3dCopy(), desiredCapturePointAcceleration.getPoint3dCopy(), 1e-10);

         computedCapturePointVelocity.scale(1 / omega0);
         CapturePointTools.computeDesiredCapturePointAcceleration(omega0, computedCapturePointVelocity, desiredCapturePointAcceleration);
         computedCapturePointVelocity.scale(omega0);

         JUnitTools.assertPoint3dEquals("", computedCapturePointVelocity.getPoint3dCopy(), desiredCapturePointAcceleration.getPoint3dCopy(), 1e-10);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testComputeConstantCenterOfPressureFromInitialAndFinalCapturePointLocations()
   {
      YoFramePoint constantCenterOfPressure = new YoFramePoint("COP", ReferenceFrame.getWorldFrame(), registry);
      YoFramePoint finalDesiredICP = new YoFramePoint("finalICP", ReferenceFrame.getWorldFrame(), registry);
      YoFramePoint initialICP = new YoFramePoint("initialICP", ReferenceFrame.getWorldFrame(), registry);
      FramePoint2d p1 = new FramePoint2d();
      FramePoint2d p2 = new FramePoint2d();

      for (int j = 0; j < nTests; j++)
      {
         initialICP.set(new Point3d(random.nextDouble(), random.nextDouble(), 0));
         finalDesiredICP.set(new Point3d(random.nextDouble(), random.nextDouble(), 0));

         double omega0 = 3.4;
         double time = 0.5;

         CapturePointTools.computeConstantCMPFromInitialAndFinalCapturePointLocations(constantCenterOfPressure, finalDesiredICP, initialICP, omega0, time);

         p1.set(initialICP.getFramePoint2dCopy());
         p2.set(finalDesiredICP.getFramePoint2dCopy());
         Line2d line = new Line2d(p1.getPointCopy(), p2.getPointCopy());
         p1.set(constantCenterOfPressure.getFramePoint2dCopy());
         boolean isPointOnLine = line.isPointOnLine(p1.getPoint());
         assertTrue(isPointOnLine);
      }
   }
}
