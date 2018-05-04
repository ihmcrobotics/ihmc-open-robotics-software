package us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.ICPGeneration;

import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.Random;

import org.junit.Test;

import us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.lists.FrameTupleArrayList;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class CapturePointToolsTest
{
   int nTests = 20;
   Random random = new Random();
   YoVariableRegistry registry = new YoVariableRegistry("");
   
   private static final double EPSILON = 10e-6;

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testComputeConstantCMPPointsWithBeginningAndEndBetweenFeetWith2Steps()
   {
      int nFootsteps = 2;
      FrameTupleArrayList<FramePoint3D> footstepList = FrameTupleArrayList.createFramePointArrayList(nFootsteps);
      ArrayList<YoFramePoint3D> arrayToPack = new ArrayList<YoFramePoint3D>();
      PoseReferenceFrame poseReferenceFrame = new PoseReferenceFrame("test", new FramePose3D());

      int numberFootstepsToConsider = 4;
      for (int i = 0; i < numberFootstepsToConsider; i++)
      {
         arrayToPack.add(new YoFramePoint3D("test" + Integer.toString(i), ReferenceFrame.getWorldFrame(), registry));
      }
      for (int i = 0; i < nFootsteps; i++)
      {
         FramePose3D pose = new FramePose3D(ReferenceFrame.getWorldFrame(), new Point3D(random.nextDouble(), random.nextDouble(), random.nextDouble()),
               new Quaternion(random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble()));

         poseReferenceFrame.setPoseAndUpdate(pose);

         Point3D pointToPack = new Point3D();
         Quaternion orientation = new Quaternion();
         poseReferenceFrame.getPosition(pointToPack);
         poseReferenceFrame.getOrientation(orientation);
         FramePoint3D footstepData = footstepList.getAndGrowIfNeeded(i);
         footstepData.setIncludingFrame(poseReferenceFrame.getRootFrame(), pointToPack);
      }

      boolean startStanding = true;
      boolean endStanding = true;
      int lastFootstepIndex = Math.min(footstepList.size(), numberFootstepsToConsider) - 1;
      CapturePointTools.computeConstantCMPs(arrayToPack, footstepList, 0, lastFootstepIndex, startStanding, endStanding);

      Point3D pointBetweenFeet = new Point3D();
      pointBetweenFeet.set(new Point3D(footstepList.get(0)));
      pointBetweenFeet.add(new Point3D(footstepList.get(1)));
      pointBetweenFeet.scale(0.5);

      for (int i = 0; i < footstepList.size(); i++)
      {
         EuclidCoreTestTools.assertTuple3DEquals("", arrayToPack.get(i), pointBetweenFeet, 1e-10);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testComputeConstantCMPPointsOnFeetAndEndBetweenFeetWith2Steps()
   {
      int nFootsteps = 2;
      FrameTupleArrayList<FramePoint3D> footstepList = FrameTupleArrayList.createFramePointArrayList(nFootsteps);
      ArrayList<YoFramePoint3D> arrayToPack = new ArrayList<YoFramePoint3D>();
      PoseReferenceFrame poseReferenceFrame = new PoseReferenceFrame("test", new FramePose3D());

      int numberFootstepsToConsider = 4;
      for (int i = 0; i < numberFootstepsToConsider; i++)
      {
         arrayToPack.add(new YoFramePoint3D("test" + Integer.toString(i), ReferenceFrame.getWorldFrame(), registry));
      }
      for (int i = 0; i < nFootsteps; i++)
      {
         FramePose3D pose = new FramePose3D(ReferenceFrame.getWorldFrame(), new Point3D(random.nextDouble(), random.nextDouble(), random.nextDouble()),
               new Quaternion(random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble()));

         poseReferenceFrame.setPoseAndUpdate(pose);

         Point3D pointToPack = new Point3D();
         Quaternion orientation = new Quaternion();
         poseReferenceFrame.getPosition(pointToPack);
         poseReferenceFrame.getOrientation(orientation);
         FramePoint3D footstepData = footstepList.getAndGrowIfNeeded(i);
         footstepData.setIncludingFrame(poseReferenceFrame.getRootFrame(), pointToPack);
      }

      boolean startStanding = false;
      boolean endStanding = true;
      int lastFootstepIndex = Math.min(footstepList.size(), numberFootstepsToConsider) - 1;
      CapturePointTools.computeConstantCMPs(arrayToPack, footstepList, 0, lastFootstepIndex, startStanding, endStanding);

      Point3D pointBetweenFeet = new Point3D();
      pointBetweenFeet.set(new Point3D(footstepList.get(0)));
      pointBetweenFeet.add(new Point3D(footstepList.get(1)));
      pointBetweenFeet.scale(0.5);

      EuclidCoreTestTools.assertTuple3DEquals("", new Point3D(footstepList.get(0)), arrayToPack.get(0), 1e-10);

      for (int i = 1; i < footstepList.size(); i++)
      {
         EuclidCoreTestTools.assertTuple3DEquals("", arrayToPack.get(i), pointBetweenFeet, 1e-10);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testComputeConstantCentersOfPressuresOnFeet()
   {
      int nFootsteps = 10;
      FrameTupleArrayList<FramePoint3D> footstepList = FrameTupleArrayList.createFramePointArrayList(nFootsteps);
      ArrayList<YoFramePoint3D> arrayToPack = new ArrayList<YoFramePoint3D>();
      PoseReferenceFrame poseReferenceFrame = new PoseReferenceFrame("test", new FramePose3D());

      int numberFootstepsToConsider = 3;
      for (int i = 0; i < numberFootstepsToConsider; i++)
      {
         arrayToPack.add(new YoFramePoint3D("test" + Integer.toString(i), ReferenceFrame.getWorldFrame(), registry));
      }

      for (int j = 0; j < nTests; j++)
      {
         for (int i = 0; i < nFootsteps; i++)
         {
            FramePose3D pose = new FramePose3D(ReferenceFrame.getWorldFrame(), new Point3D(random.nextDouble(), random.nextDouble(), random.nextDouble()),
                  new Quaternion(random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble()));

            poseReferenceFrame.setPoseAndUpdate(pose);

            Point3D pointToPack = new Point3D();
            Quaternion orientation = new Quaternion();
            poseReferenceFrame.getPosition(pointToPack);
            poseReferenceFrame.getOrientation(orientation);
            FramePoint3D footstepData = footstepList.getAndGrowIfNeeded(i);
            footstepData.setIncludingFrame(poseReferenceFrame.getRootFrame(), pointToPack);
         }

         CapturePointTools.computeConstantCMPsOnFeet(arrayToPack, footstepList, 0, numberFootstepsToConsider - 1);

         for (int i = 0; i < arrayToPack.size(); i++)
         {
            EuclidCoreTestTools.assertTuple3DEquals("", arrayToPack.get(i), new Point3D(footstepList.get(i)), 1e-10);
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testComputeConstantCentersOfPressureWithStartBetweenFeetAndRestOnFeet()
   {
      int nFootsteps = 10;
      FrameTupleArrayList<FramePoint3D> footstepList = FrameTupleArrayList.createFramePointArrayList(nFootsteps);
      ArrayList<YoFramePoint3D> arrayToPack = new ArrayList<YoFramePoint3D>();
      PoseReferenceFrame poseReferenceFrame = new PoseReferenceFrame("test", new FramePose3D());

      int numberFootstepsToConsider = 3;
      for (int i = 0; i < numberFootstepsToConsider; i++)
      {
         arrayToPack.add(new YoFramePoint3D("test" + Integer.toString(i), ReferenceFrame.getWorldFrame(), registry));
      }

      for (int j = 0; j < nTests; j++)
      {
         for (int i = 0; i < nFootsteps; i++)
         {
            FramePose3D pose = new FramePose3D(ReferenceFrame.getWorldFrame(), new Point3D(random.nextDouble(), random.nextDouble(), random.nextDouble()),
                  new Quaternion(random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble()));

            poseReferenceFrame.setPoseAndUpdate(pose);

            Point3D pointToPack = new Point3D();
            Quaternion orientation = new Quaternion();
            poseReferenceFrame.getPosition(pointToPack);
            poseReferenceFrame.getOrientation(orientation);
            FramePoint3D footstepData = footstepList.getAndGrowIfNeeded(i);
            footstepData.setIncludingFrame(poseReferenceFrame.getRootFrame(), pointToPack);
         }

         boolean startStanding = true;
         boolean endStanding = false;
         int lastFootstepIndex = Math.min(footstepList.size(), numberFootstepsToConsider) - 1;
         CapturePointTools.computeConstantCMPs(arrayToPack, footstepList, 0, lastFootstepIndex, startStanding, endStanding);

         Point3D copPos1 = new Point3D();
         copPos1.set(new Point3D(footstepList.get(0)));
         copPos1.add(new Point3D(footstepList.get(1)));
         copPos1.scale(0.5);
         EuclidCoreTestTools.assertTuple3DEquals("", copPos1, arrayToPack.get(0), 1e-10);
         for (int i = 1; i < arrayToPack.size() - 1; i++)
         {
            EuclidCoreTestTools.assertTuple3DEquals("", arrayToPack.get(i), new Point3D(footstepList.get(i)), 1e-10);
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testComputeConstantCentersOfPressureWithEndBetweenFeetAndRestOnFeet()
   {
      int nFootsteps = 10;
      FrameTupleArrayList<FramePoint3D> footstepList = FrameTupleArrayList.createFramePointArrayList(nFootsteps);
      ArrayList<YoFramePoint3D> arrayToPack = new ArrayList<YoFramePoint3D>();
      PoseReferenceFrame poseReferenceFrame = new PoseReferenceFrame("test", new FramePose3D());

      int numberFootstepsToConsider = 3;
      for (int i = 0; i < numberFootstepsToConsider; i++)
      {
         arrayToPack.add(new YoFramePoint3D("test" + Integer.toString(i), ReferenceFrame.getWorldFrame(), registry));
      }

      for (int j = 0; j < nTests; j++)
      {
         for (int i = 0; i < nFootsteps; i++)
         {
            FramePose3D pose = new FramePose3D(ReferenceFrame.getWorldFrame(), new Point3D(random.nextDouble(), random.nextDouble(), random.nextDouble()),
                  new Quaternion(random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble()));

            poseReferenceFrame.setPoseAndUpdate(pose);

            Point3D pointToPack = new Point3D();
            Quaternion orientation = new Quaternion();
            poseReferenceFrame.getPosition(pointToPack);
            poseReferenceFrame.getOrientation(orientation);
            FramePoint3D footstepData = footstepList.getAndGrowIfNeeded(i);
            footstepData.setIncludingFrame(poseReferenceFrame.getRootFrame(), pointToPack);
         }

         boolean startStanding = false;
         boolean endStanding = true;
         int lastFootstepIndex = Math.min(footstepList.size(), numberFootstepsToConsider) - 1;
         CapturePointTools.computeConstantCMPs(arrayToPack, footstepList, 0, lastFootstepIndex, startStanding, endStanding);

         Point3D copPos1 = new Point3D();
         copPos1.set(new Point3D(footstepList.get(numberFootstepsToConsider - 1)));
         copPos1.add(new Point3D(footstepList.get(numberFootstepsToConsider - 2)));
         copPos1.scale(0.5);
         EuclidCoreTestTools.assertTuple3DEquals("", copPos1, arrayToPack.get(numberFootstepsToConsider - 1), 1e-10);
         for (int i = 0; i < arrayToPack.size() - 2; i++)
         {
            EuclidCoreTestTools.assertTuple3DEquals("", arrayToPack.get(i), new Point3D(footstepList.get(i)), 1e-10);
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testComputeConstantCentersOfPressureWithEndAndBeginningBetweenFeet()
   {
      int nFootsteps = 10;
      FrameTupleArrayList<FramePoint3D> footstepList = FrameTupleArrayList.createFramePointArrayList(nFootsteps);
      ArrayList<YoFramePoint3D> arrayToPack = new ArrayList<YoFramePoint3D>();
      PoseReferenceFrame poseReferenceFrame = new PoseReferenceFrame("test", new FramePose3D());

      int numberFootstepsToConsider = 3;
      for (int i = 0; i < numberFootstepsToConsider; i++)
      {
         arrayToPack.add(new YoFramePoint3D("test" + Integer.toString(i), ReferenceFrame.getWorldFrame(), registry));
      }

      for (int j = 0; j < nTests; j++)
      {
         for (int i = 0; i < nFootsteps; i++)
         {
            FramePose3D pose = new FramePose3D(ReferenceFrame.getWorldFrame(), new Point3D(random.nextDouble(), random.nextDouble(), random.nextDouble()),
                  new Quaternion(random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble()));

            poseReferenceFrame.setPoseAndUpdate(pose);

            Point3D pointToPack = new Point3D();
            Quaternion orientation = new Quaternion();
            poseReferenceFrame.getPosition(pointToPack);
            poseReferenceFrame.getOrientation(orientation);
            FramePoint3D footstepData = footstepList.getAndGrowIfNeeded(i);
            footstepData.setIncludingFrame(poseReferenceFrame.getRootFrame(), pointToPack);
         }

         boolean startStanding = true;
         boolean endStanding = true;
         int lastFootstepIndex = Math.min(footstepList.size(), numberFootstepsToConsider) - 1;
         CapturePointTools.computeConstantCMPs(arrayToPack, footstepList, 0, lastFootstepIndex, startStanding, endStanding);

         Point3D copPos1 = new Point3D();
         copPos1.set(new Point3D(footstepList.get(numberFootstepsToConsider - 1)));
         copPos1.add(new Point3D(footstepList.get(numberFootstepsToConsider - 2)));
         copPos1.scale(0.5);
         EuclidCoreTestTools.assertTuple3DEquals("", copPos1, arrayToPack.get(numberFootstepsToConsider - 1), 1e-10);

         copPos1.set(new Point3D(footstepList.get(0)));
         copPos1.add(new Point3D(footstepList.get(1)));
         copPos1.scale(0.5);
         EuclidCoreTestTools.assertTuple3DEquals("", copPos1, arrayToPack.get(0), 1e-10);

         for (int i = 1; i < arrayToPack.size() - 2; i++)
         {
            EuclidCoreTestTools.assertTuple3DEquals("", arrayToPack.get(i), new Point3D(footstepList.get(i)), 1e-10);
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testComputeDesiredEndOfStepCapturePointLocations()
   {
      int nFootsteps = 10;
      FrameTupleArrayList<FramePoint3D> footstepList = FrameTupleArrayList.createFramePointArrayList(nFootsteps);
      ArrayList<YoFramePoint3D> constantCentersOfPressures = new ArrayList<YoFramePoint3D>();
      ArrayList<YoFramePoint3D> capturePointsToPack = new ArrayList<YoFramePoint3D>();
      YoFramePoint3D icpToCheck = new YoFramePoint3D("icpToCheck", ReferenceFrame.getWorldFrame(), registry);
      PoseReferenceFrame poseReferenceFrame = new PoseReferenceFrame("test", new FramePose3D());
      FramePoint2D p1 = new FramePoint2D();
      FramePoint2D p2 = new FramePoint2D();

      int numberFootstepsToConsider = random.nextInt(((nFootsteps - 3) + 1)) + 3;
      for (int i = 0; i < numberFootstepsToConsider; i++)
      {
         constantCentersOfPressures.add(new YoFramePoint3D("test" + Integer.toString(i), ReferenceFrame.getWorldFrame(), registry));
      }
      int lastFootstepIndex = numberFootstepsToConsider - 1;
      for (int i = 0; i < lastFootstepIndex; i++)
      {
         capturePointsToPack.add(new YoFramePoint3D("testICP" + Integer.toString(i), ReferenceFrame.getWorldFrame(), registry));
      }

      for (int j = 0; j < nTests; j++)
      {
         for (int i = 0; i < numberFootstepsToConsider; i++)
         {
            FramePose3D pose = new FramePose3D(ReferenceFrame.getWorldFrame(), new Point3D(random.nextDouble(), random.nextDouble(), random.nextDouble()),
                  new Quaternion(random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble()));

            poseReferenceFrame.setPoseAndUpdate(pose);

            Point3D pointToPack = new Point3D();
            Quaternion orientation = new Quaternion();
            poseReferenceFrame.getPosition(pointToPack);
            poseReferenceFrame.getOrientation(orientation);
            FramePoint3D footstepData = footstepList.getAndGrowIfNeeded(i);
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

         
         EuclidFrameTestTools.assertFrameTuple3DEquals(capturePointsToPack.get(0), icpToCheck, 1e-8);

         for (int i = 0; i < constantCentersOfPressures.size() - 2; i++)
         {
            p1.set(constantCentersOfPressures.get(i));
            p2.set(capturePointsToPack.get(i));
            Line2D line = new Line2D(p1, p2);
            p1.set(capturePointsToPack.get(i + 1));
            boolean isPointOnLine = line.isPointOnLine(p1);
            assertTrue(isPointOnLine);
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testComputeDesiredCapturePointLocations()
   {
      int nFootsteps = 10;
      FrameTupleArrayList<FramePoint3D> footstepList = FrameTupleArrayList.createFramePointArrayList(nFootsteps);
      ArrayList<YoFramePoint3D> constantCentersOfPressures = new ArrayList<YoFramePoint3D>();
      ArrayList<YoFramePoint3D> capturePointsToPack = new ArrayList<YoFramePoint3D>();
      PoseReferenceFrame poseReferenceFrame = new PoseReferenceFrame("test", new FramePose3D());
      FramePoint2D p1 = new FramePoint2D();
      FramePoint2D p2 = new FramePoint2D();
      YoFramePoint3D desiredICP = new YoFramePoint3D("", ReferenceFrame.getWorldFrame(), registry);

      int numberFootstepsToConsider = random.nextInt(((nFootsteps - 3) + 1)) + 3;
      for (int i = 0; i < numberFootstepsToConsider; i++)
      {
         constantCentersOfPressures.add(new YoFramePoint3D("test" + Integer.toString(i), ReferenceFrame.getWorldFrame(), registry));
      }
      for (int i = 0; i < numberFootstepsToConsider - 1; i++)
      {
         capturePointsToPack.add(new YoFramePoint3D("testICP" + Integer.toString(i), ReferenceFrame.getWorldFrame(), registry));
      }

      for (int j = 0; j < nTests; j++)
      {
         for (int i = 0; i < nFootsteps; i++)
         {
            FramePose3D pose = new FramePose3D(ReferenceFrame.getWorldFrame(), new Point3D(random.nextDouble(), random.nextDouble(), random.nextDouble()),
                  new Quaternion(random.nextDouble(), random.nextDouble(), random.nextDouble(), random.nextDouble()));

            poseReferenceFrame.setPoseAndUpdate(pose);

            Point3D pointToPack = new Point3D();
            Quaternion orientation = new Quaternion();
            poseReferenceFrame.getPosition(pointToPack);
            poseReferenceFrame.getOrientation(orientation);
            FramePoint3D footstepData = footstepList.getAndGrowIfNeeded(i);
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

            p1.set(constantCentersOfPressures.get(i));
            p2.set(capturePointsToPack.get(i));
            Line2D line = new Line2D(p1, p2);
            p1.set(desiredICP);
            boolean isPointOnLine = line.isPointOnLine(p1);
            assertTrue(isPointOnLine);
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testComputeDesiredCapturePointVelocity()
   {
      YoFramePoint3D initialCapturePointPosition = new YoFramePoint3D("", ReferenceFrame.getWorldFrame(), registry);
      YoFramePoint3D computedCapturePoint1 = new YoFramePoint3D("1", ReferenceFrame.getWorldFrame(), registry);
      YoFramePoint3D computedCapturePoint2 = new YoFramePoint3D("2", ReferenceFrame.getWorldFrame(), registry);
      YoFramePoint3D initialCenterOfPressure = new YoFramePoint3D("3", ReferenceFrame.getWorldFrame(), registry);
      YoFrameVector3D differentiatedCapturePointPosition = new YoFrameVector3D("4", ReferenceFrame.getWorldFrame(), registry);
      YoFrameVector3D computedCapturePointVelocity = new YoFrameVector3D("5", ReferenceFrame.getWorldFrame(), registry);

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

         EuclidCoreTestTools.assertTuple3DEquals("", computedCapturePointVelocity, differentiatedCapturePointPosition, 1e-3);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testComputeDesiredCapturePointAcceleration()
   {
      YoFramePoint3D initialCapturePointPosition = new YoFramePoint3D("", ReferenceFrame.getWorldFrame(), registry);
      YoFramePoint3D initialCenterOfPressure = new YoFramePoint3D("3", ReferenceFrame.getWorldFrame(), registry);
      YoFrameVector3D computedCapturePointVelocity = new YoFrameVector3D("5", ReferenceFrame.getWorldFrame(), registry);
      YoFrameVector3D desiredCapturePointAcceleration = new YoFrameVector3D("6", ReferenceFrame.getWorldFrame(), registry);

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

         EuclidCoreTestTools.assertTuple3DEquals("", computedCapturePointVelocity, desiredCapturePointAcceleration, 1e-10);

         computedCapturePointVelocity.scale(1 / omega0);
         CapturePointTools.computeDesiredCapturePointAcceleration(omega0, computedCapturePointVelocity, desiredCapturePointAcceleration);
         computedCapturePointVelocity.scale(omega0);

         EuclidCoreTestTools.assertTuple3DEquals("", computedCapturePointVelocity, desiredCapturePointAcceleration, 1e-10);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testComputeConstantCenterOfPressureFromInitialAndFinalCapturePointLocations()
   {
      YoFramePoint3D constantCenterOfPressure = new YoFramePoint3D("COP", ReferenceFrame.getWorldFrame(), registry);
      YoFramePoint3D finalDesiredICP = new YoFramePoint3D("finalICP", ReferenceFrame.getWorldFrame(), registry);
      YoFramePoint3D initialICP = new YoFramePoint3D("initialICP", ReferenceFrame.getWorldFrame(), registry);
      FramePoint2D p1 = new FramePoint2D();
      FramePoint2D p2 = new FramePoint2D();

      for (int j = 0; j < nTests; j++)
      {
         initialICP.set(new Point3D(random.nextDouble(), random.nextDouble(), 0));
         finalDesiredICP.set(new Point3D(random.nextDouble(), random.nextDouble(), 0));

         double omega0 = 3.4;
         double time = 0.5;

         CapturePointTools.computeConstantCMPFromInitialAndFinalCapturePointLocations(constantCenterOfPressure, finalDesiredICP, initialICP, omega0, time);

         p1.set(initialICP);
         p2.set(finalDesiredICP);
         Line2D line = new Line2D(p1, p2);
         p1.set(constantCenterOfPressure);
         boolean isPointOnLine = line.isPointOnLine(p1);
         assertTrue(isPointOnLine);
      }
   }
}
