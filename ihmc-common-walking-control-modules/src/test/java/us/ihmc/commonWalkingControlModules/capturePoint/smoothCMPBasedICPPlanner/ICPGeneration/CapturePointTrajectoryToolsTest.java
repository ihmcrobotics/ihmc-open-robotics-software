package us.ihmc.commonWalkingControlModules.capturePoint.smoothCMPBasedICPPlanner.ICPGeneration;

import static us.ihmc.robotics.Assert.assertTrue;

import java.util.ArrayList;
import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

import us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools;
import us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTrajectoryTools;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.lists.FrameTupleArrayList;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class CapturePointTrajectoryToolsTest
{
   int nTests = 20;
   Random random = new Random();
   YoRegistry registry = new YoRegistry("Dummy");

   private static final double EPSILON = 10e-6;

   @AfterEach
   public void tearDown()
   {
      ReferenceFrameTools.clearWorldFrameTree();
   }

   @Test
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
      CapturePointTrajectoryTools.computeConstantCMPs(arrayToPack, footstepList, 0, lastFootstepIndex, startStanding, endStanding);

      Point3D pointBetweenFeet = new Point3D();
      pointBetweenFeet.set(new Point3D(footstepList.get(0)));
      pointBetweenFeet.add(new Point3D(footstepList.get(1)));
      pointBetweenFeet.scale(0.5);

      for (int i = 0; i < footstepList.size(); i++)
      {
         EuclidCoreTestTools.assertTuple3DEquals("", arrayToPack.get(i), pointBetweenFeet, 1e-10);
      }
   }

   @Test
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
      CapturePointTrajectoryTools.computeConstantCMPs(arrayToPack, footstepList, 0, lastFootstepIndex, startStanding, endStanding);

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

   @Test
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
         CapturePointTrajectoryTools.computeConstantCMPs(arrayToPack, footstepList, 0, lastFootstepIndex, startStanding, endStanding);

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

   @Test
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
         CapturePointTrajectoryTools.computeConstantCMPs(arrayToPack, footstepList, 0, lastFootstepIndex, startStanding, endStanding);

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

   @Test
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
         CapturePointTrajectoryTools.computeConstantCMPs(arrayToPack, footstepList, 0, lastFootstepIndex, startStanding, endStanding);

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


   @Test
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

         CapturePointTrajectoryTools.computeConstantCMPsOnFeet(arrayToPack, footstepList, 0, numberFootstepsToConsider - 1);

         for (int i = 0; i < arrayToPack.size(); i++)
         {
            EuclidCoreTestTools.assertTuple3DEquals("", arrayToPack.get(i), new Point3D(footstepList.get(i)), 1e-10);
         }
      }
   }

   @Test
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

         CapturePointTrajectoryTools.putConstantCMPBetweenFeet(constantCentersOfPressures.get(0), footstepList.get(0), footstepList.get(1));
         CapturePointTrajectoryTools.computeConstantCMPsOnFeet(constantCentersOfPressures, footstepList, 1, lastFootstepIndex);
         CapturePointTrajectoryTools.putConstantCMPBetweenFeet(constantCentersOfPressures.get(lastFootstepIndex), footstepList.get(lastFootstepIndex - 1),
                                                               footstepList.get(lastFootstepIndex));
         double omega0 = 0.5;
         double time = 0.5;
         CapturePointTrajectoryTools.computeDesiredCornerPoints(capturePointsToPack, constantCentersOfPressures, false, time, omega0);

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

   @Test
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
         CapturePointTrajectoryTools.putConstantCMPBetweenFeet(constantCentersOfPressures.get(0), footstepList.get(0), footstepList.get(1));
         CapturePointTrajectoryTools.computeConstantCMPsOnFeet(constantCentersOfPressures, footstepList, 1, lastFootstepIndex);
         CapturePointTrajectoryTools.putConstantCMPBetweenFeet(constantCentersOfPressures.get(lastFootstepIndex), footstepList.get(lastFootstepIndex - 1),
                                                               footstepList.get(lastFootstepIndex));
         double omega0 = 0.5;
         double time = 0.5;
         CapturePointTrajectoryTools.computeDesiredCornerPoints(capturePointsToPack, constantCentersOfPressures, false, time, omega0);

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


   @Test
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

         CapturePointTrajectoryTools
               .computeConstantCMPFromInitialAndFinalCapturePointLocations(constantCenterOfPressure, finalDesiredICP, initialICP, omega0, time);

         p1.set(initialICP);
         p2.set(finalDesiredICP);
         Line2D line = new Line2D(p1, p2);
         p1.set(constantCenterOfPressure);
         boolean isPointOnLine = line.isPointOnLine(p1);
         assertTrue(isPointOnLine);
      }
   }
}