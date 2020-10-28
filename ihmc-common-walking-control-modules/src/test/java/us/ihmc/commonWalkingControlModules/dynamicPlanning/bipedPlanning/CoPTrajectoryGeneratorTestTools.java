package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepShiftFractions;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.Assert;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class CoPTrajectoryGeneratorTestTools
{
   static void assertFootstepEqual(Footstep footstepExpected, DynamicPlanningFootstep footstep, double epsilon)
   {
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(footstepExpected.getFootstepPose().getPosition(),
                                                                 footstep.getFootstepPose().getPosition(),
                                                                 epsilon);
      EuclidFrameTestTools.assertFrameQuaternionGeometricallyEquals(footstepExpected.getFootstepPose().getOrientation(),
                                                                    footstep.getFootstepPose().getOrientation(),
                                                                    epsilon);
      Assert.assertEquals(footstepExpected.getRobotSide(), footstep.getRobotSide());
      Assert.assertEquals(footstepExpected.hasPredictedContactPoints(), footstep.hasPredictedContactPoints());
      if (!footstepExpected.hasPredictedContactPoints())
         return;
      
      Assert.assertEquals(footstepExpected.getPredictedContactPoints().size(), footstep.getPredictedContactPoints().size());
      for (int i = 0; i < footstep.getPredictedContactPoints().size(); i++)
      {
         EuclidCoreTestTools.assertPoint2DGeometricallyEquals(footstepExpected.getPredictedContactPoints().get(i),
                                                              footstep.getPredictedContactPoints().get(i),
                                                              epsilon);
      }
   }

   static void assertFootstepEqual(DynamicPlanningFootstep footstepExpected, DynamicPlanningFootstep footstep, double epsilon)
   {
      EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(footstepExpected.getFootstepPose().getPosition(),
                                                                 footstep.getFootstepPose().getPosition(),
                                                                 epsilon);
      EuclidFrameTestTools.assertFrameQuaternionGeometricallyEquals(footstepExpected.getFootstepPose().getOrientation(),
                                                                    footstep.getFootstepPose().getOrientation(),
                                                                    epsilon);
      Assert.assertEquals(footstepExpected.getRobotSide(), footstep.getRobotSide());
      Assert.assertEquals(footstepExpected.getPredictedContactPoints().size(), footstep.getPredictedContactPoints().size());
      for (int i = 0; i < footstep.getPredictedContactPoints().size(); i++)
      {
         EuclidCoreTestTools.assertPoint2DGeometricallyEquals(footstepExpected.getPredictedContactPoints().get(i),
                                                              footstep.getPredictedContactPoints().get(i),
                                                              epsilon);
      }
   }

   static void assertShiftFractionsEqual(FootstepShiftFractions timingExpected, PlanningShiftFraction timing, double epsilon)
   {
      Assert.assertEquals(timingExpected.getSwingDurationShiftFraction(), timing.getSwingDurationShiftFraction(), epsilon);
      Assert.assertEquals(timingExpected.getSwingSplitFraction(), timing.getSwingSplitFraction(), epsilon);
      Assert.assertEquals(timingExpected.getTransferSplitFraction(), timing.getTransferSplitFraction(), epsilon);
      Assert.assertEquals(timingExpected.getTransferWeightDistribution(), timing.getTransferWeightDistribution(), epsilon);
   }

   static void assertTimingsEqual(FootstepTiming timingExpected, PlanningTiming timing, double epsilon)
   {
      Assert.assertEquals(timingExpected.getSwingTime(), timing.getSwingTime(), epsilon);
      Assert.assertEquals(timingExpected.getTransferTime(), timing.getTransferTime(), epsilon);
   }

   static FootstepShiftFractions getRandomShiftFractions(Random random)
   {
      FootstepShiftFractions shiftFractions = new FootstepShiftFractions();
      shiftFractions.setTransferWeightDistribution(RandomNumbers.nextDouble(random, 0.0, 1.0));
      shiftFractions.setShiftFractions(RandomNumbers.nextDouble(random, 0.0, 1.0),
                                       RandomNumbers.nextDouble(random, 0.0, 1.0),
                                       RandomNumbers.nextDouble(random, 0.0, 1.0));

      return shiftFractions;
   }

   static FootstepTiming getRandomTiming(Random random)
   {
      FootstepTiming timing = new FootstepTiming();
      timing.setTimings(RandomNumbers.nextDouble(random, 0.1, 10.0), RandomNumbers.nextDouble(random, 0.1, 10.0));

      return timing;
   }

   static Footstep getRandomFootstep(Random random)
   {
      Footstep footstep = new Footstep();
      footstep.getFootstepPose().set(EuclidFrameRandomTools.nextFramePose3D(random, ReferenceFrame.getWorldFrame(), 10.0, 5.0));
      footstep.setRobotSide(RobotSide.values[RandomNumbers.nextInt(random, 0, 1)]);
      List<Point2D> predictedContactPoints = new ArrayList<>();
      for (int i = 0; i < RandomNumbers.nextInt(random, 0, 5); i++)
         predictedContactPoints.add(EuclidCoreRandomTools.nextPoint2D(random));
      footstep.setPredictedContactPoints(predictedContactPoints);

      return footstep;
   }

   static DynamicPlanningFootstep getRandomPlanningFootstep(Random random)
   {
      DynamicPlanningFootstep footstep = new DynamicPlanningFootstep("", new YoRegistry("test"));
      footstep.set(getRandomFootstep(random));

      return footstep;
   }

   static SideDependentList<PoseReferenceFrame> createSoleFrames()
   {
      SideDependentList<PoseReferenceFrame> soleFrames = new SideDependentList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         PoseReferenceFrame soleFrame = new PoseReferenceFrame(robotSide.getLowerCaseName() + "SoleFrame", ReferenceFrame.getWorldFrame());
         soleFrame.setPositionWithoutChecksAndUpdate(0.0, robotSide.negateIfRightSide(0.1), 0.0);
         soleFrames.put(robotSide, soleFrame);
      }
      return soleFrames;
   }

   public static ConvexPolygon2D createDefaultSupportPolygon()
   {
      ConvexPolygon2D defaultSupportPolygon = new ConvexPolygon2D();
      defaultSupportPolygon.addVertex(0.1, 0.05);
      defaultSupportPolygon.addVertex(0.1, -0.05);
      defaultSupportPolygon.addVertex(-0.1, 0.05);
      defaultSupportPolygon.addVertex(-0.1, -0.05);
      defaultSupportPolygon.update();
      return defaultSupportPolygon;
   }
}
