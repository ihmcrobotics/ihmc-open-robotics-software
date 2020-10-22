package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTestTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepShiftFractions;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.saveableModule.YoSaveableModuleStateTools;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.Random;

public class CoPTrajectoryGeneratorStateTest
{
   private static final double epsilon = 1e-5;

   @Test
   public void testConstruction()
   {
      CoPTrajectoryGeneratorState state = new CoPTrajectoryGeneratorState(new YoRegistry("test"));
   }

   @Test
   public void testSaveAndLoad()
   {
      SideDependentList<PoseReferenceFrame> soleFrames = CoPTrajectoryGeneratorTestTools.createSoleFrames();

      CoPTrajectoryGeneratorState stateA = new CoPTrajectoryGeneratorState(new YoRegistry("test"));
      CoPTrajectoryGeneratorState stateB = new CoPTrajectoryGeneratorState(new YoRegistry("test"));

      Random random = new Random(1738L);
      for (int j = 0; j < 50; j++)
      {
         stateA.clear();

         FramePoint2DReadOnly randomInitialCoP = EuclidFrameRandomTools.nextFramePoint2D(random, ReferenceFrame.getWorldFrame(), 10.0);
         FootstepTiming randomTiming0 = CoPTrajectoryGeneratorTestTools.getRandomTiming(random);
         FootstepTiming randomTiming1 = CoPTrajectoryGeneratorTestTools.getRandomTiming(random);

         Footstep randomFootstep0 = CoPTrajectoryGeneratorTestTools.getRandomFootstep(random);
         Footstep randomFootstep1 = CoPTrajectoryGeneratorTestTools.getRandomFootstep(random);

         FramePose3DReadOnly randomLeftFootPose = EuclidFrameRandomTools.nextFramePose3D(random, ReferenceFrame.getWorldFrame(), 10.0, 5.0);
         FramePose3DReadOnly randomRightFootPose = EuclidFrameRandomTools.nextFramePose3D(random, ReferenceFrame.getWorldFrame(), 10.0, 5.0);

         soleFrames.get(RobotSide.LEFT).setPoseAndUpdate(randomLeftFootPose);
         soleFrames.get(RobotSide.RIGHT).setPoseAndUpdate(randomRightFootPose);

         FrameConvexPolygon2DReadOnly randomLeftFootPolygon = EuclidFrameRandomTools.nextFrameConvexPolygon2D(random, soleFrames.get(RobotSide.LEFT), 1.0, 6);
         FrameConvexPolygon2DReadOnly randomRightFootPolygon = EuclidFrameRandomTools.nextFrameConvexPolygon2D(random, soleFrames.get(RobotSide.RIGHT), 1.0, 6);

         stateA.setInitialCoP(randomInitialCoP);
         stateA.addFootstepTiming(randomTiming0);
         stateA.addFootstepTiming(randomTiming1);
         stateA.addFootstep(randomFootstep0);
         stateA.addFootstep(randomFootstep1);

         stateA.initializeStance(RobotSide.LEFT, randomLeftFootPolygon, soleFrames.get(RobotSide.LEFT));
         stateA.initializeStance(RobotSide.RIGHT, randomRightFootPolygon, soleFrames.get(RobotSide.RIGHT));

         stateB.loadValues(YoSaveableModuleStateTools.readSaveableRegistryToDataMap(YoSaveableModuleStateTools.writeStateToSaveableRegistry(stateA)));

         EuclidFrameTestTools.assertFramePoint2DGeometricallyEquals(randomInitialCoP, stateB.getInitialCoP(), epsilon);
         CoPTrajectoryGeneratorTestTools.assertTimingsEqual(randomTiming0, stateB.getTiming(0), epsilon);
         CoPTrajectoryGeneratorTestTools.assertTimingsEqual(randomTiming1, stateB.getTiming(1), epsilon);
         CoPTrajectoryGeneratorTestTools.assertFootstepEqual(randomFootstep0, stateB.getFootstep(0), epsilon);
         CoPTrajectoryGeneratorTestTools.assertFootstepEqual(randomFootstep1, stateB.getFootstep(1), epsilon);

         EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(randomLeftFootPose.getPosition(),
                                                                    stateB.getFootPose(RobotSide.LEFT).getPosition(),
                                                                    epsilon);
         EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(randomRightFootPose.getPosition(),
                                                                    stateB.getFootPose(RobotSide.RIGHT).getPosition(),
                                                                    epsilon);
         EuclidFrameTestTools.assertFrameQuaternionGeometricallyEquals(randomLeftFootPose.getOrientation(),
                                                                       stateB.getFootPose(RobotSide.LEFT).getOrientation(),
                                                                       epsilon);
         EuclidFrameTestTools.assertFrameQuaternionGeometricallyEquals(randomRightFootPose.getOrientation(),
                                                                       stateB.getFootPose(RobotSide.RIGHT).getOrientation(),
                                                                       epsilon);
         EuclidGeometryTestTools.assertConvexPolygon2DGeometricallyEquals(randomLeftFootPolygon, stateB.getFootPolygonInSole(RobotSide.LEFT), epsilon);
         EuclidGeometryTestTools.assertConvexPolygon2DGeometricallyEquals(randomRightFootPolygon, stateB.getFootPolygonInSole(RobotSide.RIGHT), epsilon);
         EuclidCoreTestTools.assertRigidBodyTransformEquals(randomLeftFootPolygon.getReferenceFrame().getTransformToWorldFrame(),
                                                            stateB.getFootPolygonInSole(RobotSide.LEFT).getReferenceFrame().getTransformToWorldFrame(),
                                                            epsilon);
         EuclidCoreTestTools.assertRigidBodyTransformEquals(randomRightFootPolygon.getReferenceFrame().getTransformToWorldFrame(),
                                                            stateB.getFootPolygonInSole(RobotSide.RIGHT).getReferenceFrame().getTransformToWorldFrame(),
                                                            epsilon);
      }
   }
}
