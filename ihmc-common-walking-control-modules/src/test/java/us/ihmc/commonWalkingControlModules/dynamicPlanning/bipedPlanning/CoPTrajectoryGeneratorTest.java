package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepShiftFractions;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;

public class CoPTrajectoryGeneratorTest
{
   @Test
   public void testSingleStep()
   {
      double stanceWidth = 0.4;
      YoRegistry registry = new YoRegistry("test");
      SideDependentList<PoseReferenceFrame> soleFrames = CoPTrajectoryGeneratorTestTools.createSoleFrames();
      SideDependentList<FrameConvexPolygon2D> polygons = new SideDependentList<>();

      CoPTrajectoryGenerator copTrajectory = new CoPTrajectoryGenerator(new CoPTrajectoryParameters(),
                                                                        CoPTrajectoryGeneratorTestTools.createDefaultSupportPolygon(),
                                                                        registry);
      CoPTrajectoryGeneratorState state = new CoPTrajectoryGeneratorState(soleFrames, registry);
      state.setInitialCoP(new FramePoint2D());

      for (RobotSide robotSide : RobotSide.values)
      {
         soleFrames.get(robotSide).setPositionWithoutChecksAndUpdate(0.0,  robotSide.negateIfRightSide(0.5) * stanceWidth, 0.0);
         polygons.put(robotSide, new FrameConvexPolygon2D(soleFrames.get(robotSide), CoPTrajectoryGeneratorTestTools.createDefaultSupportPolygon()));
      }

      Footstep footstep = new Footstep();
      footstep.getFootstepPose().getPosition().set(0.5, -0.5 * stanceWidth, 0.0);
      footstep.setRobotSide(RobotSide.RIGHT);

      FootstepTiming timing = new FootstepTiming();
      timing.setTimings(1.0, 0.5);

      FootstepShiftFractions shiftFractions = new FootstepShiftFractions();
      shiftFractions.setShiftFractions(0.9, 0.5, 0.5);
      shiftFractions.setTransferWeightDistribution(0.5);

      state.setFinalTransferDuration(1.0);
      state.setFinalTransferSplitFraction(0.5);
      state.setFinalTransferWeightDistribution(0.5);
      state.addFootstep(footstep);
      state.addFootstepTiming(timing);
      state.addFootstepShiftFractions(shiftFractions);
      state.initializeStance(polygons, soleFrames);

      copTrajectory.compute(state);

      CoPTrajectoryVisualizer.visualize(copTrajectory);
   }

}
