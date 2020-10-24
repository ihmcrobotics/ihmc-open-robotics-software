package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateProvider;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.saveableModule.YoSaveableModuleStateTools;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.List;

import static us.ihmc.robotics.Assert.assertEquals;

public class WalkingCoPTrajectoryGeneratorTest
{
   private static boolean visualize = false;

   @BeforeEach
   public void setup()
   {
      visualize = visualize && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer();
   }

   private static final double epsilon = 1e-7;
   @Test
   public void testSingleStep()
   {
      double stanceWidth = 0.4;
      YoRegistry registry = new YoRegistry("test");
      SideDependentList<PoseReferenceFrame> soleFrames = CoPTrajectoryGeneratorTestTools.createSoleFrames();
      SideDependentList<FrameConvexPolygon2D> polygons = new SideDependentList<>();

      double swingShiftFraction = 0.9;
      double swingSplitFraction = 0.4;
      double transferSplitFraction = 0.5;
      CoPTrajectoryParameters parameters = new CoPTrajectoryParameters()
      {
         @Override
         public double getDefaultSwingDurationShiftFraction()
         {
            return swingShiftFraction;
         }

         @Override
         public double getDefaultSwingSplitFraction()
         {
            return swingSplitFraction;
         }

         @Override
         public double getDefaultTransferSplitFraction()
         {
            return transferSplitFraction;
         }
      };

      registry.addChild(parameters.getRegistry());
      WalkingCoPTrajectoryGenerator copTrajectory = new WalkingCoPTrajectoryGenerator(parameters,
                                                                                      CoPTrajectoryGeneratorTestTools.createDefaultSupportPolygon(),
                                                                                      registry);
      CoPTrajectoryGeneratorState state = new CoPTrajectoryGeneratorState(registry);
      new DefaultParameterReader().readParametersInRegistry(registry);

      copTrajectory.registerState(state);

      state.setInitialCoP(new FramePoint2D());

      for (RobotSide robotSide : RobotSide.values)
      {
         soleFrames.get(robotSide).setPositionWithoutChecksAndUpdate(0.0,  robotSide.negateIfRightSide(0.5) * stanceWidth, 0.0);
         polygons.put(robotSide, new FrameConvexPolygon2D(soleFrames.get(robotSide), CoPTrajectoryGeneratorTestTools.createDefaultSupportPolygon()));
      }

      Footstep footstep = new Footstep();
      footstep.getFootstepPose().getPosition().set(0.5, -0.5 * stanceWidth, 0.0);
      footstep.setRobotSide(RobotSide.RIGHT);

      double swingTime = 1.0;
      double transferTime = 0.6;
      double finalTransferTime = 1.0;
      FootstepTiming timing = new FootstepTiming();
      timing.setTimings(swingTime, transferTime);

      state.setFinalTransferDuration(finalTransferTime);
      state.addFootstep(footstep);
      state.addFootstepTiming(timing);
      state.initializeStance(polygons, soleFrames);

      copTrajectory.compute(state);

      List<? extends ContactStateProvider> contactStateProviders = copTrajectory.getContactStateProviders();
      for (int i = 0; i < contactStateProviders.size() - 1; i++)
      {
         EuclidFrameTestTools.assertFramePoint3DGeometricallyEquals(contactStateProviders.get(i).getCopEndPosition(), contactStateProviders.get(i + 1).getCopStartPosition(), epsilon);
         assertEquals(contactStateProviders.get(i).getTimeInterval().getEndTime(), contactStateProviders.get(i + 1).getTimeInterval().getStartTime(), epsilon);
      }

      // check the timings
      double time = 0.0;
      assertEquals(time, contactStateProviders.get(0).getTimeInterval().getStartTime(), epsilon);
      time += Math.min(transferSplitFraction * transferTime, parameters.getDurationForContinuityMaintenanceSegment());
      assertEquals(time, contactStateProviders.get(0).getTimeInterval().getEndTime(), epsilon);
      time += transferTime -  Math.min(transferSplitFraction * transferTime, parameters.getDurationForContinuityMaintenanceSegment());
      assertEquals(time, contactStateProviders.get(1).getTimeInterval().getEndTime(), epsilon);
      time += swingShiftFraction * swingSplitFraction * swingTime;
      assertEquals(time, contactStateProviders.get(2).getTimeInterval().getEndTime(), epsilon);
      time += swingShiftFraction * (1.0 - swingSplitFraction) * swingTime;
      assertEquals(time, contactStateProviders.get(3).getTimeInterval().getEndTime(), epsilon);
      time += (1.0 - swingShiftFraction) * swingTime;
      assertEquals(time, contactStateProviders.get(4).getTimeInterval().getEndTime(), epsilon);
      time += transferSplitFraction * finalTransferTime;
      assertEquals(time, contactStateProviders.get(5).getTimeInterval().getEndTime(), epsilon);
      time += (1.0 - transferSplitFraction) * finalTransferTime;
      assertEquals(time, contactStateProviders.get(6).getTimeInterval().getEndTime(), epsilon);

      if (visualize)
         CoPTrajectoryVisualizer.visualize(copTrajectory);
   }

   public static void main(String[] args)
   {
      YoRegistry registry = new YoRegistry("test");
      CoPTrajectoryParameters parameters = new CoPTrajectoryParameters();
      registry.addChild(parameters.getRegistry());
      WalkingCoPTrajectoryGenerator copTrajectory = new WalkingCoPTrajectoryGenerator(parameters,
                                                                                      CoPTrajectoryGeneratorTestTools.createDefaultSupportPolygon(),
                                                                                      registry);
      CoPTrajectoryGeneratorState state = new CoPTrajectoryGeneratorState(copTrajectory.getYoRegistry());
      copTrajectory.registerState(state);
      state.registerStateToSave(parameters);

      YoSaveableModuleStateTools.load(state);

      copTrajectory.compute(state);
      CoPTrajectoryVisualizer.visualize(copTrajectory);
   }
}
