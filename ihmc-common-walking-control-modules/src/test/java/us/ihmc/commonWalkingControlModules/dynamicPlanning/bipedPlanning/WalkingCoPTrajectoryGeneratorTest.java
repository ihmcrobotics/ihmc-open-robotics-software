package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.capturePoint.splitFractionCalculation.DefaultSplitFractionCalculatorParameters;
import us.ihmc.commonWalkingControlModules.capturePoint.splitFractionCalculation.YoSplitFractionCalculatorParameters;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateProvider;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.SettableContactStateProvider;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.saveableModule.YoSaveableModuleStateTools;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.awt.*;
import java.util.List;

import static us.ihmc.robotics.Assert.assertEquals;

public class WalkingCoPTrajectoryGeneratorTest
{
   private static boolean visualize = true;

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

      state.setInitialCoP(new FramePoint3D());

      for (RobotSide robotSide : RobotSide.values)
      {
         soleFrames.get(robotSide).setPositionWithoutChecksAndUpdate(0.0,  robotSide.negateIfRightSide(0.5) * stanceWidth, 0.0);
         polygons.put(robotSide, new FrameConvexPolygon2D(soleFrames.get(robotSide), CoPTrajectoryGeneratorTestTools.createDefaultSupportPolygon().get(robotSide)));
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

      List<SettableContactStateProvider> contactStateProviders = copTrajectory.getContactStateProviders();
      for (int i = 0; i < contactStateProviders.size() - 1; i++)
      {
         EuclidFrameTestTools.assertGeometricallyEquals(contactStateProviders.get(i).getECMPEndPosition(), contactStateProviders.get(i + 1).getECMPStartPosition(), epsilon);
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

   @Test
   public void testSingleBigStepDownWithRightFoot()
   {
      double stanceWidth = 0.4;
      YoRegistry registry = new YoRegistry("test");
      SideDependentList<PoseReferenceFrame> soleFrames = CoPTrajectoryGeneratorTestTools.createSoleFrames();
      SideDependentList<FrameConvexPolygon2D> polygons = new SideDependentList<>();

      CoPTrajectoryParameters parameters = new CoPTrajectoryParameters();
      DefaultSplitFractionCalculatorParameters splitFractionCalculatorParameters = new DefaultSplitFractionCalculatorParameters();
      splitFractionCalculatorParameters.setCalculateSplitFractionsFromPositions(true);
      splitFractionCalculatorParameters.setCalculateSplitFractionsFromArea(false);

      registry.addChild(parameters.getRegistry());
      WalkingCoPTrajectoryGenerator copTrajectory = new WalkingCoPTrajectoryGenerator(parameters,
                                                                                      splitFractionCalculatorParameters,
                                                                                      CoPTrajectoryGeneratorTestTools.createDefaultSupportPolygon(),
                                                                                      registry);
      CoPTrajectoryGeneratorState state = new CoPTrajectoryGeneratorState(registry);
      new DefaultParameterReader().readParametersInRegistry(registry);

      copTrajectory.registerState(state);

      state.setInitialCoP(new FramePoint3D());

      for (RobotSide robotSide : RobotSide.values)
      {
         soleFrames.get(robotSide).setPositionWithoutChecksAndUpdate(0.0,  robotSide.negateIfRightSide(0.5) * stanceWidth, 0.0);
         polygons.put(robotSide, new FrameConvexPolygon2D(soleFrames.get(robotSide), CoPTrajectoryGeneratorTestTools.createDefaultSupportPolygon().get(robotSide)));
      }

      Footstep footstep = new Footstep();
      footstep.getFootstepPose().getPosition().set(0.5, -0.5 * stanceWidth, -0.5);
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

      List<SettableContactStateProvider> contactStateProviders = copTrajectory.getContactStateProviders();
      for (int i = 0; i < contactStateProviders.size() - 1; i++)
      {
         EuclidFrameTestTools.assertGeometricallyEquals(contactStateProviders.get(i).getECMPEndPosition(), contactStateProviders.get(i + 1).getECMPStartPosition(), epsilon);
         assertEquals(contactStateProviders.get(i).getTimeInterval().getEndTime(), contactStateProviders.get(i + 1).getTimeInterval().getStartTime(), epsilon);
      }

      double defaultTransferSplitFraction = parameters.getDefaultTransferSplitFraction();
      double swingShiftFraction = parameters.getDefaultSwingDurationShiftFraction();
      double swingSplitFraction = parameters.getDefaultSwingSplitFraction();
      double transferSplitFractionAtStepDown = splitFractionCalculatorParameters.getTransferSplitFractionAtFullDepth();

      // check the timings
      double time = 0.0;
      assertEquals(time, contactStateProviders.get(0).getTimeInterval().getStartTime(), epsilon);
      time += Math.min(defaultTransferSplitFraction * transferTime, parameters.getDurationForContinuityMaintenanceSegment());
      assertEquals(time, contactStateProviders.get(0).getTimeInterval().getEndTime(), epsilon);
      time += transferTime -  Math.min(defaultTransferSplitFraction * transferTime, parameters.getDurationForContinuityMaintenanceSegment());
      assertEquals(time, contactStateProviders.get(1).getTimeInterval().getEndTime(), epsilon);
      time += swingShiftFraction * swingSplitFraction * swingTime;
      assertEquals(time, contactStateProviders.get(2).getTimeInterval().getEndTime(), epsilon);
      time += swingShiftFraction * (1.0 - swingSplitFraction) * swingTime;
      assertEquals(time, contactStateProviders.get(3).getTimeInterval().getEndTime(), epsilon);
      time += (1.0 - swingShiftFraction) * swingTime;
      assertEquals(time, contactStateProviders.get(4).getTimeInterval().getEndTime(), epsilon);
      time += transferSplitFractionAtStepDown * finalTransferTime;
      assertEquals(time, contactStateProviders.get(5).getTimeInterval().getEndTime(), epsilon);
      time += (1.0 - transferSplitFractionAtStepDown) * finalTransferTime;
      assertEquals(time, contactStateProviders.get(6).getTimeInterval().getEndTime(), epsilon);

      FramePoint3D leftFoot = new FramePoint3D(soleFrames.get(RobotSide.LEFT));
      FramePoint3D rightFoot = new FramePoint3D(footstep.getFootstepPose().getPosition());

      leftFoot.changeFrame(ReferenceFrame.getWorldFrame());
      FramePoint3D expectedMidpoint = new FramePoint3D();
      expectedMidpoint.interpolate(leftFoot, rightFoot, splitFractionCalculatorParameters.getTransferFinalWeightDistributionAtFullDepth());

      EuclidFrameTestTools.assertGeometricallyEquals(expectedMidpoint, contactStateProviders.get(5).getECMPEndPosition(), 1e-5);
      EuclidFrameTestTools.assertGeometricallyEquals(expectedMidpoint, contactStateProviders.get(6).getECMPStartPosition(), 1e-5);
      EuclidFrameTestTools.assertGeometricallyEquals(expectedMidpoint, contactStateProviders.get(6).getECMPEndPosition(), 1e-5);

      if (visualize)
         CoPTrajectoryVisualizer.visualize(copTrajectory);
   }

   @Test
   public void testSingleBigStepDownWithLeftFoot()
   {
      double stanceWidth = 0.4;
      YoRegistry registry = new YoRegistry("test");
      SideDependentList<PoseReferenceFrame> soleFrames = CoPTrajectoryGeneratorTestTools.createSoleFrames();
      SideDependentList<FrameConvexPolygon2D> polygons = new SideDependentList<>();

      CoPTrajectoryParameters parameters = new CoPTrajectoryParameters();
      DefaultSplitFractionCalculatorParameters splitFractionCalculatorParameters = new DefaultSplitFractionCalculatorParameters();
      splitFractionCalculatorParameters.setCalculateSplitFractionsFromPositions(true);
      splitFractionCalculatorParameters.setCalculateSplitFractionsFromArea(false);

      registry.addChild(parameters.getRegistry());
      WalkingCoPTrajectoryGenerator copTrajectory = new WalkingCoPTrajectoryGenerator(parameters,
                                                                                      splitFractionCalculatorParameters,
                                                                                      CoPTrajectoryGeneratorTestTools.createDefaultSupportPolygon(),
                                                                                      registry);
      CoPTrajectoryGeneratorState state = new CoPTrajectoryGeneratorState(registry);
      new DefaultParameterReader().readParametersInRegistry(registry);

      copTrajectory.registerState(state);

      state.setInitialCoP(new FramePoint3D());

      for (RobotSide robotSide : RobotSide.values)
      {
         soleFrames.get(robotSide).setPositionWithoutChecksAndUpdate(0.0,  robotSide.negateIfRightSide(0.5) * stanceWidth, 0.0);
         polygons.put(robotSide, new FrameConvexPolygon2D(soleFrames.get(robotSide), CoPTrajectoryGeneratorTestTools.createDefaultSupportPolygon().get(robotSide)));
      }

      Footstep footstep = new Footstep();
      footstep.getFootstepPose().getPosition().set(0.5, 0.5 * stanceWidth, -0.5);
      footstep.setRobotSide(RobotSide.LEFT);

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

      List<SettableContactStateProvider> contactStateProviders = copTrajectory.getContactStateProviders();
      for (int i = 0; i < contactStateProviders.size() - 1; i++)
      {
         EuclidFrameTestTools.assertGeometricallyEquals(contactStateProviders.get(i).getECMPEndPosition(), contactStateProviders.get(i + 1).getECMPStartPosition(), epsilon);
         assertEquals(contactStateProviders.get(i).getTimeInterval().getEndTime(), contactStateProviders.get(i + 1).getTimeInterval().getStartTime(), epsilon);
      }

      double defaultTransferSplitFraction = parameters.getDefaultTransferSplitFraction();
      double swingShiftFraction = parameters.getDefaultSwingDurationShiftFraction();
      double swingSplitFraction = parameters.getDefaultSwingSplitFraction();
      double transferSplitFractionAtStepDown = splitFractionCalculatorParameters.getTransferSplitFractionAtFullDepth();

      // check the timings
      double time = 0.0;
      assertEquals(time, contactStateProviders.get(0).getTimeInterval().getStartTime(), epsilon);
      time += Math.min(defaultTransferSplitFraction * transferTime, parameters.getDurationForContinuityMaintenanceSegment());
      assertEquals(time, contactStateProviders.get(0).getTimeInterval().getEndTime(), epsilon);
      time += transferTime -  Math.min(defaultTransferSplitFraction * transferTime, parameters.getDurationForContinuityMaintenanceSegment());
      assertEquals(time, contactStateProviders.get(1).getTimeInterval().getEndTime(), epsilon);
      time += swingShiftFraction * swingSplitFraction * swingTime;
      assertEquals(time, contactStateProviders.get(2).getTimeInterval().getEndTime(), epsilon);
      time += swingShiftFraction * (1.0 - swingSplitFraction) * swingTime;
      assertEquals(time, contactStateProviders.get(3).getTimeInterval().getEndTime(), epsilon);
      time += (1.0 - swingShiftFraction) * swingTime;
      assertEquals(time, contactStateProviders.get(4).getTimeInterval().getEndTime(), epsilon);
      time += transferSplitFractionAtStepDown * finalTransferTime;
      assertEquals(time, contactStateProviders.get(5).getTimeInterval().getEndTime(), epsilon);
      time += (1.0 - transferSplitFractionAtStepDown) * finalTransferTime;
      assertEquals(time, contactStateProviders.get(6).getTimeInterval().getEndTime(), epsilon);

      FramePoint3D leftFoot = new FramePoint3D(footstep.getFootstepPose().getPosition());
      FramePoint3D rightFoot = new FramePoint3D(soleFrames.get(RobotSide.RIGHT));

      rightFoot.changeFrame(ReferenceFrame.getWorldFrame());
      FramePoint3D expectedMidpoint = new FramePoint3D();
      expectedMidpoint.interpolate(rightFoot, leftFoot, splitFractionCalculatorParameters.getTransferFinalWeightDistributionAtFullDepth());

      EuclidFrameTestTools.assertGeometricallyEquals(expectedMidpoint, contactStateProviders.get(5).getECMPEndPosition(), 1e-5);
      EuclidFrameTestTools.assertGeometricallyEquals(expectedMidpoint, contactStateProviders.get(6).getECMPStartPosition(), 1e-5);
      EuclidFrameTestTools.assertGeometricallyEquals(expectedMidpoint, contactStateProviders.get(6).getECMPEndPosition(), 1e-5);

      if (visualize)
         CoPTrajectoryVisualizer.visualize(copTrajectory);
   }

   @Test
   public void testWeirdStepDown()
   {
      YoRegistry registry = new YoRegistry("test");
      SideDependentList<PoseReferenceFrame> soleFrames = CoPTrajectoryGeneratorTestTools.createSoleFrames();
      SideDependentList<FrameConvexPolygon2D> polygons = new SideDependentList<>();

      CoPTrajectoryParameters parameters = new CoPTrajectoryParameters();
      DefaultSplitFractionCalculatorParameters splitFractionCalculatorParameters = new DefaultSplitFractionCalculatorParameters();
      splitFractionCalculatorParameters.setCalculateSplitFractionsFromPositions(true);
      splitFractionCalculatorParameters.setCalculateSplitFractionsFromArea(false);
      splitFractionCalculatorParameters.setStepHeightForLargeStepDown(0.1);
      splitFractionCalculatorParameters.setLargestStepDownHeight(0.15);
      splitFractionCalculatorParameters.setTransferSplitFractionAtFullDepth(0.3);
      splitFractionCalculatorParameters.setTransferWeightDistributionAtFullDepth(0.75);
      splitFractionCalculatorParameters.setTransferFinalWeightDistributionAtFullDepth(0.8);

      registry.addChild(parameters.getRegistry());
      WalkingCoPTrajectoryGenerator copTrajectory = new WalkingCoPTrajectoryGenerator(parameters,
                                                                                      splitFractionCalculatorParameters,
                                                                                      CoPTrajectoryGeneratorTestTools.createDefaultSupportPolygon(),
                                                                                      registry);
      CoPTrajectoryGeneratorState state = new CoPTrajectoryGeneratorState(registry);
      new DefaultParameterReader().readParametersInRegistry(registry);

      copTrajectory.registerState(state);

      state.setInitialCoP(new FramePoint3D(ReferenceFrame.getWorldFrame(), 2.3497, 0.8024, 0.2613));

      FramePose3D leftFootPose = new FramePose3D(ReferenceFrame.getWorldFrame(),
                                                 new Point3D(2.1486, 0.7727, 0.2502),
                                                 new Quaternion(-0.03736, 0.148, 0.4505, 0.8796));
      FramePose3D rightFootPose = new FramePose3D(ReferenceFrame.getWorldFrame(),
                                                 new Point3D(2.5466, 0.8474, 0.3083),
                                                 new Quaternion(-0.0124, 0.0057, 0.2141, 0.9767));
      soleFrames.get(RobotSide.LEFT).setPoseAndUpdate(leftFootPose);
      soleFrames.get(RobotSide.RIGHT).setPoseAndUpdate(rightFootPose);

      ConvexPolygon2D leftFootPolygon = new ConvexPolygon2D();
      ConvexPolygon2D rightFootPolygon = new ConvexPolygon2D();

      leftFootPolygon.addVertex(0.11, -0.0);
      leftFootPolygon.update();

      rightFootPolygon.addVertex(-0.11, 0.055);
      rightFootPolygon.addVertex(0.11, 0.0425);
      rightFootPolygon.addVertex(0.11, -0.0425);
      rightFootPolygon.addVertex(-0.11, -0.055);
      rightFootPolygon.update();

      polygons.put(RobotSide.LEFT, new FrameConvexPolygon2D(soleFrames.get(RobotSide.LEFT), leftFootPolygon));
      polygons.put(RobotSide.RIGHT, new FrameConvexPolygon2D(soleFrames.get(RobotSide.RIGHT), rightFootPolygon));

      Footstep footstep = new Footstep();
      footstep.getFootstepPose().getPosition().set(2.5162, 1.1913, 0.1341);
      footstep.getFootstepPose().getOrientation().set(-0.0047, -0.0129, 0.1081, 0.994);
      footstep.setRobotSide(RobotSide.LEFT);
      footstep.setPredictedContactPoints(rightFootPolygon.getVertexBufferView());

      double swingTime = 1.2817;
      double transferTime = 1.0;
      double finalTransferTime = 0.8;
      FootstepTiming timing = new FootstepTiming();
      timing.setTimings(swingTime, transferTime);

      state.setFinalTransferDuration(finalTransferTime);
      state.setPercentageStandingWeightDistributionOnLeftFoot(0.2);
      state.addFootstep(footstep);
      state.addFootstepTiming(timing);
      state.initializeStance(polygons, soleFrames);

      copTrajectory.compute(state);


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
