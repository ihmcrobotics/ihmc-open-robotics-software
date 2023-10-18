package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

import static us.ihmc.humanoidRobotics.footstep.FootstepUtils.worldFrame;

import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMTrajectoryPlanner;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.SettableContactStateProvider;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.math.trajectories.core.FramePolynomial3D;
import us.ihmc.robotics.math.trajectories.generators.MultipleSegmentPositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.interfaces.FramePolynomial3DBasics;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.time.TimeIntervalBasics;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;
import us.ihmc.yoVariables.registry.YoRegistry;

public class AngularMomentumHandlerTest
{
   private static final double epsilon = 1e-4;

   @Test
   public void testThreeStepsWithoutCoMPlanner()
   {
      double gravityZ = -9.81;
      double transferDuration = 0.25;
      double swingDuration = 0.6;
      YoRegistry registry = new YoRegistry(getClass().getSimpleName());
      CoPTrajectoryParameters coPTrajectoryParameters = new CoPTrajectoryParameters();
      registry.addChild(coPTrajectoryParameters.getRegistry());


      WalkingCoPTrajectoryGenerator copTrajectoryGenerator = new WalkingCoPTrajectoryGenerator(coPTrajectoryParameters,
                                                                                               CoPTrajectoryGeneratorTestTools.createDefaultSupportPolygon(),
                                                                                               registry);
      CoPTrajectoryGeneratorState state = new CoPTrajectoryGeneratorState(registry);
      copTrajectoryGenerator.registerState(state);

      AngularMomentumHandler<SettableContactStateProvider> angularMomentumHandler = new AngularMomentumHandler<>(1.0,
                                                                                                                 gravityZ,
                                                                                                                 null,
                                                                                                                 null,
                                                                                                                 SettableContactStateProvider::new,
                                                                                                                 registry,
                                                                                                                 null);

      DefaultParameterReader parameterReader = new DefaultParameterReader();
      parameterReader.readParametersInRegistry(registry);

      Footstep footstep1 = new Footstep();
      Footstep footstep2 = new Footstep();
      Footstep footstep3 = new Footstep();

      footstep1.setRobotSide(RobotSide.LEFT);
      footstep2.setRobotSide(RobotSide.RIGHT);
      footstep3.setRobotSide(RobotSide.LEFT);
      footstep1.setPose(new FramePose3D(new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.2, 0.1, 0.0), new FrameQuaternion()));
      footstep2.setPose(new FramePose3D(new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.4, -0.1, 0.0), new FrameQuaternion()));
      footstep3.setPose(new FramePose3D(new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.6, 0.1, 0.0), new FrameQuaternion()));

      FootstepTiming timing1 = new FootstepTiming(swingDuration, transferDuration);
      FootstepTiming timing2 = new FootstepTiming(swingDuration, transferDuration);
      FootstepTiming timing3 = new FootstepTiming(swingDuration, transferDuration);

      ReferenceFrame leftFootFrame = new ReferenceFrame("leftFootFrame", ReferenceFrame.getWorldFrame())
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.setTranslationAndIdentityRotation(new Vector3D(0.0, 0.1, 0.0));
         }
      };

      ReferenceFrame rightFootFrame = new ReferenceFrame("rightFootFrame", ReferenceFrame.getWorldFrame())
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.setTranslationAndIdentityRotation(new Vector3D(0.0, -0.1, 0.0));
         }
      };


      state.addFootstep(footstep1);
      state.addFootstep(footstep2);
      state.addFootstep(footstep3);
      state.addFootstepTiming(timing1);
      state.addFootstepTiming(timing2);
      state.addFootstepTiming(timing3);
      state.setInitialCoP(new FramePoint3D());
      state.setFinalTransferDuration(1.0);

      state.initializeStance(RobotSide.LEFT,
                             new FrameConvexPolygon2D(leftFootFrame, CoPTrajectoryGeneratorTestTools.createDefaultSupportPolygon().get(RobotSide.LEFT)),
                             leftFootFrame);
      state.initializeStance(RobotSide.RIGHT,
                             new FrameConvexPolygon2D(rightFootFrame, CoPTrajectoryGeneratorTestTools.createDefaultSupportPolygon().get(RobotSide.RIGHT)),
                             rightFootFrame);

      copTrajectoryGenerator.compute(state);
      List<SettableContactStateProvider> copTrajectories = copTrajectoryGenerator.getContactStateProviders();

      MultipleSegmentPositionTrajectoryGenerator<FramePolynomial3DBasics> desiredCoMTrajectories = new MultipleSegmentPositionTrajectoryGenerator<>(
            "",
            ReferenceFrame.getWorldFrame(),
            () -> new FramePolynomial3D(6, ReferenceFrame.getWorldFrame()),
            registry);

      FramePoint3D comStart = new FramePoint3D();
      {
         double startTime = 0.0;
         FramePolynomial3D comSegment = new FramePolynomial3D(6, worldFrame);
         comSegment.setLinear(0.0, swingDuration + transferDuration, comStart, footstep1.getFootstepPose().getPosition());
         comSegment.getTimeInterval().setInterval(startTime, startTime + swingDuration + transferDuration);
         desiredCoMTrajectories.appendSegment(comSegment);

         startTime += transferDuration + swingDuration;

         comSegment = new FramePolynomial3D(6, worldFrame);
         comSegment.setLinear(0.0, swingDuration, footstep1.getFootstepPose().getPosition(), footstep2.getFootstepPose().getPosition());
         comSegment.getTimeInterval().setInterval(startTime, startTime + swingDuration + transferDuration);
         desiredCoMTrajectories.appendSegment(comSegment);

         startTime += transferDuration + swingDuration;

         comSegment = new FramePolynomial3D(6, worldFrame);
         comSegment.setLinear(0.0, swingDuration, footstep2.getFootstepPose().getPosition(), footstep3.getFootstepPose().getPosition());
         comSegment.getTimeInterval().setInterval(startTime, startTime + swingDuration + transferDuration);
         desiredCoMTrajectories.appendSegment(comSegment);

         startTime += transferDuration + swingDuration;
         comSegment = new FramePolynomial3D(6, worldFrame);
         comSegment.setConstant(0.0, 1.0, footstep3.getFootstepPose().getPosition());
         comSegment.getTimeInterval().setInterval(startTime, startTime + 1.0);
         desiredCoMTrajectories.appendSegment(comSegment);
      }



      angularMomentumHandler.solveForAngularMomentumTrajectory(state, copTrajectories, desiredCoMTrajectories);

      List<FrameVector3DReadOnly> startAMRate = new ArrayList<>();
      List<FrameVector3DReadOnly> endAMRate = new ArrayList<>();

      for (int i = 0; i < copTrajectories.size(); i++)
      {
         TimeIntervalBasics timeInterval = copTrajectories.get(i).getTimeInterval();

         angularMomentumHandler.computeAngularMomentum(timeInterval.getStartTime() + 1e-8);

         startAMRate.add(new FrameVector3D(angularMomentumHandler.getDesiredAngularMomentumRate()));

         angularMomentumHandler.computeAngularMomentum(timeInterval.getEndTime() - 1e-8);

         endAMRate.add(new FrameVector3D(angularMomentumHandler.getDesiredAngularMomentumRate()));
      }

      List<SettableContactStateProvider> cmpTrajectories = angularMomentumHandler.computeECMPTrajectory(copTrajectories);


      gravityZ = Math.abs(gravityZ);

      for (int i = 0; i < cmpTrajectories.size() - 1; i++)
      {
         FramePoint3D startCMPExpected = new FramePoint3D();
         FramePoint3D endCMPExpected = new FramePoint3D();

         startCMPExpected.set(copTrajectories.get(i).getECMPStartPosition());
         endCMPExpected.set(copTrajectories.get(i).getECMPEndPosition());

         startCMPExpected.addX(1.0 / gravityZ * startAMRate.get(i).getY());
         startCMPExpected.addY(-1.0 / gravityZ * startAMRate.get(i).getX());

         endCMPExpected.addX(1.0 / gravityZ * endAMRate.get(i).getY());
         endCMPExpected.addY(-1.0 / gravityZ * endAMRate.get(i).getX());

         String errorMessage = "segment " + i + " failed.";
         EuclidFrameTestTools.assertGeometricallyEquals(errorMessage, startCMPExpected, cmpTrajectories.get(i).getECMPStartPosition(), epsilon);
         EuclidFrameTestTools.assertGeometricallyEquals(errorMessage, endCMPExpected, cmpTrajectories.get(i).getECMPEndPosition(), epsilon);


         FrameVector3D startCMPRateExpected = new FrameVector3D();
         FrameVector3D endCMPRateExpected = new FrameVector3D();

         angularMomentumHandler.getAngularMomentumTrajectories().getSegment(i).compute(0.0);

         startCMPRateExpected.set(copTrajectories.get(i).getECMPStartVelocity());
         endCMPRateExpected.set(copTrajectories.get(i).getECMPEndVelocity());

         startCMPRateExpected.addX(1.0 / gravityZ * angularMomentumHandler.getAngularMomentumTrajectories().getSegment(i).getAcceleration().getY());
         startCMPRateExpected.addY(-1.0 / gravityZ * angularMomentumHandler.getAngularMomentumTrajectories().getSegment(i).getAcceleration().getX());

         angularMomentumHandler.getAngularMomentumTrajectories().getSegment(i).compute(copTrajectories.get(i).getTimeInterval().getDuration());

         endCMPRateExpected.addX(1.0 / gravityZ * angularMomentumHandler.getAngularMomentumTrajectories().getSegment(i).getAcceleration().getY());
         endCMPRateExpected.addY(-1.0 / gravityZ * angularMomentumHandler.getAngularMomentumTrajectories().getSegment(i).getAcceleration().getX());

         EuclidFrameTestTools.assertGeometricallyEquals(errorMessage, startCMPRateExpected, cmpTrajectories.get(i).getECMPStartVelocity(), epsilon);
         EuclidFrameTestTools.assertGeometricallyEquals(errorMessage, endCMPRateExpected, cmpTrajectories.get(i).getECMPEndVelocity(), epsilon);
      }

      int lastIdx = copTrajectories.size() - 1;
      EuclidFrameTestTools.assertGeometricallyEquals(copTrajectories.get(lastIdx).getECMPStartPosition(), cmpTrajectories.get(lastIdx).getECMPStartPosition(), epsilon);
      EuclidFrameTestTools.assertGeometricallyEquals(copTrajectories.get(lastIdx).getECMPEndPosition(), cmpTrajectories.get(lastIdx).getECMPEndPosition(), epsilon);

      EuclidFrameTestTools.assertGeometricallyEquals(copTrajectories.get(lastIdx).getECMPStartVelocity(), cmpTrajectories.get(lastIdx).getECMPStartVelocity(), epsilon);
      EuclidFrameTestTools.assertGeometricallyEquals(copTrajectories.get(lastIdx).getECMPEndVelocity(), cmpTrajectories.get(lastIdx).getECMPEndVelocity(), epsilon);

      for (int i = 0; i < copTrajectories.size() - 1; i++)
      {
         FramePolynomial3D ecmpTrajectory = new FramePolynomial3D(6, worldFrame);
         FramePolynomial3D copTrajectory = new FramePolynomial3D(6, worldFrame);

         SettableContactStateProvider copProvider = copTrajectories.get(i);
         SettableContactStateProvider cmpProvider = cmpTrajectories.get(i);

         copTrajectory.setLinear(0.0, copProvider.getTimeInterval().getDuration(), copProvider.getECMPStartPosition(), copProvider.getECMPEndPosition());
         ecmpTrajectory.setCubic(0.0, cmpProvider.getTimeInterval().getDuration(), cmpProvider.getECMPStartPosition(), cmpProvider.getECMPStartVelocity(),
                                 cmpProvider.getECMPEndPosition(), cmpProvider.getECMPEndVelocity());


         for (double time = 0.0; time <= copProvider.getTimeInterval().getDuration(); time += 0.001)
         {

            double globalTime = time + copProvider.getTimeInterval().getStartTime();
            if (time == 0.0)
               globalTime += 1e-7;

            FramePoint3D desiredCoP = new FramePoint3D();



            String errorMessage = "failed at time " + time + " of segment " + i;

            copTrajectory.compute(time);
            ecmpTrajectory.compute(time);
            angularMomentumHandler.computeAngularMomentum(globalTime);

            angularMomentumHandler.computeCoPPosition(ecmpTrajectory.getPosition(), desiredCoP);
            EuclidFrameTestTools.assertGeometricallyEquals(errorMessage, copTrajectory.getPosition(), desiredCoP, epsilon);
         }
      }

   }

   @Test
   public void testThreeStepsWithCoMPlanner()
   {
      double gravityZ = -9.81;
      double transferDuration = 0.25;
      double swingDuration = 0.6;
      YoRegistry registry = new YoRegistry(getClass().getSimpleName());
      CoPTrajectoryParameters coPTrajectoryParameters = new CoPTrajectoryParameters();
      registry.addChild(coPTrajectoryParameters.getRegistry());


      WalkingCoPTrajectoryGenerator copTrajectoryGenerator = new WalkingCoPTrajectoryGenerator(coPTrajectoryParameters,
                                                                                               CoPTrajectoryGeneratorTestTools.createDefaultSupportPolygon(),
                                                                                               registry);
      CoPTrajectoryGeneratorState state = new CoPTrajectoryGeneratorState(registry);
      copTrajectoryGenerator.registerState(state);

      AngularMomentumHandler<SettableContactStateProvider> angularMomentumHandler = new AngularMomentumHandler<>(1.0,
                                                                                                                 gravityZ,
                                                                                                                 null,
                                                                                                                 null,
                                                                                                                 SettableContactStateProvider::new,
                                                                                                                 registry,
                                                                                                                 null);

      CoMTrajectoryPlanner comTrajectoryPlanner = new CoMTrajectoryPlanner(gravityZ, 1.0, registry);

      DefaultParameterReader parameterReader = new DefaultParameterReader();
      parameterReader.readParametersInRegistry(registry);

      Footstep footstep1 = new Footstep();
      Footstep footstep2 = new Footstep();
      Footstep footstep3 = new Footstep();

      footstep1.setRobotSide(RobotSide.LEFT);
      footstep2.setRobotSide(RobotSide.RIGHT);
      footstep3.setRobotSide(RobotSide.LEFT);
      footstep1.setPose(new FramePose3D(new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.2, 0.1, 0.0), new FrameQuaternion()));
      footstep2.setPose(new FramePose3D(new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.4, -0.1, 0.0), new FrameQuaternion()));
      footstep3.setPose(new FramePose3D(new FramePoint3D(ReferenceFrame.getWorldFrame(), 0.6, 0.1, 0.0), new FrameQuaternion()));

      FootstepTiming timing1 = new FootstepTiming(swingDuration, transferDuration);
      FootstepTiming timing2 = new FootstepTiming(swingDuration, transferDuration);
      FootstepTiming timing3 = new FootstepTiming(swingDuration, transferDuration);

      ReferenceFrame leftFootFrame = new ReferenceFrame("leftFootFrame", ReferenceFrame.getWorldFrame())
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.setTranslationAndIdentityRotation(new Vector3D(0.0, 0.1, 0.0));
         }
      };

      ReferenceFrame rightFootFrame = new ReferenceFrame("rightFootFrame", ReferenceFrame.getWorldFrame())
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.setTranslationAndIdentityRotation(new Vector3D(0.0, -0.1, 0.0));
         }
      };


      state.addFootstep(footstep1);
      state.addFootstep(footstep2);
      state.addFootstep(footstep3);
      state.addFootstepTiming(timing1);
      state.addFootstepTiming(timing2);
      state.addFootstepTiming(timing3);
      state.setInitialCoP(new FramePoint3D());
      state.setFinalTransferDuration(1.0);

      state.initializeStance(RobotSide.LEFT,
                             new FrameConvexPolygon2D(leftFootFrame, CoPTrajectoryGeneratorTestTools.createDefaultSupportPolygon().get(RobotSide.LEFT)),
                             leftFootFrame);
      state.initializeStance(RobotSide.RIGHT,
                             new FrameConvexPolygon2D(rightFootFrame, CoPTrajectoryGeneratorTestTools.createDefaultSupportPolygon().get(RobotSide.RIGHT)),
                             rightFootFrame);

      copTrajectoryGenerator.compute(state);
      List<SettableContactStateProvider> copTrajectories = copTrajectoryGenerator.getContactStateProviders();

      FramePoint3D initialCoM = new FramePoint3D(worldFrame, 0.0, 0.0, 1.0);
      comTrajectoryPlanner.setInitialCenterOfMassState(initialCoM, new FrameVector3D());
      comTrajectoryPlanner.solveForTrajectory(copTrajectories);



      angularMomentumHandler.solveForAngularMomentumTrajectory(state, copTrajectories, comTrajectoryPlanner.getCoMTrajectory());

      List<FrameVector3DReadOnly> startAMRate = new ArrayList<>();
      List<FrameVector3DReadOnly> endAMRate = new ArrayList<>();

      for (int i = 0; i < copTrajectories.size(); i++)
      {
         TimeIntervalBasics timeInterval = copTrajectories.get(i).getTimeInterval();

         angularMomentumHandler.computeAngularMomentum(timeInterval.getStartTime() + 1e-8);

         startAMRate.add(new FrameVector3D(angularMomentumHandler.getDesiredAngularMomentumRate()));

         angularMomentumHandler.computeAngularMomentum(timeInterval.getEndTime() - 1e-8);

         endAMRate.add(new FrameVector3D(angularMomentumHandler.getDesiredAngularMomentumRate()));
      }

      List<SettableContactStateProvider> cmpTrajectories = angularMomentumHandler.computeECMPTrajectory(copTrajectories);


      gravityZ = Math.abs(gravityZ);

      for (int i = 0; i < cmpTrajectories.size() - 1; i++)
      {
         FramePoint3D startCMPExpected = new FramePoint3D();
         FramePoint3D endCMPExpected = new FramePoint3D();

         startCMPExpected.set(copTrajectories.get(i).getECMPStartPosition());
         endCMPExpected.set(copTrajectories.get(i).getECMPEndPosition());

         startCMPExpected.addX(1.0 / gravityZ * startAMRate.get(i).getY());
         startCMPExpected.addY(-1.0 / gravityZ * startAMRate.get(i).getX());

         endCMPExpected.addX(1.0 / gravityZ * endAMRate.get(i).getY());
         endCMPExpected.addY(-1.0 / gravityZ * endAMRate.get(i).getX());

         String errorMessage = "segment " + i + " failed.";
         EuclidFrameTestTools.assertGeometricallyEquals(errorMessage, startCMPExpected, cmpTrajectories.get(i).getECMPStartPosition(), epsilon);
         EuclidFrameTestTools.assertGeometricallyEquals(errorMessage, endCMPExpected, cmpTrajectories.get(i).getECMPEndPosition(), epsilon);


         FrameVector3D startCMPRateExpected = new FrameVector3D();
         FrameVector3D endCMPRateExpected = new FrameVector3D();

         angularMomentumHandler.getAngularMomentumTrajectories().getSegment(i).compute(0.0);

         startCMPRateExpected.set(copTrajectories.get(i).getECMPStartVelocity());
         endCMPRateExpected.set(copTrajectories.get(i).getECMPEndVelocity());

         startCMPRateExpected.addX(1.0 / gravityZ * angularMomentumHandler.getAngularMomentumTrajectories().getSegment(i).getAcceleration().getY());
         startCMPRateExpected.addY(-1.0 / gravityZ * angularMomentumHandler.getAngularMomentumTrajectories().getSegment(i).getAcceleration().getX());

         angularMomentumHandler.getAngularMomentumTrajectories().getSegment(i).compute(copTrajectories.get(i).getTimeInterval().getDuration());

         endCMPRateExpected.addX(1.0 / gravityZ * angularMomentumHandler.getAngularMomentumTrajectories().getSegment(i).getAcceleration().getY());
         endCMPRateExpected.addY(-1.0 / gravityZ * angularMomentumHandler.getAngularMomentumTrajectories().getSegment(i).getAcceleration().getX());

         EuclidFrameTestTools.assertGeometricallyEquals(errorMessage, startCMPRateExpected, cmpTrajectories.get(i).getECMPStartVelocity(), epsilon);
         EuclidFrameTestTools.assertGeometricallyEquals(errorMessage, endCMPRateExpected, cmpTrajectories.get(i).getECMPEndVelocity(), epsilon);
      }

      int lastIdx = copTrajectories.size() - 1;
      EuclidFrameTestTools.assertGeometricallyEquals(copTrajectories.get(lastIdx).getECMPStartPosition(), cmpTrajectories.get(lastIdx).getECMPStartPosition(), epsilon);
      EuclidFrameTestTools.assertGeometricallyEquals(copTrajectories.get(lastIdx).getECMPEndPosition(), cmpTrajectories.get(lastIdx).getECMPEndPosition(), epsilon);

      EuclidFrameTestTools.assertGeometricallyEquals(copTrajectories.get(lastIdx).getECMPStartVelocity(), cmpTrajectories.get(lastIdx).getECMPStartVelocity(), epsilon);
      EuclidFrameTestTools.assertGeometricallyEquals(copTrajectories.get(lastIdx).getECMPEndVelocity(), cmpTrajectories.get(lastIdx).getECMPEndVelocity(), epsilon);


      comTrajectoryPlanner.solveForTrajectory(cmpTrajectories);

      for (int i = 0; i < copTrajectories.size() - 1; i++)
      {
         FramePolynomial3D ecmpTrajectory = new FramePolynomial3D(6, worldFrame);
         FramePolynomial3D copTrajectory = new FramePolynomial3D(6, worldFrame);

         SettableContactStateProvider copProvider = copTrajectories.get(i);
         SettableContactStateProvider cmpProvider = cmpTrajectories.get(i);

         copTrajectory.setLinear(0.0, copProvider.getTimeInterval().getDuration(), copProvider.getECMPStartPosition(), copProvider.getECMPEndPosition());
         ecmpTrajectory.setCubic(0.0, cmpProvider.getTimeInterval().getDuration(), cmpProvider.getECMPStartPosition(), cmpProvider.getECMPStartVelocity(),
                                 cmpProvider.getECMPEndPosition(), cmpProvider.getECMPEndVelocity());


         for (double time = 0.0; time <= copProvider.getTimeInterval().getDuration(); time += 0.001)
         {

            double globalTime = time + copProvider.getTimeInterval().getStartTime();
            if (time == 0.0)
               globalTime += 1e-7;

            FramePoint3D desiredCoP = new FramePoint3D();

            String errorMessage = "failed at time " + time + " of segment " + i;

            copTrajectory.compute(time);
            ecmpTrajectory.compute(time);
            comTrajectoryPlanner.compute(globalTime);
            angularMomentumHandler.computeAngularMomentum(globalTime);

            EuclidFrameTestTools.assertGeometricallyEquals(ecmpTrajectory.getPosition(), comTrajectoryPlanner.getDesiredECMPPosition(), epsilon);
            angularMomentumHandler.computeCoPPosition(ecmpTrajectory.getPosition(), desiredCoP);
            EuclidFrameTestTools.assertGeometricallyEquals(errorMessage, copTrajectory.getPosition(), desiredCoP, epsilon);
         }
      }

   }
}
