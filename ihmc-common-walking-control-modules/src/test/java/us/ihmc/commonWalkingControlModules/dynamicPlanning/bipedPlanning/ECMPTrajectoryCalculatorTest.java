package us.ihmc.commonWalkingControlModules.dynamicPlanning.bipedPlanning;

import org.junit.jupiter.api.Test;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMTrajectoryPlanner;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateProvider;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.SettableContactStateProvider;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.math.trajectories.abstracts.AbstractFramePolynomial3D;
import us.ihmc.robotics.math.trajectories.core.FramePolynomial3D;
import us.ihmc.robotics.math.trajectories.generators.MultipleSegmentPositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.interfaces.*;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.time.TimeInterval;
import us.ihmc.robotics.time.TimeIntervalBasics;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static us.ihmc.humanoidRobotics.footstep.FootstepUtils.worldFrame;

public class ECMPTrajectoryCalculatorTest
{
   private static final double epsilon = 1e-4;

   @Test
   public void testThreeStepsWithoutCoM()
   {
      double gravityZ = -9.81;
      YoRegistry registry = new YoRegistry(getClass().getSimpleName());
      CoPTrajectoryParameters coPTrajectoryParameters = new CoPTrajectoryParameters();
      registry.addChild(coPTrajectoryParameters.getRegistry());


      WalkingCoPTrajectoryGenerator copTrajectoryGenerator = new WalkingCoPTrajectoryGenerator(coPTrajectoryParameters,
                                                                                               CoPTrajectoryGeneratorTestTools.createDefaultSupportPolygon(),
                                                                                               registry);
      CoPTrajectoryGeneratorState state = new CoPTrajectoryGeneratorState(registry);
      copTrajectoryGenerator.registerState(state);

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

      FootstepTiming timing1 = new FootstepTiming(0.6, 0.25);
      FootstepTiming timing2 = new FootstepTiming(0.6, 0.25);
      FootstepTiming timing3 = new FootstepTiming(0.6, 0.25);

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

      MultipleSegmentPositionTrajectoryGenerator<FramePolynomial3DBasics> desiredAngularMomentumTrajectories = new MultipleSegmentPositionTrajectoryGenerator<>(
            "",
            ReferenceFrame.getWorldFrame(),
            () -> new FramePolynomial3D(6, ReferenceFrame.getWorldFrame()),
            registry);

      Random random = new Random(1738L);
      List<FramePoint3DReadOnly> startAMValue = new ArrayList<>();
      List<FramePoint3DReadOnly> endAMValue = new ArrayList<>();
      List<FrameVector3DReadOnly> startAMRate = new ArrayList<>();
      List<FrameVector3DReadOnly> endAMRate = new ArrayList<>();
      List<FramePolynomial3DBasics> amSegment = new ArrayList<>();

      for (int i = 0; i < copTrajectories.size(); i++)
      {
         FramePoint3DReadOnly startValue = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());
         FramePoint3DReadOnly endValue = EuclidFrameRandomTools.nextFramePoint3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3DReadOnly startRate = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());
         FrameVector3DReadOnly endRate = EuclidFrameRandomTools.nextFrameVector3D(random, ReferenceFrame.getWorldFrame());

         TimeIntervalBasics timeInterval = copTrajectories.get(i).getTimeInterval();
         FramePolynomial3DBasics amTrajectory = new FramePolynomial3D(4, ReferenceFrame.getWorldFrame());
         amTrajectory.setCubic(0.0, timeInterval.getDuration(), startValue, startRate, endValue, endRate);
         amTrajectory.getTimeInterval().set(timeInterval);

         desiredAngularMomentumTrajectories.appendSegment(amTrajectory);

         startAMValue.add(startValue);
         startAMRate.add(startRate);
         endAMValue.add(endValue);
         endAMRate.add(endRate);

         amSegment.add(amTrajectory);
      }

      ECMPTrajectoryCalculator<SettableContactStateProvider> ecmpTrajectoryCalculator = new ECMPTrajectoryCalculator<>(1.0, gravityZ, SettableContactStateProvider::new, registry);
      List<SettableContactStateProvider> cmpTrajectories = ecmpTrajectoryCalculator.computeECMPTrajectory(copTrajectories, desiredAngularMomentumTrajectories);

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

         amSegment.get(i).compute(0.0);

         startCMPRateExpected.set(copTrajectories.get(i).getECMPStartVelocity());
         endCMPRateExpected.set(copTrajectories.get(i).getECMPEndVelocity());

         startCMPRateExpected.addX(1.0 / gravityZ * amSegment.get(i).getAcceleration().getY());
         startCMPRateExpected.addY(-1.0 / gravityZ * amSegment.get(i).getAcceleration().getX());

         amSegment.get(i).compute(copTrajectories.get(i).getTimeInterval().getDuration());

         endCMPRateExpected.addX(1.0 / gravityZ * amSegment.get(i).getAcceleration().getY());
         endCMPRateExpected.addY(-1.0 / gravityZ * amSegment.get(i).getAcceleration().getX());

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

         ContactStateProvider copProvider = copTrajectories.get(i);
         ContactStateProvider cmpProvider = cmpTrajectories.get(i);

         copTrajectory.setCubic(0.0, copProvider.getTimeInterval().getDuration(), copProvider.getECMPStartPosition(), copProvider.getECMPStartVelocity(),
                                copProvider.getECMPEndPosition(), copProvider.getECMPEndVelocity());
         ecmpTrajectory.setCubic(0.0, cmpProvider.getTimeInterval().getDuration(), cmpProvider.getECMPStartPosition(), cmpProvider.getECMPStartVelocity(),
                                 cmpProvider.getECMPEndPosition(), cmpProvider.getECMPEndVelocity());


         for (double time = 0.0; time <= copProvider.getTimeInterval().getDuration(); time += 0.001)
         {

            double globalTime = time + copProvider.getTimeInterval().getStartTime();
            if (time == 0.0)
               globalTime += 1e-7;
            desiredAngularMomentumTrajectories.compute(globalTime);
            FrameVector2D desiredECMPOffset = new FrameVector2D();
            FrameVector2D ecmpOffsetExpected = new FrameVector2D();

            ecmpOffsetExpected.setX(1.0 / gravityZ * desiredAngularMomentumTrajectories.getVelocity().getY());
            ecmpOffsetExpected.setY(-1.0 / gravityZ * desiredAngularMomentumTrajectories.getVelocity().getX());
            FramePoint3D desiredCoP = new FramePoint3D();
            ecmpTrajectoryCalculator.computeECMPOffset(desiredAngularMomentumTrajectories.getVelocity(), desiredECMPOffset);

            amSegment.get(i).compute(time);


            String errorMessage = "failed at time " + time + " of segment " + i;
            EuclidFrameTestTools.assertGeometricallyEquals(errorMessage, ecmpOffsetExpected, desiredECMPOffset, epsilon);
            EuclidFrameTestTools.assertGeometricallyEquals(errorMessage, amSegment.get(i).getPosition(), desiredAngularMomentumTrajectories.getPosition(), epsilon);
            EuclidFrameTestTools.assertGeometricallyEquals(errorMessage, amSegment.get(i).getVelocity(), desiredAngularMomentumTrajectories.getVelocity(), 1e-3);

            copTrajectory.compute(time);
            ecmpTrajectory.compute(time);

            desiredCoP.set(ecmpTrajectory.getPosition());
            desiredCoP.subX(desiredECMPOffset.getX());
            desiredCoP.subY(desiredECMPOffset.getY());

            EuclidFrameTestTools.assertGeometricallyEquals(errorMessage, copTrajectory.getPosition(), desiredCoP, epsilon);
         }
      }

   }
}
