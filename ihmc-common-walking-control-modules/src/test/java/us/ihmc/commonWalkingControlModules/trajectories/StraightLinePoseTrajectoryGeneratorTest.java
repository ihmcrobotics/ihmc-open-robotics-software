package us.ihmc.commonWalkingControlModules.trajectories;

import static us.ihmc.robotics.Assert.*;

import java.util.Random;

import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.robotics.math.trajectories.OrientationInterpolationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.StraightLinePositionTrajectoryGenerator;
import us.ihmc.robotics.trajectories.providers.ConstantDoubleProvider;
import us.ihmc.robotics.trajectories.providers.ConstantOrientationProvider;
import us.ihmc.robotics.trajectories.providers.ConstantPositionProvider;
import us.ihmc.robotics.trajectories.providers.OrientationProvider;
import us.ihmc.robotics.trajectories.providers.PositionProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

public class StraightLinePoseTrajectoryGeneratorTest
{
   private static final Random random = new Random(1516351L);

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final ReferenceFrame frameA = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("frameA", worldFrame, EuclidCoreRandomTools.nextRigidBodyTransform(random));

   private static final double EPSILON = 1.0e-10;

   @Test
   public void testCompareWithSingleFrameTrajectoryGenerators()
   {
      YoRegistry registry = new YoRegistry("youpiloup");
      StraightLinePoseTrajectoryGenerator trajToTest = new StraightLinePoseTrajectoryGenerator("blop", worldFrame, registry);

      DoubleProvider trajectoryTimeProvider = new ConstantDoubleProvider(10.0);
      FramePoint3D initialPosition = EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame, 100.0, 100.0, 100.0);
      PositionProvider initialPositionProvider = new ConstantPositionProvider(initialPosition);
      FramePoint3D finalPosition = EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame, 100.0, 100.0, 100.0);
      PositionProvider finalPositionProvider = new ConstantPositionProvider(finalPosition);

      FrameQuaternion initialOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);
      OrientationProvider initialOrientationProvider = new ConstantOrientationProvider(initialOrientation);
      FrameQuaternion finalOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);
      OrientationProvider finalOrientationProvider = new ConstantOrientationProvider(finalOrientation);

      StraightLinePositionTrajectoryGenerator originalPosition = new StraightLinePositionTrajectoryGenerator("position", worldFrame, trajectoryTimeProvider,
            initialPositionProvider, finalPositionProvider, registry);
      OrientationInterpolationTrajectoryGenerator originalOrientation = new OrientationInterpolationTrajectoryGenerator("orientation", worldFrame,
            trajectoryTimeProvider, initialOrientationProvider, finalOrientationProvider, registry);

      trajToTest.setInitialPose(initialPosition, initialOrientation);
      trajToTest.setFinalPose(new FramePose3D(finalPosition, finalOrientation));
      trajToTest.setTrajectoryTime(trajectoryTimeProvider.getValue());

      originalPosition.initialize();
      originalOrientation.initialize();
      trajToTest.initialize();

      double dt = 1.0e-3;
      FramePoint3D position1 = new FramePoint3D();
      FrameVector3D velocity1 = new FrameVector3D();
      FrameVector3D acceleration1 = new FrameVector3D();
      FrameQuaternion orientation1 = new FrameQuaternion();
      FrameVector3D angularVelocity1 = new FrameVector3D();
      FrameVector3D angularAcceleration1 = new FrameVector3D();

      FramePoint3D position2 = new FramePoint3D();
      FrameVector3D velocity2 = new FrameVector3D();
      FrameVector3D acceleration2 = new FrameVector3D();
      FrameQuaternion orientation2 = new FrameQuaternion();
      FrameVector3D angularVelocity2 = new FrameVector3D();
      FrameVector3D angularAcceleration2 = new FrameVector3D();

      for (double t = 0.0; t <= trajectoryTimeProvider.getValue(); t += dt)
      {
         originalPosition.compute(t);
         originalOrientation.compute(t);
         trajToTest.compute(t);

         originalPosition.getLinearData(position1, velocity1, acceleration1);
         originalOrientation.getAngularData(orientation1, angularVelocity1, angularAcceleration1);

         trajToTest.getLinearData(position2, velocity2, acceleration2);
         trajToTest.getAngularData(orientation2, angularVelocity2, angularAcceleration2);

         assertTrue(position1.epsilonEquals(position2, EPSILON));
         assertTrue(velocity1.epsilonEquals(velocity2, EPSILON));
         assertTrue(acceleration1.epsilonEquals(acceleration2, EPSILON));
         assertTrue(orientation1.epsilonEquals(orientation2, EPSILON));
         assertTrue(angularVelocity1.epsilonEquals(angularVelocity2, EPSILON));
         assertTrue(angularAcceleration1.epsilonEquals(angularAcceleration2, EPSILON));
      }
   }

   @Test
   public void testNegativeTime()
   {
      YoRegistry registry = new YoRegistry("youpiloup");
      StraightLinePoseTrajectoryGenerator trajToTest = new StraightLinePoseTrajectoryGenerator("blop", worldFrame, registry);

      DoubleProvider trajectoryTimeProvider = new ConstantDoubleProvider(10.0);
      FramePoint3D initialPosition = EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame, 100.0, 100.0, 100.0);
      FramePoint3D finalPosition = EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame, 100.0, 100.0, 100.0);

      FrameQuaternion initialOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);
      FrameQuaternion finalOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);

      trajToTest.setInitialPose(initialPosition, initialOrientation);
      trajToTest.setFinalPose(new FramePose3D(finalPosition, finalOrientation));
      trajToTest.setTrajectoryTime(trajectoryTimeProvider.getValue());

      trajToTest.initialize();
      trajToTest.compute(-5.0);

      FramePoint3D position1 = new FramePoint3D(initialPosition);
      FrameVector3D velocity1 = new FrameVector3D(worldFrame);
      FrameVector3D acceleration1 = new FrameVector3D(worldFrame);
      FrameQuaternion orientation1 = new FrameQuaternion(initialOrientation);
      FrameVector3D angularVelocity1 = new FrameVector3D(worldFrame);
      FrameVector3D angularAcceleration1 = new FrameVector3D(worldFrame);

      FramePoint3D position2 = new FramePoint3D();
      FrameVector3D velocity2 = new FrameVector3D();
      FrameVector3D acceleration2 = new FrameVector3D();
      FrameQuaternion orientation2 = new FrameQuaternion();
      FrameVector3D angularVelocity2 = new FrameVector3D();
      FrameVector3D angularAcceleration2 = new FrameVector3D();

      trajToTest.getLinearData(position2, velocity2, acceleration2);
      trajToTest.getAngularData(orientation2, angularVelocity2, angularAcceleration2);

      assertTrue(position1.epsilonEquals(position2, EPSILON));
      assertTrue(velocity1.epsilonEquals(velocity2, EPSILON));
      assertTrue(acceleration1.epsilonEquals(acceleration2, EPSILON));
      assertTrue(orientation1.epsilonEquals(orientation2, EPSILON));
      assertTrue(angularVelocity1.epsilonEquals(angularVelocity2, EPSILON));
      assertTrue(angularAcceleration1.epsilonEquals(angularAcceleration2, EPSILON));
   }

   @Test
   public void testTooBigTime()
   {
      YoRegistry registry = new YoRegistry("youpiloup");
      StraightLinePoseTrajectoryGenerator trajToTest = new StraightLinePoseTrajectoryGenerator("blop", worldFrame, registry);

      DoubleProvider trajectoryTimeProvider = new ConstantDoubleProvider(10.0);
      FramePoint3D initialPosition = EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame, 100.0, 100.0, 100.0);
      FramePoint3D finalPosition = EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame, 100.0, 100.0, 100.0);

      FrameQuaternion initialOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);
      FrameQuaternion finalOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);

      trajToTest.setInitialPose(initialPosition, initialOrientation);
      trajToTest.setFinalPose(new FramePose3D(finalPosition, finalOrientation));
      trajToTest.setTrajectoryTime(trajectoryTimeProvider.getValue());

      trajToTest.initialize();
      trajToTest.compute(15.0);

      FramePoint3D position1 = new FramePoint3D(finalPosition);
      FrameVector3D velocity1 = new FrameVector3D(worldFrame);
      FrameVector3D acceleration1 = new FrameVector3D(worldFrame);
      FrameQuaternion orientation1 = new FrameQuaternion(finalOrientation);
      FrameVector3D angularVelocity1 = new FrameVector3D(worldFrame);
      FrameVector3D angularAcceleration1 = new FrameVector3D(worldFrame);

      FramePoint3D position2 = new FramePoint3D();
      FrameVector3D velocity2 = new FrameVector3D();
      FrameVector3D acceleration2 = new FrameVector3D();
      FrameQuaternion orientation2 = new FrameQuaternion();
      FrameVector3D angularVelocity2 = new FrameVector3D();
      FrameVector3D angularAcceleration2 = new FrameVector3D();

      trajToTest.getLinearData(position2, velocity2, acceleration2);
      trajToTest.getAngularData(orientation2, angularVelocity2, angularAcceleration2);

      assertTrue(position1.epsilonEquals(position2, EPSILON));
      assertTrue(velocity1.epsilonEquals(velocity2, EPSILON));
      assertTrue(acceleration1.epsilonEquals(acceleration2, EPSILON));
      assertTrue(orientation1.epsilonEquals(orientation2, EPSILON));
      assertTrue(angularVelocity1.epsilonEquals(angularVelocity2, EPSILON));
      assertTrue(angularAcceleration1.epsilonEquals(angularAcceleration2, EPSILON));
   }

   @Test
   public void testMultipleFramesWithSingleFrameTrajectoryGenerators()
   {
      YoRegistry registry = new YoRegistry("youpiloup");
      StraightLinePoseTrajectoryGenerator trajToTest = new StraightLinePoseTrajectoryGenerator("blop", worldFrame, registry);

      DoubleProvider trajectoryTimeProvider = new ConstantDoubleProvider(10.0);
      FramePoint3D initialPosition = EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame, 100.0, 100.0, 100.0);
      PositionProvider initialPositionProvider = new ConstantPositionProvider(initialPosition);
      FramePoint3D finalPosition = EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame, 100.0, 100.0, 100.0);
      PositionProvider finalPositionProvider = new ConstantPositionProvider(finalPosition);

      FrameQuaternion initialOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);
      OrientationProvider initialOrientationProvider = new ConstantOrientationProvider(initialOrientation);
      FrameQuaternion finalOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);
      OrientationProvider finalOrientationProvider = new ConstantOrientationProvider(finalOrientation);

      StraightLinePositionTrajectoryGenerator originalPosition = new StraightLinePositionTrajectoryGenerator("position1", worldFrame, trajectoryTimeProvider,
            initialPositionProvider, finalPositionProvider, registry);
      OrientationInterpolationTrajectoryGenerator originalOrientation = new OrientationInterpolationTrajectoryGenerator("orientation1", worldFrame,
            trajectoryTimeProvider, initialOrientationProvider, finalOrientationProvider, registry);

      trajToTest.setInitialPose(initialPosition, initialOrientation);
      trajToTest.setFinalPose(new FramePose3D(finalPosition, finalOrientation));
      trajToTest.setTrajectoryTime(trajectoryTimeProvider.getValue());

      originalPosition.initialize();
      originalOrientation.initialize();
      trajToTest.initialize();

      double dt = 1.0e-3;
      FramePoint3D position1 = new FramePoint3D();
      FrameVector3D velocity1 = new FrameVector3D();
      FrameVector3D acceleration1 = new FrameVector3D();
      FrameQuaternion orientation1 = new FrameQuaternion();
      FrameVector3D angularVelocity1 = new FrameVector3D();
      FrameVector3D angularAcceleration1 = new FrameVector3D();

      FramePoint3D position2 = new FramePoint3D();
      FrameVector3D velocity2 = new FrameVector3D();
      FrameVector3D acceleration2 = new FrameVector3D();
      FrameQuaternion orientation2 = new FrameQuaternion();
      FrameVector3D angularVelocity2 = new FrameVector3D();
      FrameVector3D angularAcceleration2 = new FrameVector3D();

      for (double t = 0.0; t <= trajectoryTimeProvider.getValue(); t += dt)
      {
         originalPosition.compute(t);
         originalOrientation.compute(t);
         trajToTest.compute(t);

         originalPosition.getLinearData(position1, velocity1, acceleration1);
         originalOrientation.getAngularData(orientation1, angularVelocity1, angularAcceleration1);

         trajToTest.getLinearData(position2, velocity2, acceleration2);
         trajToTest.getAngularData(orientation2, angularVelocity2, angularAcceleration2);

         assertTrue(position1.epsilonEquals(position2, EPSILON));
         assertTrue(velocity1.epsilonEquals(velocity2, EPSILON));
         assertTrue(acceleration1.epsilonEquals(acceleration2, EPSILON));
         assertTrue(orientation1.epsilonEquals(orientation2, EPSILON));
         assertTrue(angularVelocity1.epsilonEquals(angularVelocity2, EPSILON));
         assertTrue(angularAcceleration1.epsilonEquals(angularAcceleration2, EPSILON));
      }

      // Do the same in another frame
      initialPosition = EuclidFrameRandomTools.nextFramePoint3D(random, frameA, 100.0, 100.0, 100.0);
      initialPositionProvider = new ConstantPositionProvider(initialPosition);
      finalPosition = EuclidFrameRandomTools.nextFramePoint3D(random, frameA, 100.0, 100.0, 100.0);
      finalPositionProvider = new ConstantPositionProvider(finalPosition);

      initialOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, frameA);
      initialOrientationProvider = new ConstantOrientationProvider(initialOrientation);
      finalOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, frameA);
      finalOrientationProvider = new ConstantOrientationProvider(finalOrientation);

      originalPosition = new StraightLinePositionTrajectoryGenerator("position2", frameA, trajectoryTimeProvider, initialPositionProvider,
            finalPositionProvider, registry);
      originalOrientation = new OrientationInterpolationTrajectoryGenerator("orientation2", frameA, trajectoryTimeProvider, initialOrientationProvider,
            finalOrientationProvider, registry);

      trajToTest.switchTrajectoryFrame(frameA);
      trajToTest.setInitialPose(initialPosition, initialOrientation);
      trajToTest.setFinalPose(new FramePose3D(finalPosition, finalOrientation));
      trajToTest.setTrajectoryTime(trajectoryTimeProvider.getValue());

      originalPosition.initialize();
      originalOrientation.initialize();
      trajToTest.initialize();

      for (double t = 0.0; t <= trajectoryTimeProvider.getValue(); t += dt)
      {
         originalPosition.compute(t);
         originalOrientation.compute(t);
         trajToTest.compute(t);

         originalPosition.getLinearData(position1, velocity1, acceleration1);
         originalOrientation.getAngularData(orientation1, angularVelocity1, angularAcceleration1);

         trajToTest.getLinearData(position2, velocity2, acceleration2);
         trajToTest.getAngularData(orientation2, angularVelocity2, angularAcceleration2);

         assertTrue(position1.epsilonEquals(position2, EPSILON));
         assertTrue(velocity1.epsilonEquals(velocity2, EPSILON));
         assertTrue(acceleration1.epsilonEquals(acceleration2, EPSILON));
         assertTrue(orientation1.epsilonEquals(orientation2, EPSILON));
         assertTrue(angularVelocity1.epsilonEquals(angularVelocity2, EPSILON));
         assertTrue(angularAcceleration1.epsilonEquals(angularAcceleration2, EPSILON));
      }
   }
}
