package us.ihmc.commonWalkingControlModules.trajectories;

import static us.ihmc.robotics.Assert.*;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.robotics.math.trajectories.OrientationInterpolationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.StraightLinePositionTrajectoryGenerator;
import us.ihmc.robotics.trajectories.providers.FrameOrientationProvider;
import us.ihmc.robotics.trajectories.providers.FramePositionProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

public class StraightLinePoseTrajectoryGeneratorTest
{
   private static final Random random = new Random(1516351L);

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final ReferenceFrame frameA = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("frameA", worldFrame, EuclidCoreRandomTools.nextRigidBodyTransform(random));

   private static final double EPSILON = 1e-4;

   @Test
   public void testCompareWithSingleFrameTrajectoryGenerators()
   {
      YoRegistry registry = new YoRegistry("youpiloup");
      StraightLinePoseTrajectoryGenerator trajToTest = new StraightLinePoseTrajectoryGenerator("blop", worldFrame, registry);

      DoubleProvider trajectoryTimeProvider = () -> 10.0;
      FramePoint3D initialPosition = EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame, 100.0, 100.0, 100.0);
      FramePositionProvider initialPositionProvider = () -> initialPosition;
      FramePoint3D finalPosition = EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame, 100.0, 100.0, 100.0);
      FramePositionProvider finalPositionProvider = () -> finalPosition;

      FrameQuaternion initialOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);
      FrameOrientationProvider initialOrientationProvider = () -> initialOrientation;
      FrameQuaternion finalOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);
      FrameOrientationProvider finalOrientationProvider = () -> finalOrientation;

      StraightLinePositionTrajectoryGenerator originalPosition = new StraightLinePositionTrajectoryGenerator("position", worldFrame, trajectoryTimeProvider,
            initialPositionProvider, finalPositionProvider, registry);
      OrientationInterpolationTrajectoryGenerator originalOrientation = new OrientationInterpolationTrajectoryGenerator("orientation", worldFrame,
            trajectoryTimeProvider, initialOrientationProvider, finalOrientationProvider, registry);

      trajToTest.setInitialPose(initialPosition, initialOrientation);
      trajToTest.setFinalPose(finalPosition, finalOrientation);
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

         EuclidFrameTestTools.assertGeometricallyEquals(position1, position2, EPSILON);
         EuclidFrameTestTools.assertGeometricallyEquals(position1, originalPosition.getPosition(), EPSILON);
         EuclidFrameTestTools.assertGeometricallyEquals(position1, trajToTest.getPosition(), EPSILON);
         EuclidFrameTestTools.assertGeometricallyEquals(velocity1, velocity2, EPSILON);
         EuclidFrameTestTools.assertGeometricallyEquals(velocity1, originalPosition.getVelocity(), EPSILON);
         EuclidFrameTestTools.assertGeometricallyEquals(velocity1, trajToTest.getVelocity(), EPSILON);
         EuclidFrameTestTools.assertGeometricallyEquals(acceleration1, acceleration2, EPSILON);
         EuclidFrameTestTools.assertGeometricallyEquals(acceleration1, originalPosition.getAcceleration(), EPSILON);
         EuclidFrameTestTools.assertGeometricallyEquals(acceleration1, trajToTest.getAcceleration(), EPSILON);
         EuclidFrameTestTools.assertGeometricallyEquals(orientation1, orientation2, EPSILON);
         EuclidFrameTestTools.assertGeometricallyEquals(angularVelocity1, angularVelocity2, EPSILON);
         EuclidFrameTestTools.assertGeometricallyEquals(angularAcceleration1, angularAcceleration2, EPSILON);
      }
   }

   @Test
   public void testNegativeTime()
   {
      YoRegistry registry = new YoRegistry("youpiloup");
      StraightLinePoseTrajectoryGenerator trajToTest = new StraightLinePoseTrajectoryGenerator("blop", worldFrame, registry);

      DoubleProvider trajectoryTimeProvider = () -> 10.0;
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

      EuclidFrameTestTools.assertGeometricallyEquals(position1, position2, EPSILON);
      EuclidFrameTestTools.assertGeometricallyEquals(velocity1, velocity2, EPSILON);
      EuclidFrameTestTools.assertGeometricallyEquals(acceleration1, acceleration2, EPSILON);
      EuclidFrameTestTools.assertGeometricallyEquals(orientation1, orientation2, EPSILON);
      EuclidFrameTestTools.assertGeometricallyEquals(angularVelocity1, angularVelocity2, EPSILON);
      EuclidFrameTestTools.assertGeometricallyEquals(angularAcceleration1, angularAcceleration2, EPSILON);
   }

   @Test
   public void testTooBigTime()
   {
      YoRegistry registry = new YoRegistry("youpiloup");
      StraightLinePoseTrajectoryGenerator trajToTest = new StraightLinePoseTrajectoryGenerator("blop", worldFrame, registry);

      DoubleProvider trajectoryTimeProvider = () -> 10.0;
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

      EuclidFrameTestTools.assertGeometricallyEquals(position1, position2, EPSILON);
      EuclidFrameTestTools.assertGeometricallyEquals(velocity1, velocity2, EPSILON);
      EuclidFrameTestTools.assertGeometricallyEquals(acceleration1, acceleration2, EPSILON);
      EuclidFrameTestTools.assertGeometricallyEquals(orientation1, orientation2, EPSILON);
      EuclidFrameTestTools.assertGeometricallyEquals(angularVelocity1, angularVelocity2, EPSILON);
      EuclidFrameTestTools.assertGeometricallyEquals(angularAcceleration1, angularAcceleration2, EPSILON);
   }

   @Test
   public void testMultipleFramesWithSingleFrameTrajectoryGenerators()
   {
      YoRegistry registry = new YoRegistry("youpiloup");
      StraightLinePoseTrajectoryGenerator trajToTest = new StraightLinePoseTrajectoryGenerator("blop", worldFrame, registry);

      DoubleProvider trajectoryTimeProvider = () -> 10.0;
      final FramePoint3D initialPosition = EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame, 100.0, 100.0, 100.0);
      FramePositionProvider initialPositionProvider = () -> initialPosition;
      final FramePoint3D finalPosition = EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame, 100.0, 100.0, 100.0);
      FramePositionProvider finalPositionProvider = () -> finalPosition;

      final FrameQuaternion initialOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);
      FrameOrientationProvider initialOrientationProvider = () -> initialOrientation;
      final FrameQuaternion finalOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);
      FrameOrientationProvider finalOrientationProvider = () -> finalOrientation;

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

         EuclidFrameTestTools.assertGeometricallyEquals(position1, position2, EPSILON);
         EuclidFrameTestTools.assertGeometricallyEquals(velocity1, velocity2, EPSILON);
         EuclidFrameTestTools.assertGeometricallyEquals(acceleration1, acceleration2, EPSILON);
         EuclidFrameTestTools.assertGeometricallyEquals(orientation1, orientation2, EPSILON);
         EuclidFrameTestTools.assertGeometricallyEquals(angularVelocity1, angularVelocity2, EPSILON);
         EuclidFrameTestTools.assertGeometricallyEquals(angularAcceleration1, angularAcceleration2, EPSILON);
      }

      // Do the same in another frame
      initialPosition.setIncludingFrame(EuclidFrameRandomTools.nextFramePoint3D(random, frameA, 100.0, 100.0, 100.0));
      finalPosition.setIncludingFrame(EuclidFrameRandomTools.nextFramePoint3D(random, frameA, 100.0, 100.0, 100.0));

      initialOrientation.setIncludingFrame(EuclidFrameRandomTools.nextFrameQuaternion(random, frameA));
      finalOrientation.setIncludingFrame(EuclidFrameRandomTools.nextFrameQuaternion(random, frameA));

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

         EuclidFrameTestTools.assertGeometricallyEquals(position1, position2, EPSILON);
         EuclidFrameTestTools.assertGeometricallyEquals(velocity1, velocity2, EPSILON);
         EuclidFrameTestTools.assertGeometricallyEquals(acceleration1, acceleration2, EPSILON);
         EuclidFrameTestTools.assertGeometricallyEquals(orientation1, orientation2, EPSILON);
         EuclidFrameTestTools.assertGeometricallyEquals(angularVelocity1, angularVelocity2, EPSILON);
         EuclidFrameTestTools.assertGeometricallyEquals(angularAcceleration1, angularAcceleration2, EPSILON);
      }
   }
}
