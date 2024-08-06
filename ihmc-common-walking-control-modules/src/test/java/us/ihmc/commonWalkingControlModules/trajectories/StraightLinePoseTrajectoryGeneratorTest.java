package us.ihmc.commonWalkingControlModules.trajectories;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.commons.InterpolationTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.robotics.math.interpolators.OrientationInterpolationCalculator;
import us.ihmc.robotics.math.trajectories.OrientationInterpolationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.core.Polynomial;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.trajectories.providers.FrameOrientationProvider;
import us.ihmc.robotics.trajectories.providers.FramePositionProvider;
import us.ihmc.robotics.trajectories.providers.SettableDoubleProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

import static org.junit.jupiter.api.Assertions.*;

public class StraightLinePoseTrajectoryGeneratorTest
{
   private static final Random random = new Random(1516351L);

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final ReferenceFrame frameA = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("frameA", worldFrame, EuclidCoreRandomTools.nextRigidBodyTransform(random));

   private String namePrefix = "namePrefixTEST";
   private static double trajectoryTime = 10.0;

   private static final double EPSILON = 1e-4;

   @Test
   public void testCompareWithSingleFrameTrajectoryGenerators()
   {
      YoRegistry registry = new YoRegistry("youpiloup");
      StraightLinePoseTrajectoryGenerator trajToTest = new StraightLinePoseTrajectoryGenerator("blop", worldFrame, registry);

      DoubleProvider trajectoryTimeProvider = () -> 10.0;
      FramePoint3D initialPosition = EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame, 100.0, 100.0, 100.0);
      FramePoint3D finalPosition = EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame, 100.0, 100.0, 100.0);

      FrameQuaternion initialOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);
      FrameQuaternion finalOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);

      Polynomial interpolationPolynomial = new Polynomial(6);
      interpolationPolynomial.setQuintic(0.0, trajectoryTimeProvider.getValue(), 0.0, 0.0, 0.0, 1.0, 0.0, 0.0);

      trajToTest.setInitialPose(initialPosition, initialOrientation);
      trajToTest.setFinalPose(finalPosition, finalOrientation);
      trajToTest.setTrajectoryTime(trajectoryTimeProvider.getValue());

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
         interpolationPolynomial.compute(t);

         boolean isDone = t >= trajectoryTimeProvider.getValue();

         double parameter = isDone ? 1.0 : interpolationPolynomial.getValue();
         double parameterd = isDone ? 0.0 : interpolationPolynomial.getVelocity();
         double parameterdd = isDone ? 0.0 : interpolationPolynomial.getAcceleration();

         OrientationInterpolationCalculator orientationInterpolationCalculator = new OrientationInterpolationCalculator();
         FrameVector3D differenceVector = new FrameVector3D(worldFrame);
         differenceVector.sub(finalPosition, initialPosition);

         position1.interpolate(initialPosition, finalPosition, parameter);
         velocity1.setAndScale(parameterd, differenceVector);
         acceleration1.setAndScale(parameterdd, differenceVector);

         orientation1.interpolate(initialOrientation, finalOrientation, parameter);
         orientationInterpolationCalculator.computeAngularVelocity(angularVelocity1, initialOrientation, finalOrientation, parameterd);
         orientationInterpolationCalculator.computeAngularAcceleration(angularAcceleration1, initialOrientation, finalOrientation, parameterdd);

         trajToTest.compute(t);

         trajToTest.getLinearData(position2, velocity2, acceleration2);
         trajToTest.getAngularData(orientation2, angularVelocity2, angularAcceleration2);

         EuclidFrameTestTools.assertGeometricallyEquals(position1, position2, EPSILON);
         EuclidFrameTestTools.assertGeometricallyEquals(position1, trajToTest.getPosition(), EPSILON);
         EuclidFrameTestTools.assertGeometricallyEquals(velocity1, velocity2, EPSILON);
         EuclidFrameTestTools.assertGeometricallyEquals(velocity1, trajToTest.getVelocity(), EPSILON);
         EuclidFrameTestTools.assertGeometricallyEquals(acceleration1, acceleration2, EPSILON);
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
      final FramePoint3D finalPosition = EuclidFrameRandomTools.nextFramePoint3D(random, worldFrame, 100.0, 100.0, 100.0);

      final FrameQuaternion initialOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);
      final FrameQuaternion finalOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);

      Polynomial interpolationPolynomial = new Polynomial(6);
      interpolationPolynomial.setQuintic(0.0, trajectoryTimeProvider.getValue(), 0.0, 0.0, 0.0, 1.0, 0.0, 0.0);


      trajToTest.setInitialPose(initialPosition, initialOrientation);
      trajToTest.setFinalPose(new FramePose3D(finalPosition, finalOrientation));
      trajToTest.setTrajectoryTime(trajectoryTimeProvider.getValue());

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
         interpolationPolynomial.compute(t);

         boolean isDone = t >= trajectoryTimeProvider.getValue();

         double parameter = isDone ? 1.0 : interpolationPolynomial.getValue();
         double parameterd = isDone ? 0.0 : interpolationPolynomial.getVelocity();
         double parameterdd = isDone ? 0.0 : interpolationPolynomial.getAcceleration();

         OrientationInterpolationCalculator orientationInterpolationCalculator = new OrientationInterpolationCalculator();
         FrameVector3D differenceVector = new FrameVector3D(worldFrame);
         differenceVector.sub(finalPosition, initialPosition);

         position1.interpolate(initialPosition, finalPosition, parameter);
         velocity1.setAndScale(parameterd, differenceVector);
         acceleration1.setAndScale(parameterdd, differenceVector);

         orientation1.interpolate(initialOrientation, finalOrientation, parameter);
         orientationInterpolationCalculator.computeAngularVelocity(angularVelocity1, initialOrientation, finalOrientation, parameterd);
         orientationInterpolationCalculator.computeAngularAcceleration(angularAcceleration1, initialOrientation, finalOrientation, parameterdd);

         trajToTest.compute(t);

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

      position1.changeFrame(frameA);
      orientation1.changeFrame(frameA);
      velocity1.changeFrame(frameA);
      angularVelocity1.changeFrame(frameA);
      acceleration1.changeFrame(frameA);
      angularAcceleration1.changeFrame(frameA);

      trajToTest.switchTrajectoryFrame(frameA);
      trajToTest.setInitialPose(initialPosition, initialOrientation);
      trajToTest.setFinalPose(new FramePose3D(finalPosition, finalOrientation));
      trajToTest.setTrajectoryTime(trajectoryTimeProvider.getValue());

      trajToTest.initialize();

      for (double t = 0.0; t <= trajectoryTimeProvider.getValue(); t += dt)
      {
         interpolationPolynomial.compute(t);

         boolean isDone = t >= trajectoryTimeProvider.getValue();

         double parameter = isDone ? 1.0 : interpolationPolynomial.getValue();
         double parameterd = isDone ? 0.0 : interpolationPolynomial.getVelocity();
         double parameterdd = isDone ? 0.0 : interpolationPolynomial.getAcceleration();

         OrientationInterpolationCalculator orientationInterpolationCalculator = new OrientationInterpolationCalculator();
         FrameVector3D differenceVector = new FrameVector3D(frameA);
         differenceVector.sub(finalPosition, initialPosition);

         position1.interpolate(initialPosition, finalPosition, parameter);
         velocity1.setAndScale(parameterd, differenceVector);
         acceleration1.setAndScale(parameterdd, differenceVector);

         orientation1.interpolate(initialOrientation, finalOrientation, parameter);
         orientationInterpolationCalculator.computeAngularVelocity(angularVelocity1, initialOrientation, finalOrientation, parameterd);
         orientationInterpolationCalculator.computeAngularAcceleration(angularAcceleration1, initialOrientation, finalOrientation, parameterdd);

         trajToTest.compute(t);

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

   @Test
   public void testPackAngularData()
   {
      ReferenceFrame referenceFrame = createTestFrame();
      FrameQuaternion orientation = new FrameQuaternion(referenceFrame);

      YoRegistry parentRegistry = new YoRegistry("registry");

      FrameQuaternion orientationToPack = new FrameQuaternion(referenceFrame);
      orientationToPack.setYawPitchRollIncludingFrame(referenceFrame, 4.4, 3.3, 1.4);

      StraightLinePoseTrajectoryGenerator generator = new StraightLinePoseTrajectoryGenerator(namePrefix, referenceFrame, parentRegistry);
      orientationToPack.setIncludingFrame(generator.getOrientation());

      generator.setInitialPose(new FramePoint3D(referenceFrame), orientation);
      generator.setFinalPose(new FramePoint3D(referenceFrame), orientation);
      generator.setTrajectoryTime(trajectoryTime);

      assertEquals(referenceFrame, orientationToPack.getReferenceFrame());

      orientationToPack.setIncludingFrame(generator.getOrientation());

      assertEquals(referenceFrame, orientationToPack.getReferenceFrame());

      FrameVector3D angularVelocityToPack = new FrameVector3D(ReferenceFrame.getWorldFrame(), 10.0, 10.0, 10.0);
      FrameVector3D angularAccelerationToPack = new FrameVector3D(ReferenceFrame.getWorldFrame(), 10.0, 10.0, 10.0);

      assertNotEquals(referenceFrame, angularVelocityToPack.getReferenceFrame());
      assertEquals(ReferenceFrame.getWorldFrame(), angularVelocityToPack.getReferenceFrame());

      assertNotEquals(referenceFrame, angularAccelerationToPack.getReferenceFrame());
      assertEquals(ReferenceFrame.getWorldFrame(), angularAccelerationToPack.getReferenceFrame());

      generator.getAngularData(orientationToPack, angularVelocityToPack, angularAccelerationToPack);

      assertEquals(0.0, orientationToPack.getYaw(), EPSILON);
      assertEquals(0.0, orientationToPack.getPitch(), EPSILON);
      assertEquals(0.0, orientationToPack.getRoll(), EPSILON);
      assertSame(referenceFrame, orientationToPack.getReferenceFrame());

      assertEquals(0.0, angularVelocityToPack.getX(), EPSILON);
      assertEquals(0.0, angularVelocityToPack.getY(), EPSILON);
      assertEquals(0.0, angularVelocityToPack.getZ(), EPSILON);
      assertSame(referenceFrame, angularVelocityToPack.getReferenceFrame());

      assertEquals(0.0, angularAccelerationToPack.getX(), EPSILON);
      assertEquals(0.0, angularAccelerationToPack.getY(), EPSILON);
      assertEquals(0.0, angularAccelerationToPack.getZ(), EPSILON);
      assertSame(referenceFrame, angularAccelerationToPack.getReferenceFrame());
   }


   @Test
   public void testIsDone()
   {
      ReferenceFrame referenceFrame = createTestFrame();
      StraightLinePoseTrajectoryGenerator generator = new StraightLinePoseTrajectoryGenerator(namePrefix, referenceFrame, new YoRegistry("test"));

      FrameQuaternion orientation = new FrameQuaternion(referenceFrame);
      generator.setInitialPose(new FramePoint3D(referenceFrame), orientation);
      generator.setFinalPose(new FramePoint3D(referenceFrame), orientation);
      generator.setTrajectoryTime(trajectoryTime);
      generator.initialize();
      generator.compute(5.0);
      assertFalse(generator.isDone());

      generator.compute(trajectoryTime + EPSILON);
      assertTrue(generator.isDone());
   }

   @Test
   public void testGet()
   {
      ReferenceFrame referenceFrame = createTestFrame();

      StraightLinePoseTrajectoryGenerator generator = new StraightLinePoseTrajectoryGenerator(namePrefix, referenceFrame, new YoRegistry("test"));
      assertEquals(referenceFrame, generator.getOrientation().getReferenceFrame());
   }

   @Test
   public void testPackAngularVelocity()
   {
      ReferenceFrame referenceFrame = createTestFrame();

      StraightLinePoseTrajectoryGenerator generator = new StraightLinePoseTrajectoryGenerator(namePrefix, referenceFrame, new YoRegistry("test"));

      FrameQuaternion orientation = new FrameQuaternion(referenceFrame);
      generator.setInitialPose(new FramePoint3D(referenceFrame), orientation);
      generator.setFinalPose(new FramePoint3D(referenceFrame), orientation);
      generator.setTrajectoryTime(trajectoryTime);

      FrameVector3D angularVelocityToPack = new FrameVector3D(ReferenceFrame.getWorldFrame(), 10.0, 10.0, 10.0);

      assertNotEquals(referenceFrame, angularVelocityToPack.getReferenceFrame());

      angularVelocityToPack.setIncludingFrame(generator.getAngularVelocity());

      assertEquals(0.0, angularVelocityToPack.getX(), EPSILON);
      assertEquals(0.0, angularVelocityToPack.getY(), EPSILON);
      assertEquals(0.0, angularVelocityToPack.getZ(), EPSILON);
      assertSame(referenceFrame, angularVelocityToPack.getReferenceFrame());
   }

   @Test
   public void testPackAngularAcceleration()
   {
      ReferenceFrame referenceFrame = createTestFrame();

      StraightLinePoseTrajectoryGenerator generator = new StraightLinePoseTrajectoryGenerator(namePrefix, referenceFrame, new YoRegistry("test"));

      FrameQuaternion orientation = new FrameQuaternion(referenceFrame);
      generator.setInitialPose(new FramePoint3D(referenceFrame), orientation);
      generator.setFinalPose(new FramePoint3D(referenceFrame), orientation);
      generator.setTrajectoryTime(trajectoryTime);

      FrameVector3D angularAccelerationToPack = new FrameVector3D(ReferenceFrame.getWorldFrame(), 10.0, 10.0, 10.0);

      assertNotEquals(referenceFrame, angularAccelerationToPack.getReferenceFrame());

      angularAccelerationToPack.setIncludingFrame(generator.getAngularAcceleration());

      assertEquals(0.0, angularAccelerationToPack.getX(), EPSILON);
      assertEquals(0.0, angularAccelerationToPack.getY(), EPSILON);
      assertEquals(0.0, angularAccelerationToPack.getZ(), EPSILON);
      assertSame(referenceFrame, angularAccelerationToPack.getReferenceFrame());
   }

   private static ReferenceFrame createTestFrame()
   {
      return new PoseReferenceFrame("TestFrame", worldFrame);
   }
}
