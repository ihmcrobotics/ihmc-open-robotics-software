package us.ihmc.commonWalkingControlModules.trajectories;

import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.trajectories.OrientationInterpolationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.StraightLinePositionTrajectoryGenerator;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.trajectories.providers.ConstantDoubleProvider;
import us.ihmc.robotics.trajectories.providers.ConstantOrientationProvider;
import us.ihmc.robotics.trajectories.providers.ConstantPositionProvider;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;
import us.ihmc.robotics.trajectories.providers.OrientationProvider;
import us.ihmc.robotics.trajectories.providers.PositionProvider;

public class StraightLinePoseTrajectoryGeneratorTest
{
   private static final Random random = new Random(1516351L);

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final ReferenceFrame frameA = ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent("frameA", worldFrame,
         EuclidCoreRandomTools.generateRandomRigidBodyTransform(random));

   private static final double EPSILON = 1.0e-10;

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testRuntimeExceptions()
   {
      YoVariableRegistry registry = new YoVariableRegistry("youpiloup");
      StraightLinePoseTrajectoryGenerator trajToTest1 = new StraightLinePoseTrajectoryGenerator("blop1", worldFrame, registry);
      StraightLinePoseTrajectoryGenerator trajToTest2 = new StraightLinePoseTrajectoryGenerator("blop2", false, worldFrame, registry);

      try
      {
         trajToTest1.registerNewTrajectoryFrame(frameA);
         fail("Should have thrown an exception.");
      }
      catch (Exception e)
      {
         // Good
      }

      try
      {
         trajToTest1.switchTrajectoryFrame(worldFrame);
         fail("Should have thrown an exception.");
      }
      catch (Exception e)
      {
         // Good
      }

      try
      {
         trajToTest2.registerNewTrajectoryFrame(frameA);
         fail("Should have thrown an exception.");
      }
      catch (Exception e)
      {
         // Good
      }

      try
      {
         trajToTest2.switchTrajectoryFrame(worldFrame);
         fail("Should have thrown an exception.");
      }
      catch (Exception e)
      {
         // Good
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.2)
   @Test(timeout = 30000)
   public void testCompareWithSingleFrameTrajectoryGenerators()
   {
      YoVariableRegistry registry = new YoVariableRegistry("youpiloup");
      StraightLinePoseTrajectoryGenerator trajToTest = new StraightLinePoseTrajectoryGenerator("blop", worldFrame, registry);

      DoubleProvider trajectoryTimeProvider = new ConstantDoubleProvider(10.0);
      FramePoint initialPosition = FramePoint.generateRandomFramePoint(random, worldFrame, 100.0, 100.0, 100.0);
      PositionProvider initialPositionProvider = new ConstantPositionProvider(initialPosition);
      FramePoint finalPosition = FramePoint.generateRandomFramePoint(random, worldFrame, 100.0, 100.0, 100.0);
      PositionProvider finalPositionProvider = new ConstantPositionProvider(finalPosition);

      FrameOrientation initialOrientation = FrameOrientation.generateRandomFrameOrientation(random, worldFrame);
      OrientationProvider initialOrientationProvider = new ConstantOrientationProvider(initialOrientation);
      FrameOrientation finalOrientation = FrameOrientation.generateRandomFrameOrientation(random, worldFrame);
      OrientationProvider finalOrientationProvider = new ConstantOrientationProvider(finalOrientation);

      StraightLinePositionTrajectoryGenerator originalPosition = new StraightLinePositionTrajectoryGenerator("position", worldFrame, trajectoryTimeProvider,
            initialPositionProvider, finalPositionProvider, registry);
      OrientationInterpolationTrajectoryGenerator originalOrientation = new OrientationInterpolationTrajectoryGenerator("orientation", worldFrame,
            trajectoryTimeProvider, initialOrientationProvider, finalOrientationProvider, registry);

      trajToTest.setInitialPose(initialPosition, initialOrientation);
      trajToTest.setFinalPose(new FramePose(finalPosition, finalOrientation));
      trajToTest.setTrajectoryTime(trajectoryTimeProvider.getValue());

      originalPosition.initialize();
      originalOrientation.initialize();
      trajToTest.initialize();

      double dt = 1.0e-3;
      FramePoint position1 = new FramePoint();
      FrameVector velocity1 = new FrameVector();
      FrameVector acceleration1 = new FrameVector();
      FrameOrientation orientation1 = new FrameOrientation();
      FrameVector angularVelocity1 = new FrameVector();
      FrameVector angularAcceleration1 = new FrameVector();

      FramePoint position2 = new FramePoint();
      FrameVector velocity2 = new FrameVector();
      FrameVector acceleration2 = new FrameVector();
      FrameOrientation orientation2 = new FrameOrientation();
      FrameVector angularVelocity2 = new FrameVector();
      FrameVector angularAcceleration2 = new FrameVector();

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

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testNegativeTime()
   {
      YoVariableRegistry registry = new YoVariableRegistry("youpiloup");
      StraightLinePoseTrajectoryGenerator trajToTest = new StraightLinePoseTrajectoryGenerator("blop", worldFrame, registry);

      DoubleProvider trajectoryTimeProvider = new ConstantDoubleProvider(10.0);
      FramePoint initialPosition = FramePoint.generateRandomFramePoint(random, worldFrame, 100.0, 100.0, 100.0);
      FramePoint finalPosition = FramePoint.generateRandomFramePoint(random, worldFrame, 100.0, 100.0, 100.0);

      FrameOrientation initialOrientation = FrameOrientation.generateRandomFrameOrientation(random, worldFrame);
      FrameOrientation finalOrientation = FrameOrientation.generateRandomFrameOrientation(random, worldFrame);

      trajToTest.setInitialPose(initialPosition, initialOrientation);
      trajToTest.setFinalPose(new FramePose(finalPosition, finalOrientation));
      trajToTest.setTrajectoryTime(trajectoryTimeProvider.getValue());

      trajToTest.initialize();
      trajToTest.compute(-5.0);

      FramePoint position1 = new FramePoint(initialPosition);
      FrameVector velocity1 = new FrameVector(worldFrame);
      FrameVector acceleration1 = new FrameVector(worldFrame);
      FrameOrientation orientation1 = new FrameOrientation(initialOrientation);
      FrameVector angularVelocity1 = new FrameVector(worldFrame);
      FrameVector angularAcceleration1 = new FrameVector(worldFrame);

      FramePoint position2 = new FramePoint();
      FrameVector velocity2 = new FrameVector();
      FrameVector acceleration2 = new FrameVector();
      FrameOrientation orientation2 = new FrameOrientation();
      FrameVector angularVelocity2 = new FrameVector();
      FrameVector angularAcceleration2 = new FrameVector();

      trajToTest.getLinearData(position2, velocity2, acceleration2);
      trajToTest.getAngularData(orientation2, angularVelocity2, angularAcceleration2);

      assertTrue(position1.epsilonEquals(position2, EPSILON));
      assertTrue(velocity1.epsilonEquals(velocity2, EPSILON));
      assertTrue(acceleration1.epsilonEquals(acceleration2, EPSILON));
      assertTrue(orientation1.epsilonEquals(orientation2, EPSILON));
      assertTrue(angularVelocity1.epsilonEquals(angularVelocity2, EPSILON));
      assertTrue(angularAcceleration1.epsilonEquals(angularAcceleration2, EPSILON));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testTooBigTime()
   {
      YoVariableRegistry registry = new YoVariableRegistry("youpiloup");
      StraightLinePoseTrajectoryGenerator trajToTest = new StraightLinePoseTrajectoryGenerator("blop", worldFrame, registry);

      DoubleProvider trajectoryTimeProvider = new ConstantDoubleProvider(10.0);
      FramePoint initialPosition = FramePoint.generateRandomFramePoint(random, worldFrame, 100.0, 100.0, 100.0);
      FramePoint finalPosition = FramePoint.generateRandomFramePoint(random, worldFrame, 100.0, 100.0, 100.0);

      FrameOrientation initialOrientation = FrameOrientation.generateRandomFrameOrientation(random, worldFrame);
      FrameOrientation finalOrientation = FrameOrientation.generateRandomFrameOrientation(random, worldFrame);

      trajToTest.setInitialPose(initialPosition, initialOrientation);
      trajToTest.setFinalPose(new FramePose(finalPosition, finalOrientation));
      trajToTest.setTrajectoryTime(trajectoryTimeProvider.getValue());

      trajToTest.initialize();
      trajToTest.compute(15.0);

      FramePoint position1 = new FramePoint(finalPosition);
      FrameVector velocity1 = new FrameVector(worldFrame);
      FrameVector acceleration1 = new FrameVector(worldFrame);
      FrameOrientation orientation1 = new FrameOrientation(finalOrientation);
      FrameVector angularVelocity1 = new FrameVector(worldFrame);
      FrameVector angularAcceleration1 = new FrameVector(worldFrame);

      FramePoint position2 = new FramePoint();
      FrameVector velocity2 = new FrameVector();
      FrameVector acceleration2 = new FrameVector();
      FrameOrientation orientation2 = new FrameOrientation();
      FrameVector angularVelocity2 = new FrameVector();
      FrameVector angularAcceleration2 = new FrameVector();

      trajToTest.getLinearData(position2, velocity2, acceleration2);
      trajToTest.getAngularData(orientation2, angularVelocity2, angularAcceleration2);

      assertTrue(position1.epsilonEquals(position2, EPSILON));
      assertTrue(velocity1.epsilonEquals(velocity2, EPSILON));
      assertTrue(acceleration1.epsilonEquals(acceleration2, EPSILON));
      assertTrue(orientation1.epsilonEquals(orientation2, EPSILON));
      assertTrue(angularVelocity1.epsilonEquals(angularVelocity2, EPSILON));
      assertTrue(angularAcceleration1.epsilonEquals(angularAcceleration2, EPSILON));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.2)
   @Test(timeout = 30000)
   public void testMultipleFramesWithSingleFrameTrajectoryGenerators()
   {
      YoVariableRegistry registry = new YoVariableRegistry("youpiloup");
      StraightLinePoseTrajectoryGenerator trajToTest = new StraightLinePoseTrajectoryGenerator("blop", true, worldFrame, registry);

      DoubleProvider trajectoryTimeProvider = new ConstantDoubleProvider(10.0);
      FramePoint initialPosition = FramePoint.generateRandomFramePoint(random, worldFrame, 100.0, 100.0, 100.0);
      PositionProvider initialPositionProvider = new ConstantPositionProvider(initialPosition);
      FramePoint finalPosition = FramePoint.generateRandomFramePoint(random, worldFrame, 100.0, 100.0, 100.0);
      PositionProvider finalPositionProvider = new ConstantPositionProvider(finalPosition);

      FrameOrientation initialOrientation = FrameOrientation.generateRandomFrameOrientation(random, worldFrame);
      OrientationProvider initialOrientationProvider = new ConstantOrientationProvider(initialOrientation);
      FrameOrientation finalOrientation = FrameOrientation.generateRandomFrameOrientation(random, worldFrame);
      OrientationProvider finalOrientationProvider = new ConstantOrientationProvider(finalOrientation);

      StraightLinePositionTrajectoryGenerator originalPosition = new StraightLinePositionTrajectoryGenerator("position1", worldFrame, trajectoryTimeProvider,
            initialPositionProvider, finalPositionProvider, registry);
      OrientationInterpolationTrajectoryGenerator originalOrientation = new OrientationInterpolationTrajectoryGenerator("orientation1", worldFrame,
            trajectoryTimeProvider, initialOrientationProvider, finalOrientationProvider, registry);

      trajToTest.setInitialPose(initialPosition, initialOrientation);
      trajToTest.setFinalPose(new FramePose(finalPosition, finalOrientation));
      trajToTest.setTrajectoryTime(trajectoryTimeProvider.getValue());

      originalPosition.initialize();
      originalOrientation.initialize();
      trajToTest.initialize();

      double dt = 1.0e-3;
      FramePoint position1 = new FramePoint();
      FrameVector velocity1 = new FrameVector();
      FrameVector acceleration1 = new FrameVector();
      FrameOrientation orientation1 = new FrameOrientation();
      FrameVector angularVelocity1 = new FrameVector();
      FrameVector angularAcceleration1 = new FrameVector();

      FramePoint position2 = new FramePoint();
      FrameVector velocity2 = new FrameVector();
      FrameVector acceleration2 = new FrameVector();
      FrameOrientation orientation2 = new FrameOrientation();
      FrameVector angularVelocity2 = new FrameVector();
      FrameVector angularAcceleration2 = new FrameVector();

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
      initialPosition = FramePoint.generateRandomFramePoint(random, frameA, 100.0, 100.0, 100.0);
      initialPositionProvider = new ConstantPositionProvider(initialPosition);
      finalPosition = FramePoint.generateRandomFramePoint(random, frameA, 100.0, 100.0, 100.0);
      finalPositionProvider = new ConstantPositionProvider(finalPosition);

      initialOrientation = FrameOrientation.generateRandomFrameOrientation(random, frameA);
      initialOrientationProvider = new ConstantOrientationProvider(initialOrientation);
      finalOrientation = FrameOrientation.generateRandomFrameOrientation(random, frameA);
      finalOrientationProvider = new ConstantOrientationProvider(finalOrientation);

      originalPosition = new StraightLinePositionTrajectoryGenerator("position2", frameA, trajectoryTimeProvider, initialPositionProvider,
            finalPositionProvider, registry);
      originalOrientation = new OrientationInterpolationTrajectoryGenerator("orientation2", frameA, trajectoryTimeProvider, initialOrientationProvider,
            finalOrientationProvider, registry);

      trajToTest.registerNewTrajectoryFrame(frameA);
      trajToTest.switchTrajectoryFrame(frameA);
      trajToTest.setInitialPose(initialPosition, initialOrientation);
      trajToTest.setFinalPose(new FramePose(finalPosition, finalOrientation));
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
