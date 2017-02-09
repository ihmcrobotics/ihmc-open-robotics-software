package us.ihmc.robotics.math.trajectories;

import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.trajectories.providers.ConstantDoubleProvider;
import us.ihmc.robotics.trajectories.providers.ConstantOrientationProvider;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;
import us.ihmc.robotics.trajectories.providers.OrientationProvider;

public class SimpleOrientationTrajectoryGeneratorTest
{
   private static final Random random = new Random(1516351L);

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final ReferenceFrame frameA = ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent("frameA", worldFrame,
         RigidBodyTransform.generateRandomTransform(random));

   private static final double EPSILON = 1.0e-10;

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testRuntimeExceptions()
   {
      YoVariableRegistry registry = new YoVariableRegistry("youpiloup");
      SimpleOrientationTrajectoryGenerator trajToTest1 = new SimpleOrientationTrajectoryGenerator("blop1", worldFrame, registry);
      SimpleOrientationTrajectoryGenerator trajToTest2 = new SimpleOrientationTrajectoryGenerator("blop2", false, worldFrame, registry);

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

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testCompareWithSingleFrameTrajectoryGenerator()
   {
      YoVariableRegistry registry = new YoVariableRegistry("youpiloup");
      SimpleOrientationTrajectoryGenerator trajToTest = new SimpleOrientationTrajectoryGenerator("blop", worldFrame, registry);

      DoubleProvider trajectoryTimeProvider = new ConstantDoubleProvider(10.0);

      FrameOrientation initialOrientation = FrameOrientation.generateRandomFrameOrientation(random, worldFrame);
      OrientationProvider initialOrientationProvider = new ConstantOrientationProvider(initialOrientation);
      FrameOrientation finalOrientation = FrameOrientation.generateRandomFrameOrientation(random, worldFrame);
      OrientationProvider finalOrientationProvider = new ConstantOrientationProvider(finalOrientation);

      OrientationInterpolationTrajectoryGenerator originalOrientation = new OrientationInterpolationTrajectoryGenerator("orientation", worldFrame,
            trajectoryTimeProvider, initialOrientationProvider, finalOrientationProvider, registry);

      trajToTest.setInitialOrientation(initialOrientation);
      trajToTest.setFinalOrientation(finalOrientation);
      trajToTest.setTrajectoryTime(trajectoryTimeProvider.getValue());

      originalOrientation.initialize();
      trajToTest.initialize();

      double dt = 1.0e-3;
      FrameOrientation orientation1 = new FrameOrientation();
      FrameVector angularVelocity1 = new FrameVector();
      FrameVector angularAcceleration1 = new FrameVector();

      FrameOrientation orientation2 = new FrameOrientation();
      FrameVector angularVelocity2 = new FrameVector();
      FrameVector angularAcceleration2 = new FrameVector();

      for (double t = 0.0; t <= trajectoryTimeProvider.getValue(); t += dt)
      {
         originalOrientation.compute(t);
         trajToTest.compute(t);

         originalOrientation.getAngularData(orientation1, angularVelocity1, angularAcceleration1);

         trajToTest.getAngularData(orientation2, angularVelocity2, angularAcceleration2);

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
      SimpleOrientationTrajectoryGenerator trajToTest = new SimpleOrientationTrajectoryGenerator("blop", worldFrame, registry);

      DoubleProvider trajectoryTimeProvider = new ConstantDoubleProvider(10.0);

      FrameOrientation initialOrientation = FrameOrientation.generateRandomFrameOrientation(random, worldFrame);
      FrameOrientation finalOrientation = FrameOrientation.generateRandomFrameOrientation(random, worldFrame);

      trajToTest.setInitialOrientation(initialOrientation);
      trajToTest.setFinalOrientation(finalOrientation);
      trajToTest.setTrajectoryTime(trajectoryTimeProvider.getValue());

      trajToTest.initialize();
      trajToTest.compute(-5.0);

      FrameOrientation orientation1 = new FrameOrientation(initialOrientation);
      FrameVector angularVelocity1 = new FrameVector(worldFrame);
      FrameVector angularAcceleration1 = new FrameVector(worldFrame);

      FrameOrientation orientation2 = new FrameOrientation();
      FrameVector angularVelocity2 = new FrameVector();
      FrameVector angularAcceleration2 = new FrameVector();

      trajToTest.getAngularData(orientation2, angularVelocity2, angularAcceleration2);

      assertTrue(orientation1.epsilonEquals(orientation2, EPSILON));
      assertTrue(angularVelocity1.epsilonEquals(angularVelocity2, EPSILON));
      assertTrue(angularAcceleration1.epsilonEquals(angularAcceleration2, EPSILON));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testTooBigTime()
   {
      YoVariableRegistry registry = new YoVariableRegistry("youpiloup");
      SimpleOrientationTrajectoryGenerator trajToTest = new SimpleOrientationTrajectoryGenerator("blop", worldFrame, registry);

      DoubleProvider trajectoryTimeProvider = new ConstantDoubleProvider(10.0);

      FrameOrientation initialOrientation = FrameOrientation.generateRandomFrameOrientation(random, worldFrame);
      FrameOrientation finalOrientation = FrameOrientation.generateRandomFrameOrientation(random, worldFrame);

      trajToTest.setInitialOrientation(initialOrientation);
      trajToTest.setFinalOrientation(finalOrientation);
      trajToTest.setTrajectoryTime(trajectoryTimeProvider.getValue());

      trajToTest.initialize();
      trajToTest.compute(15.0);

      FrameOrientation orientation1 = new FrameOrientation(finalOrientation);
      FrameVector angularVelocity1 = new FrameVector(worldFrame);
      FrameVector angularAcceleration1 = new FrameVector(worldFrame);

      FrameOrientation orientation2 = new FrameOrientation();
      FrameVector angularVelocity2 = new FrameVector();
      FrameVector angularAcceleration2 = new FrameVector();

      trajToTest.getAngularData(orientation2, angularVelocity2, angularAcceleration2);

      assertTrue(orientation1.epsilonEquals(orientation2, EPSILON));
      assertTrue(angularVelocity1.epsilonEquals(angularVelocity2, EPSILON));
      assertTrue(angularAcceleration1.epsilonEquals(angularAcceleration2, EPSILON));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.2)
	@Test(timeout = 30000)
   public void testMultipleFramesWithSingleFrameTrajectoryGenerators()
   {
      YoVariableRegistry registry = new YoVariableRegistry("youpiloup");
      SimpleOrientationTrajectoryGenerator trajToTest = new SimpleOrientationTrajectoryGenerator("blop", true, worldFrame, registry);

      DoubleProvider trajectoryTimeProvider = new ConstantDoubleProvider(10.0);

      FrameOrientation initialOrientation = FrameOrientation.generateRandomFrameOrientation(random, worldFrame);
      OrientationProvider initialOrientationProvider = new ConstantOrientationProvider(initialOrientation);
      FrameOrientation finalOrientation = FrameOrientation.generateRandomFrameOrientation(random, worldFrame);
      OrientationProvider finalOrientationProvider = new ConstantOrientationProvider(finalOrientation);

      OrientationInterpolationTrajectoryGenerator originalOrientation = new OrientationInterpolationTrajectoryGenerator("orientation1", worldFrame,
            trajectoryTimeProvider, initialOrientationProvider, finalOrientationProvider, registry);

      trajToTest.setInitialOrientation(initialOrientation);
      trajToTest.setFinalOrientation(finalOrientation);
      trajToTest.setTrajectoryTime(trajectoryTimeProvider.getValue());

      originalOrientation.initialize();
      trajToTest.initialize();

      double dt = 1.0e-3;
      FrameOrientation orientation1 = new FrameOrientation();
      FrameVector angularVelocity1 = new FrameVector();
      FrameVector angularAcceleration1 = new FrameVector();

      FrameOrientation orientation2 = new FrameOrientation();
      FrameVector angularVelocity2 = new FrameVector();
      FrameVector angularAcceleration2 = new FrameVector();

      for (double t = 0.0; t <= trajectoryTimeProvider.getValue(); t += dt)
      {
         originalOrientation.compute(t);
         trajToTest.compute(t);

         originalOrientation.getAngularData(orientation1, angularVelocity1, angularAcceleration1);

         trajToTest.getAngularData(orientation2, angularVelocity2, angularAcceleration2);

         assertTrue(orientation1.epsilonEquals(orientation2, EPSILON));
         assertTrue(angularVelocity1.epsilonEquals(angularVelocity2, EPSILON));
         assertTrue(angularAcceleration1.epsilonEquals(angularAcceleration2, EPSILON));
      }

      // Do the same in another frame

      initialOrientation = FrameOrientation.generateRandomFrameOrientation(random, frameA);
      initialOrientationProvider = new ConstantOrientationProvider(initialOrientation);
      finalOrientation = FrameOrientation.generateRandomFrameOrientation(random, frameA);
      finalOrientationProvider = new ConstantOrientationProvider(finalOrientation);

      originalOrientation = new OrientationInterpolationTrajectoryGenerator("orientation2", frameA,
            trajectoryTimeProvider, initialOrientationProvider, finalOrientationProvider, registry);

      trajToTest.registerNewTrajectoryFrame(frameA);
      trajToTest.switchTrajectoryFrame(frameA);
      trajToTest.setInitialOrientation(initialOrientation);
      trajToTest.setFinalOrientation(finalOrientation);
      trajToTest.setTrajectoryTime(trajectoryTimeProvider.getValue());

      originalOrientation.initialize();
      trajToTest.initialize();

      for (double t = 0.0; t <= trajectoryTimeProvider.getValue(); t += dt)
      {
         originalOrientation.compute(t);
         trajToTest.compute(t);

         originalOrientation.getAngularData(orientation1, angularVelocity1, angularAcceleration1);

         trajToTest.getAngularData(orientation2, angularVelocity2, angularAcceleration2);

         assertTrue(orientation1.epsilonEquals(orientation2, EPSILON));
         assertTrue(angularVelocity1.epsilonEquals(angularVelocity2, EPSILON));
         assertTrue(angularAcceleration1.epsilonEquals(angularAcceleration2, EPSILON));
      }
   }
}
