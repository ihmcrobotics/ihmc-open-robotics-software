package us.ihmc.robotics.math.trajectories;

import static us.ihmc.robotics.Assert.*;

import java.util.Random;

import org.junit.jupiter.api.Test;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Disabled;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.robotics.trajectories.providers.ConstantDoubleProvider;
import us.ihmc.robotics.trajectories.providers.ConstantOrientationProvider;
import us.ihmc.robotics.trajectories.providers.OrientationProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

public class SimpleOrientationTrajectoryGeneratorTest
{
   private static final Random random = new Random(1516351L);

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final ReferenceFrame frameA = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("frameA", worldFrame, EuclidCoreRandomTools.nextRigidBodyTransform(random));

   private static final double EPSILON = 1.0e-10;

	@Test
   public void testCompareWithSingleFrameTrajectoryGenerator()
   {
      YoRegistry registry = new YoRegistry("youpiloup");
      SimpleOrientationTrajectoryGenerator trajToTest = new SimpleOrientationTrajectoryGenerator("blop", worldFrame, registry);

      DoubleProvider trajectoryTimeProvider = new ConstantDoubleProvider(10.0);

      FrameQuaternion initialOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);
      OrientationProvider initialOrientationProvider = new ConstantOrientationProvider(initialOrientation);
      FrameQuaternion finalOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);
      OrientationProvider finalOrientationProvider = new ConstantOrientationProvider(finalOrientation);

      OrientationInterpolationTrajectoryGenerator originalOrientation = new OrientationInterpolationTrajectoryGenerator("orientation", worldFrame,
            trajectoryTimeProvider, initialOrientationProvider, finalOrientationProvider, registry);

      trajToTest.setInitialOrientation(initialOrientation);
      trajToTest.setFinalOrientation(finalOrientation);
      trajToTest.setTrajectoryTime(trajectoryTimeProvider.getValue());

      originalOrientation.initialize();
      trajToTest.initialize();

      double dt = 1.0e-3;
      FrameQuaternion orientation1 = new FrameQuaternion();
      FrameVector3D angularVelocity1 = new FrameVector3D();
      FrameVector3D angularAcceleration1 = new FrameVector3D();

      FrameQuaternion orientation2 = new FrameQuaternion();
      FrameVector3D angularVelocity2 = new FrameVector3D();
      FrameVector3D angularAcceleration2 = new FrameVector3D();

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

	@Test
   public void testNegativeTime()
   {
      YoRegistry registry = new YoRegistry("youpiloup");
      SimpleOrientationTrajectoryGenerator trajToTest = new SimpleOrientationTrajectoryGenerator("blop", worldFrame, registry);

      DoubleProvider trajectoryTimeProvider = new ConstantDoubleProvider(10.0);

      FrameQuaternion initialOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);
      FrameQuaternion finalOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);

      trajToTest.setInitialOrientation(initialOrientation);
      trajToTest.setFinalOrientation(finalOrientation);
      trajToTest.setTrajectoryTime(trajectoryTimeProvider.getValue());

      trajToTest.initialize();
      trajToTest.compute(-5.0);

      FrameQuaternion orientation1 = new FrameQuaternion(initialOrientation);
      FrameVector3D angularVelocity1 = new FrameVector3D(worldFrame);
      FrameVector3D angularAcceleration1 = new FrameVector3D(worldFrame);

      FrameQuaternion orientation2 = new FrameQuaternion();
      FrameVector3D angularVelocity2 = new FrameVector3D();
      FrameVector3D angularAcceleration2 = new FrameVector3D();

      trajToTest.getAngularData(orientation2, angularVelocity2, angularAcceleration2);

      assertTrue(orientation1.epsilonEquals(orientation2, EPSILON));
      assertTrue(angularVelocity1.epsilonEquals(angularVelocity2, EPSILON));
      assertTrue(angularAcceleration1.epsilonEquals(angularAcceleration2, EPSILON));
   }

	@Test
   public void testTooBigTime()
   {
      YoRegistry registry = new YoRegistry("youpiloup");
      SimpleOrientationTrajectoryGenerator trajToTest = new SimpleOrientationTrajectoryGenerator("blop", worldFrame, registry);

      DoubleProvider trajectoryTimeProvider = new ConstantDoubleProvider(10.0);

      FrameQuaternion initialOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);
      FrameQuaternion finalOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);

      trajToTest.setInitialOrientation(initialOrientation);
      trajToTest.setFinalOrientation(finalOrientation);
      trajToTest.setTrajectoryTime(trajectoryTimeProvider.getValue());

      trajToTest.initialize();
      trajToTest.compute(15.0);

      FrameQuaternion orientation1 = new FrameQuaternion(finalOrientation);
      FrameVector3D angularVelocity1 = new FrameVector3D(worldFrame);
      FrameVector3D angularAcceleration1 = new FrameVector3D(worldFrame);

      FrameQuaternion orientation2 = new FrameQuaternion();
      FrameVector3D angularVelocity2 = new FrameVector3D();
      FrameVector3D angularAcceleration2 = new FrameVector3D();

      trajToTest.getAngularData(orientation2, angularVelocity2, angularAcceleration2);

      assertTrue(orientation1.epsilonEquals(orientation2, EPSILON));
      assertTrue(angularVelocity1.epsilonEquals(angularVelocity2, EPSILON));
      assertTrue(angularAcceleration1.epsilonEquals(angularAcceleration2, EPSILON));
   }

	@Test
   public void testMultipleFramesWithSingleFrameTrajectoryGenerators()
   {
      YoRegistry registry = new YoRegistry("youpiloup");
      SimpleOrientationTrajectoryGenerator trajToTest = new SimpleOrientationTrajectoryGenerator("blop", true, worldFrame, registry);

      DoubleProvider trajectoryTimeProvider = new ConstantDoubleProvider(10.0);

      FrameQuaternion initialOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);
      OrientationProvider initialOrientationProvider = new ConstantOrientationProvider(initialOrientation);
      FrameQuaternion finalOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);
      OrientationProvider finalOrientationProvider = new ConstantOrientationProvider(finalOrientation);

      OrientationInterpolationTrajectoryGenerator originalOrientation = new OrientationInterpolationTrajectoryGenerator("orientation1", worldFrame,
            trajectoryTimeProvider, initialOrientationProvider, finalOrientationProvider, registry);

      trajToTest.setInitialOrientation(initialOrientation);
      trajToTest.setFinalOrientation(finalOrientation);
      trajToTest.setTrajectoryTime(trajectoryTimeProvider.getValue());

      originalOrientation.initialize();
      trajToTest.initialize();

      double dt = 1.0e-3;
      FrameQuaternion orientation1 = new FrameQuaternion();
      FrameVector3D angularVelocity1 = new FrameVector3D();
      FrameVector3D angularAcceleration1 = new FrameVector3D();

      FrameQuaternion orientation2 = new FrameQuaternion();
      FrameVector3D angularVelocity2 = new FrameVector3D();
      FrameVector3D angularAcceleration2 = new FrameVector3D();

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

      initialOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, frameA);
      initialOrientationProvider = new ConstantOrientationProvider(initialOrientation);
      finalOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, frameA);
      finalOrientationProvider = new ConstantOrientationProvider(finalOrientation);

      originalOrientation = new OrientationInterpolationTrajectoryGenerator("orientation2", frameA,
            trajectoryTimeProvider, initialOrientationProvider, finalOrientationProvider, registry);

      trajToTest.setReferenceFrame(frameA);
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
