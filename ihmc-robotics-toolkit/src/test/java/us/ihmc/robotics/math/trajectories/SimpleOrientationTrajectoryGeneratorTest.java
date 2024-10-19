package us.ihmc.robotics.math.trajectories;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.robotics.math.trajectories.core.SimpleOrientationTrajectoryGenerator;
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

      double trajectoryTime = 10.0;

      FrameQuaternion initialOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);
      FrameQuaternion finalOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);

      StraightLinePoseTrajectoryGenerator trajectoryGenerator = new StraightLinePoseTrajectoryGenerator("orientation", worldFrame, registry);
      trajectoryGenerator.setInitialPose(new FramePoint3D(worldFrame), initialOrientation);
      trajectoryGenerator.setFinalPose(new FramePoint3D(worldFrame), finalOrientation);
      trajectoryGenerator.setTrajectoryTime(trajectoryTime);


      trajToTest.setInitialOrientation(initialOrientation);
      trajToTest.setFinalOrientation(finalOrientation);
      trajToTest.setTrajectoryTime(trajectoryTime);

      trajectoryGenerator.initialize();
      trajToTest.initialize();

      double dt = 1.0e-3;
      FrameQuaternion orientation1 = new FrameQuaternion();
      FrameVector3D angularVelocity1 = new FrameVector3D();
      FrameVector3D angularAcceleration1 = new FrameVector3D();

      FrameQuaternion orientation2 = new FrameQuaternion();
      FrameVector3D angularVelocity2 = new FrameVector3D();
      FrameVector3D angularAcceleration2 = new FrameVector3D();

      for (double t = 0.0; t <= trajectoryTime; t += dt)
      {
         trajectoryGenerator.compute(t);
         trajToTest.compute(t);

         trajectoryGenerator.getAngularData(orientation1, angularVelocity1, angularAcceleration1);

         trajToTest.getAngularData(orientation2, angularVelocity2, angularAcceleration2);

         EuclidCoreTestTools.assertEquals(orientation1, orientation2, EPSILON);
         EuclidCoreTestTools.assertEquals(angularVelocity1, angularVelocity2, EPSILON);
         EuclidCoreTestTools.assertEquals(angularAcceleration1, angularAcceleration2, EPSILON);
      }
   }

	@Test
   public void testNegativeTime()
   {
      YoRegistry registry = new YoRegistry("youpiloup");
      SimpleOrientationTrajectoryGenerator trajToTest = new SimpleOrientationTrajectoryGenerator("blop", worldFrame, registry);

      DoubleProvider trajectoryTimeProvider = () -> 10.0;

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

      EuclidFrameTestTools.assertEquals(orientation1, orientation2, EPSILON);
      EuclidFrameTestTools.assertEquals(angularVelocity1, angularVelocity2, EPSILON);
      EuclidFrameTestTools.assertEquals(angularAcceleration1, angularAcceleration2, EPSILON);
   }

	@Test
   public void testTooBigTime()
   {
      YoRegistry registry = new YoRegistry("youpiloup");
      SimpleOrientationTrajectoryGenerator trajToTest = new SimpleOrientationTrajectoryGenerator("blop", worldFrame, registry);

      DoubleProvider trajectoryTimeProvider = () -> 10.0;

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

      EuclidFrameTestTools.assertEquals(orientation1, orientation2, EPSILON);
      EuclidFrameTestTools.assertEquals(angularVelocity1, angularVelocity2, EPSILON);
      EuclidFrameTestTools.assertEquals(angularAcceleration1, angularAcceleration2, EPSILON);
   }

	@Test
   public void testMultipleFramesWithSingleFrameTrajectoryGenerators()
   {
      YoRegistry registry = new YoRegistry("youpiloup");
      SimpleOrientationTrajectoryGenerator trajToTest = new SimpleOrientationTrajectoryGenerator("blop", true, worldFrame, registry);

      double trajectoryTime = 10.0;
      DoubleProvider trajectoryTimeProvider = () -> trajectoryTime;

      final FrameQuaternion initialOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);
      final FrameQuaternion finalOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);

      StraightLinePoseTrajectoryGenerator trajectoryGenerator = new StraightLinePoseTrajectoryGenerator("orientation", worldFrame, registry);
      trajectoryGenerator.setInitialPose(new FramePoint3D(worldFrame), initialOrientation);
      trajectoryGenerator.setFinalPose(new FramePoint3D(worldFrame), finalOrientation);
      trajectoryGenerator.setTrajectoryTime(trajectoryTime);


      trajToTest.setInitialOrientation(initialOrientation);
      trajToTest.setFinalOrientation(finalOrientation);
      trajToTest.setTrajectoryTime(trajectoryTime);

      trajectoryGenerator.initialize();
      trajToTest.initialize();

      double dt = 1.0e-3;
      FrameQuaternion orientation1 = new FrameQuaternion();
      FrameVector3D angularVelocity1 = new FrameVector3D();
      FrameVector3D angularAcceleration1 = new FrameVector3D();

      FrameQuaternion orientation2 = new FrameQuaternion();
      FrameVector3D angularVelocity2 = new FrameVector3D();
      FrameVector3D angularAcceleration2 = new FrameVector3D();

      for (double t = 0.0; t <= trajectoryTime; t += dt)
      {
         trajectoryGenerator.compute(t);
         trajToTest.compute(t);

         trajectoryGenerator.getAngularData(orientation1, angularVelocity1, angularAcceleration1);

         trajToTest.getAngularData(orientation2, angularVelocity2, angularAcceleration2);

         EuclidFrameTestTools.assertEquals(orientation1, orientation2, EPSILON);
         EuclidFrameTestTools.assertEquals(angularVelocity1, angularVelocity2, EPSILON);
         EuclidFrameTestTools.assertEquals(angularAcceleration1, angularAcceleration2, EPSILON);
      }

      // Do the same in another frame

      initialOrientation.setIncludingFrame(EuclidFrameRandomTools.nextFrameQuaternion(random, frameA));
      finalOrientation.setIncludingFrame(EuclidFrameRandomTools.nextFrameQuaternion(random, frameA));

      trajectoryGenerator = new StraightLinePoseTrajectoryGenerator("orientation2", frameA, registry);
      trajectoryGenerator.setInitialPose(new FramePoint3D(frameA), initialOrientation);
      trajectoryGenerator.setFinalPose(new FramePoint3D(frameA), finalOrientation);
      trajectoryGenerator.setTrajectoryTime(trajectoryTime);

      trajToTest.setReferenceFrame(frameA);
      trajToTest.setInitialOrientation(initialOrientation);
      trajToTest.setFinalOrientation(finalOrientation);
      trajToTest.setTrajectoryTime(trajectoryTimeProvider.getValue());

      trajectoryGenerator.initialize();
      trajToTest.initialize();

      for (double t = 0.0; t <= trajectoryTime; t += dt)
      {
         trajectoryGenerator.compute(t);
         trajToTest.compute(t);

         trajectoryGenerator.getAngularData(orientation1, angularVelocity1, angularAcceleration1);

         trajToTest.getAngularData(orientation2, angularVelocity2, angularAcceleration2);

         EuclidFrameTestTools.assertEquals(orientation1, orientation2, EPSILON);
         EuclidFrameTestTools.assertEquals(angularVelocity1, angularVelocity2, EPSILON);
         EuclidFrameTestTools.assertEquals(angularAcceleration1, angularAcceleration2, EPSILON);
      }
   }
}
