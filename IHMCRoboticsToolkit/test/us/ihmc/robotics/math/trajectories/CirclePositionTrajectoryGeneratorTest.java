package us.ihmc.robotics.math.trajectories;

import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameTestTools;
import us.ihmc.robotics.random.RandomGeometry;
import us.ihmc.robotics.trajectories.providers.ConstantDoubleProvider;
import us.ihmc.robotics.trajectories.providers.ConstantPositionProvider;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;
import us.ihmc.robotics.trajectories.providers.PositionProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;


public class CirclePositionTrajectoryGeneratorTest
{

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testVersusNumericalDifferentiation()
   {
      Random random = new Random(12522535L);

      ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();
      FramePoint3D offset = new FramePoint3D(referenceFrame, 1.0, 2.0, 3.0);
      double rotationAngle = 2.0 * Math.PI;
      double trajectoryTime = 1.0;

      CirclePositionTrajectoryGenerator circleTrajectory = createTrajectory(referenceFrame, offset, rotationAngle, trajectoryTime);
      circleTrajectory.initialize();

      double deltaT = 1e-8;
      double tInitial = random.nextDouble() - deltaT;
      double tFinal = tInitial + deltaT;

      FramePoint3D position = new FramePoint3D();
      FrameVector3D velocity = new FrameVector3D();
      FrameVector3D acceleration = new FrameVector3D();

      circleTrajectory.compute(tInitial);
      circleTrajectory.getPosition(position);
      circleTrajectory.getVelocity(velocity);
      circleTrajectory.getAcceleration(acceleration);

      FramePoint3D position2 = new FramePoint3D();
      FrameVector3D velocity2 = new FrameVector3D();

      circleTrajectory.compute(tFinal);
      circleTrajectory.getPosition(position2);
      circleTrajectory.getVelocity(velocity2);

      FrameVector3D numericallyDifferentiatedVelocity = numericallyDifferentiate(deltaT, position, position2);
      EuclidFrameTestTools.assertFrameTuple3DEquals(velocity, numericallyDifferentiatedVelocity, 1e-5);

      FrameVector3D numericallyDifferentiatedAcceleration = numericallyDifferentiate(deltaT, velocity, velocity2);
      EuclidFrameTestTools.assertFrameTuple3DEquals(acceleration, numericallyDifferentiatedAcceleration, 1e-4);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testInitialAndFinalPositionFullCircle()
   {
      Random random = new Random(125125L);


      ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();
      FramePoint3D offset = new FramePoint3D(referenceFrame, RandomGeometry.nextVector3D(random));
      double rotationAngle = 2.0 * Math.PI;
      double trajectoryTime = random.nextDouble();

      CirclePositionTrajectoryGenerator circleTrajectory = createTrajectory(referenceFrame, offset, rotationAngle, trajectoryTime);
      circleTrajectory.initialize();

      FramePoint3D position = new FramePoint3D();
      FrameVector3D velocity = new FrameVector3D();
      FrameVector3D acceleration = new FrameVector3D();
      FrameVector3D zero = new FrameVector3D(referenceFrame);

      circleTrajectory.compute(0.0);
      circleTrajectory.getPosition(position);
      circleTrajectory.getVelocity(velocity);

      EuclidFrameTestTools.assertFrameTuple3DEquals(offset, position, 1e-12);
      EuclidFrameTestTools.assertFrameTuple3DEquals(zero, velocity, 1e-12);

      circleTrajectory.compute(trajectoryTime);
      circleTrajectory.getPosition(position);
      circleTrajectory.getVelocity(velocity);
      circleTrajectory.getAcceleration(acceleration);

      EuclidFrameTestTools.assertFrameTuple3DEquals(offset, position, 1e-12);
      EuclidFrameTestTools.assertFrameTuple3DEquals(zero, velocity, 1e-12);
      EuclidFrameTestTools.assertFrameTuple3DEquals(zero, acceleration, 1e-12);
   }

   private FrameVector3D numericallyDifferentiate(double deltaT, FrameVector3D velocity, FrameVector3D velocity2)
   {
      FrameVector3D ret = new FrameVector3D(velocity2);
      ret.sub(velocity);
      ret.scale(1.0 / deltaT);

      return ret;
   }

   private FrameVector3D numericallyDifferentiate(double deltaT, FramePoint3D position, FramePoint3D position2)
   {
      FrameVector3D ret = new FrameVector3D(position2);
      ret.sub(position);
      ret.scale(1.0 / deltaT);

      return ret;
   }

   private CirclePositionTrajectoryGenerator createTrajectory(ReferenceFrame referenceFrame, FramePoint3D offset, double rotationAngle, double trajectoryTime)
   {
      PositionProvider initialPositionProvider = new ConstantPositionProvider(offset);
      DoubleProvider desiredRotationAngleProvider = new ConstantDoubleProvider(rotationAngle);
      YoVariableRegistry parentRegistry = new YoVariableRegistry("CirclePositionTrajectoryGeneratorTest");

      return new CirclePositionTrajectoryGenerator("", referenceFrame, new ConstantDoubleProvider(trajectoryTime), initialPositionProvider, parentRegistry,
              desiredRotationAngleProvider);
   }
}
