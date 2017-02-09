package us.ihmc.robotics.math.trajectories;

import static org.junit.Assert.assertTrue;

import java.util.Random;

import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.math.QuaternionCalculus;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class VelocityConstrainedOrientationTrajectoryGeneratorTest
{
   private static boolean DEBUG = true;

   @ContinuousIntegrationTest(estimatedDuration = 2.2)
   @Test(timeout = 30000)
   public void testDerivativesConsistency() throws Exception
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      Random random = new Random(5165165161L);
      double dt = 1.0e-5;
      double trajectoryTime = 3.0;
      double startIntegrationTime = 1.0;
      double endIntegrationTime = 2.0;

      FrameOrientation currentOrientation = new FrameOrientation();
      FrameVector currentAngularVelocity = new FrameVector();
      FrameVector currentAngularAcceleration = new FrameVector();

      FrameOrientation orientationFromIntegration = new FrameOrientation();
      Vector3d angularVelocityVector = new Vector3d();
      Quat4d quaternionFromIntegration = new Quat4d();
      Quat4d integratedAngularVelocity = new Quat4d();

      FrameVector angularVelocityFromIntegration = new FrameVector();
      Vector3d integratedAngularAcceleration = new Vector3d();

      VelocityConstrainedOrientationTrajectoryGenerator traj = new VelocityConstrainedOrientationTrajectoryGenerator("traj", worldFrame, new YoVariableRegistry("null"));
      traj.setTrajectoryTime(trajectoryTime);

      for (int i = 0; i < 5; i++)
      {
         FrameOrientation initialOrientation = FrameOrientation.generateRandomFrameOrientation(random, worldFrame);
         FrameVector initialAngularVelocity = FrameVector.generateRandomFrameVector(random, worldFrame, -10.0, 10.0, -10.0, 10.0, -10.0, 10.0);
         FrameOrientation finalOrientation = FrameOrientation.generateRandomFrameOrientation(random, worldFrame);
         FrameVector finalAngularVelocity = FrameVector.generateRandomFrameVector(random, worldFrame, -10.0, 10.0, -10.0, 10.0, -10.0, 10.0);

         traj.setInitialConditions(initialOrientation, initialAngularVelocity);
         traj.setFinalConditions(finalOrientation, finalAngularVelocity);
         traj.initialize();


         traj.compute(startIntegrationTime - dt);
         traj.getOrientation(orientationFromIntegration);
         traj.getAngularVelocity(angularVelocityFromIntegration);

         for (double time = startIntegrationTime; time <= endIntegrationTime; time += dt)
         {
            traj.compute(time);
            traj.getAngularData(currentOrientation, currentAngularVelocity, currentAngularAcceleration);

            currentAngularVelocity.get(angularVelocityVector);
            RotationTools.integrateAngularVelocity(angularVelocityVector, dt, integratedAngularVelocity);
            orientationFromIntegration.getQuaternion(quaternionFromIntegration);
            quaternionFromIntegration.mul(integratedAngularVelocity, quaternionFromIntegration);
            orientationFromIntegration.set(quaternionFromIntegration);

            currentAngularAcceleration.get(integratedAngularAcceleration);
            integratedAngularAcceleration.scale(dt);
            angularVelocityFromIntegration.add(integratedAngularAcceleration);

            assertTrue(currentOrientation.epsilonEquals(orientationFromIntegration, 1.0e-4));
            assertTrue(currentAngularVelocity.epsilonEquals(angularVelocityFromIntegration, 1.0e-3));
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.2)
   @Test(timeout = 30000)
   public void testLimitConditions() throws Exception
   {
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      Random random = new Random(5165165161L);
      VelocityConstrainedOrientationTrajectoryGenerator traj = new VelocityConstrainedOrientationTrajectoryGenerator("traj", worldFrame, new YoVariableRegistry("null"));
      double trajectoryTime = 3.0;
      traj.setTrajectoryTime(trajectoryTime);

      for (int i = 0; i < 10000; i++)
      {
         FrameOrientation initialOrientation = FrameOrientation.generateRandomFrameOrientation(random, worldFrame);
         FrameVector initialAngularVelocity = FrameVector.generateRandomFrameVector(random, worldFrame, -10.0, 10.0, -10.0, 10.0, -10.0, 10.0);
         FrameOrientation finalOrientation = FrameOrientation.generateRandomFrameOrientation(random, worldFrame);
         FrameVector finalAngularVelocity = FrameVector.generateRandomFrameVector(random, worldFrame, -10.0, 10.0, -10.0, 10.0, -10.0, 10.0);

         FrameVector zeroAngularAcceleration = new FrameVector();

         traj.setInitialConditions(initialOrientation, initialAngularVelocity);
         traj.setFinalConditions(finalOrientation, finalAngularVelocity);
         traj.initialize();

         FrameOrientation currentOrientation = new FrameOrientation();
         FrameVector currentAngularVelocity = new FrameVector();
         FrameVector currentAngularAcceleration = new FrameVector();

         double dt = 1.0e-8;
         traj.compute(0.0 + dt);
         traj.getAngularData(currentOrientation, currentAngularVelocity, currentAngularAcceleration);

         double epsilon = 5.0e-5;
         double accelerationEpsilon = 5.0e-4;
         assertTrue(initialOrientation.epsilonEquals(currentOrientation, epsilon));
         boolean goodInitialVelocity = initialAngularVelocity.epsilonEquals(currentAngularVelocity, epsilon);
         if (DEBUG && !goodInitialVelocity)
         {
            FrameVector error = new FrameVector();
            error.sub(initialAngularVelocity, currentAngularVelocity);
            System.out.println("Bad initial velocity, error: " + error);
            printLimitConditions(initialOrientation, initialAngularVelocity, finalOrientation, finalAngularVelocity);
         }
         assertTrue(goodInitialVelocity);
         boolean goodInitialAngularAcceleration = zeroAngularAcceleration.epsilonEquals(currentAngularAcceleration, accelerationEpsilon);
         assertTrue(goodInitialAngularAcceleration);

         traj.compute(trajectoryTime - dt);
         traj.getAngularData(currentOrientation, currentAngularVelocity, currentAngularAcceleration);

         assertTrue(finalOrientation.epsilonEquals(currentOrientation, epsilon));
         assertTrue(finalAngularVelocity.epsilonEquals(currentAngularVelocity, epsilon));
         boolean goodFinalAngularAcceleration = zeroAngularAcceleration.epsilonEquals(currentAngularAcceleration, accelerationEpsilon);
         if (DEBUG && !goodFinalAngularAcceleration)
         {
            System.out.println("Bad final acceleration: " + currentAngularAcceleration);
            printLimitConditions(initialOrientation, initialAngularVelocity, finalOrientation, finalAngularVelocity);
         }
         assertTrue(goodFinalAngularAcceleration);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.6)
   @Test(timeout = 30000)
   public void testContinuityForSlowTrajectory() throws Exception
   {
      QuaternionCalculus quaternionCalculus = new QuaternionCalculus();
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      Random random = new Random(5165165161L);
      VelocityConstrainedOrientationTrajectoryGenerator traj = new VelocityConstrainedOrientationTrajectoryGenerator("traj", worldFrame, new YoVariableRegistry("null"));
      double trajectoryTime = 10.0;

      double maxVelocityRecorded = 0.0;
      double maxAccelerationRecorded = 0.0;
      double maxJerkRecorded = 0.0;

      traj.setTrajectoryTime(trajectoryTime);

      for (int i = 0; i < 100; i++)
      {
         FrameOrientation initialOrientation = FrameOrientation.generateRandomFrameOrientation(random, worldFrame);
         FrameVector initialAngularVelocity = FrameVector.generateRandomFrameVector(random, worldFrame);
         FrameOrientation finalOrientation = FrameOrientation.generateRandomFrameOrientation(random, worldFrame);
         FrameVector finalAngularVelocity = FrameVector.generateRandomFrameVector(random, worldFrame);

         traj.setInitialConditions(initialOrientation, initialAngularVelocity);
         traj.setFinalConditions(finalOrientation, finalAngularVelocity);
         traj.initialize();

         FrameOrientation currentOrientation = new FrameOrientation();
         FrameVector currentAngularVelocity = new FrameVector();
         FrameVector currentAngularAcceleration = new FrameVector();

         Quat4d currentQuaternion = new Quat4d();
         Quat4d previousQuaternion = new Quat4d();
         Vector3d delta = new Vector3d();

         FrameOrientation previousOrientation = new FrameOrientation();
         FrameVector previousAngularVelocity = new FrameVector();
         FrameVector previousAngularAcceleration = new FrameVector();

         
         double maxVelocity = 4.0;
         double maxAcceleration = 8.0;
         double maxJerk = 60.0;

         traj.compute(0.0);
         traj.getAngularData(previousOrientation, previousAngularVelocity, previousAngularAcceleration);

         double dt = 1.0e-2;
         for (double time = dt; time <= trajectoryTime; time += dt)
         {
            traj.compute(time);
            traj.getAngularData(currentOrientation, currentAngularVelocity, currentAngularAcceleration);

            currentOrientation.getQuaternion(currentQuaternion);
            previousOrientation.getQuaternion(previousQuaternion);
            if (quaternionCalculus.dot(currentQuaternion, previousQuaternion) < 0.0)
               previousQuaternion.negate();
            currentQuaternion.mulInverse(previousQuaternion);
            quaternionCalculus.log(currentQuaternion, delta);
            double velocityFD = delta.length() / dt;
            maxVelocityRecorded = Math.max(maxVelocityRecorded, velocityFD);
            boolean velocityLow = velocityFD < maxVelocity;
            if (DEBUG && !velocityLow)
            {
               System.out.println("High velocity: " + velocityFD);
               printLimitConditions(initialOrientation, initialAngularVelocity, finalOrientation, finalAngularVelocity);
            }
            assertTrue(velocityLow);
            
            previousAngularVelocity.sub(currentAngularVelocity);
            double accelerationFD = previousAngularVelocity.length() / dt;
            maxAccelerationRecorded = Math.max(maxAccelerationRecorded, accelerationFD);
            boolean accelerationLow = accelerationFD < maxAcceleration;
            if (DEBUG && !accelerationLow)
            {
               System.out.println("High acceleration: " + accelerationFD);
               printLimitConditions(initialOrientation, initialAngularVelocity, finalOrientation, finalAngularVelocity);
            }
            assertTrue(accelerationLow);
            previousAngularAcceleration.sub(currentAngularAcceleration);
            double jerkFD = previousAngularAcceleration.length() / dt;
            maxJerkRecorded = Math.max(maxJerkRecorded, jerkFD);
            boolean jerkLow = jerkFD < maxJerk;
            if (DEBUG && !jerkLow)
            {
               System.out.println("High jerk: " + jerkFD);
               printLimitConditions(initialOrientation, initialAngularVelocity, finalOrientation, finalAngularVelocity);
            }
            assertTrue(jerkLow);

            traj.getAngularData(previousOrientation, previousAngularVelocity, previousAngularAcceleration);
         }
      }

      if (DEBUG)
      {
         System.out.println("maxVelocityRecorded    : " + maxVelocityRecorded);
         System.out.println("maxAccelerationRecorded: " + maxAccelerationRecorded);
         System.out.println("maxJerkRecorded        : " + maxJerkRecorded);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.5)
   @Test(timeout = 30000)
   public void testContinuityForFastishTrajectory() throws Exception
   {
      QuaternionCalculus quaternionCalculus = new QuaternionCalculus();
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      Random random = new Random(5165165161L);
      VelocityConstrainedOrientationTrajectoryGenerator traj = new VelocityConstrainedOrientationTrajectoryGenerator("traj", worldFrame, new YoVariableRegistry("null"));
      double trajectoryTime = 1.0;

      double maxVelocityRecorded = 0.0;
      double maxAccelerationRecorded = 0.0;
      double maxJerkRecorded = 0.0;

      traj.setTrajectoryTime(trajectoryTime);

      for (int i = 0; i < 1000; i++)
      {
         FrameOrientation initialOrientation = FrameOrientation.generateRandomFrameOrientation(random, worldFrame);
         FrameVector initialAngularVelocity = FrameVector.generateRandomFrameVector(random, worldFrame, -10.0, 10.0, -10.0, 10.0, -10.0, 10.0);
         FrameOrientation finalOrientation = FrameOrientation.generateRandomFrameOrientation(random, worldFrame);
         FrameVector finalAngularVelocity = FrameVector.generateRandomFrameVector(random, worldFrame, -10.0, 10.0, -10.0, 10.0, -10.0, 10.0);

         traj.setInitialConditions(initialOrientation, initialAngularVelocity);
         traj.setFinalConditions(finalOrientation, finalAngularVelocity);
         traj.initialize();

         FrameOrientation currentOrientation = new FrameOrientation();
         FrameVector currentAngularVelocity = new FrameVector();
         FrameVector currentAngularAcceleration = new FrameVector();

         Quat4d currentQuaternion = new Quat4d();
         Quat4d previousQuaternion = new Quat4d();
         Vector3d delta = new Vector3d();

         FrameOrientation previousOrientation = new FrameOrientation();
         FrameVector previousAngularVelocity = new FrameVector();
         FrameVector previousAngularAcceleration = new FrameVector();

         
         double maxVelocity = 40.0;
         double maxAcceleration = 700.0;
         double maxJerk = 100000.0;

         traj.compute(0.0);
         traj.getAngularData(previousOrientation, previousAngularVelocity, previousAngularAcceleration);

         double dt = 1.0e-2;
         for (double time = dt; time <= trajectoryTime; time += dt)
         {
            traj.compute(time);
            traj.getAngularData(currentOrientation, currentAngularVelocity, currentAngularAcceleration);

            currentOrientation.getQuaternion(currentQuaternion);
            previousOrientation.getQuaternion(previousQuaternion);
            if (quaternionCalculus.dot(currentQuaternion, previousQuaternion) < 0.0)
               previousQuaternion.negate();
            currentQuaternion.mulInverse(previousQuaternion);
            quaternionCalculus.log(currentQuaternion, delta);
            double velocityFD = delta.length() / dt;
            maxVelocityRecorded = Math.max(maxVelocityRecorded, velocityFD);
            boolean velocityLow = velocityFD < maxVelocity;
            if (DEBUG && !velocityLow)
            {
               System.out.println("High velocity: " + velocityFD);
               printLimitConditions(initialOrientation, initialAngularVelocity, finalOrientation, finalAngularVelocity);
            }
            assertTrue(velocityLow);
            
            previousAngularVelocity.sub(currentAngularVelocity);
            double accelerationFD = previousAngularVelocity.length() / dt;
            maxAccelerationRecorded = Math.max(maxAccelerationRecorded, accelerationFD);
            boolean accelerationLow = accelerationFD < maxAcceleration;
            if (DEBUG && !accelerationLow)
            {
               System.out.println("High acceleration: " + accelerationFD);
               printLimitConditions(initialOrientation, initialAngularVelocity, finalOrientation, finalAngularVelocity);
            }
            assertTrue(accelerationLow);

            previousAngularAcceleration.sub(currentAngularAcceleration);
            double jerkFD = previousAngularAcceleration.length() / dt;
            maxJerkRecorded = Math.max(maxJerkRecorded, jerkFD);
            boolean jerkLow = jerkFD < maxJerk;
            if (DEBUG && !jerkLow)
            {
               System.out.println("High jerk: " + jerkFD);
               printLimitConditions(initialOrientation, initialAngularVelocity, finalOrientation, finalAngularVelocity);
            }
            assertTrue(jerkLow);

            traj.getAngularData(previousOrientation, previousAngularVelocity, previousAngularAcceleration);
         }
      }

      if (DEBUG)
      {
         System.out.println("maxVelocityRecorded    : " + maxVelocityRecorded);
         System.out.println("maxAccelerationRecorded: " + maxAccelerationRecorded);
         System.out.println("maxJerkRecorded        : " + maxJerkRecorded);
      }
   }

   private void printLimitConditions(FrameOrientation initialOrientation, FrameVector initialAngularVelocity, FrameOrientation finalOrientation, FrameVector finalAngularVelocity)
   {
      System.out.println("FrameOrientation initialOrientation" + toStringFrameOrientationForVizualizer(initialOrientation));
      System.out.println("FrameVector initialAngularVelocity" + toStringFrameVectorForVizualizer(initialAngularVelocity));
      System.out.println("FrameOrientation finalOrientation" + toStringFrameOrientationForVizualizer(finalOrientation));
      System.out.println("FrameVector finalAngularVelocity" + toStringFrameVectorForVizualizer(finalAngularVelocity));
   }

   private String toStringFrameOrientationForVizualizer(FrameOrientation frameOrientation)
   {
      double qx = frameOrientation.getQuaternionCopy().getX();
      double qy = frameOrientation.getQuaternionCopy().getY();
      double qz = frameOrientation.getQuaternionCopy().getZ();
      double qs = frameOrientation.getQuaternionCopy().getW();
      return " = new FrameOrientation(worldFrame, " + qx + ", " + qy + ", " + qz + ", " + qs + ");";
   }

   private String toStringFrameVectorForVizualizer(FrameVector frameVector)
   {
      double x = frameVector.getX();
      double y = frameVector.getY();
      double z = frameVector.getZ();
      return " = new FrameVector(worldFrame, " + x + ", " + y + ", " + z + ");";
   }
}
