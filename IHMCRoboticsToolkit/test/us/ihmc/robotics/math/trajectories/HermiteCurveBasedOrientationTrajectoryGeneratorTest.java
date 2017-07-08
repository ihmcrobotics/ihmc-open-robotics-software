package us.ihmc.robotics.math.trajectories;

import static org.junit.Assert.*;

import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector3D;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.math.QuaternionCalculus;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class HermiteCurveBasedOrientationTrajectoryGeneratorTest
{
   private static final boolean DEBUG = false;

   @ContinuousIntegrationTest(estimatedDuration = 0.9)
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
      FrameVector3D currentAngularVelocity = new FrameVector3D();
      FrameVector3D currentAngularAcceleration = new FrameVector3D();

      FrameOrientation orientationFromIntegration = new FrameOrientation();
      Vector3D angularVelocityVector = new Vector3D();
      Quaternion quaternionFromIntegration = new Quaternion();
      Quaternion integratedAngularVelocity = new Quaternion();

      FrameVector3D angularVelocityFromIntegration = new FrameVector3D();
      Vector3D integratedAngularAcceleration = new Vector3D();

      HermiteCurveBasedOrientationTrajectoryGenerator traj = new HermiteCurveBasedOrientationTrajectoryGenerator("traj", worldFrame,
                                                                                                                 new YoVariableRegistry("null"));
      traj.setTrajectoryTime(trajectoryTime);

      for (int i = 0; i < 5; i++)
      {
         FrameOrientation initialOrientation = FrameOrientation.generateRandomFrameOrientation(random, worldFrame);
         FrameVector3D initialAngularVelocity = FrameVector3D.generateRandomFrameVector(random, worldFrame, -10.0, 10.0, -10.0, 10.0, -10.0, 10.0);
         FrameOrientation finalOrientation = FrameOrientation.generateRandomFrameOrientation(random, worldFrame);
         FrameVector3D finalAngularVelocity = FrameVector3D.generateRandomFrameVector(random, worldFrame, -10.0, 10.0, -10.0, 10.0, -10.0, 10.0);

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
            quaternionFromIntegration.multiply(integratedAngularVelocity, quaternionFromIntegration);
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
      HermiteCurveBasedOrientationTrajectoryGenerator traj = new HermiteCurveBasedOrientationTrajectoryGenerator("traj", worldFrame,
                                                                                                                 new YoVariableRegistry("null"));
      double trajectoryTime = 5.0;
      traj.setTrajectoryTime(trajectoryTime);

      for (int i = 0; i < 10000; i++)
      {
         FrameOrientation initialOrientation = FrameOrientation.generateRandomFrameOrientation(random, worldFrame);
         FrameVector3D initialAngularVelocity = FrameVector3D.generateRandomFrameVector(random, worldFrame, -5.0, 5, -5, 5, -5, 5);
         FrameOrientation finalOrientation = FrameOrientation.generateRandomFrameOrientation(random, worldFrame);
         FrameVector3D finalAngularVelocity = FrameVector3D.generateRandomFrameVector(random, worldFrame, -5, 5, -5, 5, -5, 5);

         FrameVector3D zeroAngularAcceleration = new FrameVector3D();

         traj.setInitialConditions(initialOrientation, initialAngularVelocity);
         traj.setFinalConditions(finalOrientation, finalAngularVelocity);
         traj.initialize();

         FrameOrientation currentOrientation = new FrameOrientation();
         FrameVector3D currentAngularVelocity = new FrameVector3D();
         FrameVector3D currentAngularAcceleration = new FrameVector3D();

         double dt = 1.0e-8;
         traj.compute(0.0 + dt);
         traj.getAngularData(currentOrientation, currentAngularVelocity, currentAngularAcceleration);

         double orientationEpsilon = 5.0e-4;
         double velocityEpsilon = 5.0e-4;
         double accelerationEpsilon = 5.0e-4;

         boolean goodInitialOrientation = initialOrientation.epsilonEquals(currentOrientation, orientationEpsilon);
         boolean goodInitialVelocity = initialAngularVelocity.epsilonEquals(currentAngularVelocity, velocityEpsilon);
         boolean goodInitialAngularAcceleration = zeroAngularAcceleration.epsilonEquals(currentAngularAcceleration, accelerationEpsilon);

         if (DEBUG)
         {
            if (!goodInitialVelocity)
            {
               FrameVector3D error = new FrameVector3D();
               error.sub(initialAngularVelocity, currentAngularVelocity);
               System.out.println("Bad initial velocity, error: " + error);
               printLimitConditions(initialOrientation, initialAngularVelocity, finalOrientation, finalAngularVelocity);
            }

            if (!goodInitialAngularAcceleration)
            {
               FrameVector3D error = new FrameVector3D();
               error.sub(zeroAngularAcceleration, currentAngularAcceleration);
               System.out.println("Bad initial acceleration, error: " + error);
               printLimitConditions(initialOrientation, zeroAngularAcceleration, finalOrientation, zeroAngularAcceleration);
            }
         }

         //                  assertTrue(goodInitialAngularAcceleration);

         traj.compute(trajectoryTime - dt);
         traj.getAngularData(currentOrientation, currentAngularVelocity, currentAngularAcceleration);

         boolean goodFinalOrientation = finalOrientation.epsilonEquals(currentOrientation, orientationEpsilon);
         boolean goodFinalAngularVelocity = finalAngularVelocity.epsilonEquals(currentAngularVelocity, velocityEpsilon);
         boolean goodFinalAngularAcceleration = zeroAngularAcceleration.epsilonEquals(currentAngularAcceleration, accelerationEpsilon);

         if (DEBUG)
         {
            if (!goodFinalAngularVelocity)
            {
               FrameVector3D error = new FrameVector3D();
               error.sub(finalAngularVelocity, currentAngularVelocity);
               System.out.println("Bad final velocity, error: " + error);
               System.out.println("final X angular velocity " + currentAngularVelocity.getX());
               System.out.println("final Y angular velocity " + currentAngularVelocity.getY());
               System.out.println("final Z angular velocity " + currentAngularVelocity.getZ());

               printLimitConditions(initialOrientation, initialAngularVelocity, finalOrientation, finalAngularVelocity);
            }
            if (!goodFinalAngularAcceleration)
            {
               System.out.println("Bad final acceleration: " + currentAngularAcceleration);
               printLimitConditions(initialOrientation, initialAngularVelocity, finalOrientation, finalAngularVelocity);
            }
         }
         //         assertTrue(goodFinalAngularAcceleration);
         if (!(goodInitialOrientation && goodFinalOrientation && goodInitialVelocity && goodFinalAngularVelocity))
         {
            printLimitConditions(initialOrientation, initialAngularVelocity, finalOrientation, finalAngularVelocity);
         }

         assertTrue(goodInitialOrientation);
         assertTrue(goodFinalOrientation);
         assertTrue(goodInitialVelocity);
         assertTrue(goodFinalAngularVelocity);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testContinuityForSlowTrajectory() throws Exception
   {
      QuaternionCalculus quaternionCalculus = new QuaternionCalculus();
      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      Random random = new Random(5165165161L);
      HermiteCurveBasedOrientationTrajectoryGenerator traj = new HermiteCurveBasedOrientationTrajectoryGenerator("traj", worldFrame,
                                                                                                                 new YoVariableRegistry("null"));
      double trajectoryTime = 10.0;

      double maxVelocityRecorded = 0.0;
      double maxAccelerationRecorded = 0.0;
      double maxJerkRecorded = 0.0;

      traj.setTrajectoryTime(trajectoryTime);

      for (int i = 0; i < 100; i++)
      {
         FrameOrientation initialOrientation = FrameOrientation.generateRandomFrameOrientation(random, worldFrame);
         FrameVector3D initialAngularVelocity = FrameVector3D.generateRandomFrameVector(random, worldFrame);
         FrameOrientation finalOrientation = FrameOrientation.generateRandomFrameOrientation(random, worldFrame);
         FrameVector3D finalAngularVelocity = FrameVector3D.generateRandomFrameVector(random, worldFrame);

         traj.setInitialConditions(initialOrientation, initialAngularVelocity);
         traj.setFinalConditions(finalOrientation, finalAngularVelocity);
         traj.initialize();

         FrameOrientation currentOrientation = new FrameOrientation();
         FrameVector3D currentAngularVelocity = new FrameVector3D();
         FrameVector3D currentAngularAcceleration = new FrameVector3D();

         Quaternion currentQuaternion = new Quaternion();
         Quaternion previousQuaternion = new Quaternion();
         Vector3D delta = new Vector3D();

         FrameOrientation previousOrientation = new FrameOrientation();
         FrameVector3D previousAngularVelocity = new FrameVector3D();
         FrameVector3D previousAngularAcceleration = new FrameVector3D();

         double maxVelocity = 2.0;
         double maxAcceleration = 1.0;
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
            if (currentQuaternion.dot(previousQuaternion) < 0.0)
               previousQuaternion.negate();
            currentQuaternion.multiplyConjugateOther(previousQuaternion);
            quaternionCalculus.log(currentQuaternion, delta);
            double velocityFD = delta.length() / dt;
            maxVelocityRecorded = Math.max(maxVelocityRecorded, velocityFD);
            boolean velocityLow = velocityFD < maxVelocity;

            previousAngularVelocity.sub(currentAngularVelocity);
            double accelerationFD = previousAngularVelocity.length() / dt;
            maxAccelerationRecorded = Math.max(maxAccelerationRecorded, accelerationFD);
            boolean accelerationLow = accelerationFD < maxAcceleration;

            previousAngularAcceleration.sub(currentAngularAcceleration);
            double jerkFD = previousAngularAcceleration.length() / dt;
            maxJerkRecorded = Math.max(maxJerkRecorded, jerkFD);
            boolean jerkLow = jerkFD < maxJerk;

            if (DEBUG)
            {
               if (!velocityLow)
               {
                  System.out.println("High velocity: " + velocityFD);
                  printLimitConditions(initialOrientation, initialAngularVelocity, finalOrientation, finalAngularVelocity);
               }
               if (!accelerationLow)
               {
                  System.out.println("High acceleration: " + accelerationFD);
                  printLimitConditions(initialOrientation, initialAngularVelocity, finalOrientation, finalAngularVelocity);
               }
               if (!jerkLow)
               {
                  System.out.println("High jerk: " + jerkFD);
                  printLimitConditions(initialOrientation, initialAngularVelocity, finalOrientation, finalAngularVelocity);
               }
            }
            assertTrue(velocityLow);
            assertTrue(accelerationLow);
            //            assertTrue(jerkLow);

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
      HermiteCurveBasedOrientationTrajectoryGenerator traj = new HermiteCurveBasedOrientationTrajectoryGenerator("traj", worldFrame,
                                                                                                                 new YoVariableRegistry("null"));
      double trajectoryTime = 2.0;

      double maxVelocityRecorded = 0.0;
      double maxAccelerationRecorded = 0.0;
      double maxJerkRecorded = 0.0;

      traj.setTrajectoryTime(trajectoryTime);

      for (int i = 0; i < 1000; i++)
      {
         FrameOrientation initialOrientation = FrameOrientation.generateRandomFrameOrientation(random, worldFrame);
         FrameVector3D initialAngularVelocity = FrameVector3D.generateRandomFrameVector(random, worldFrame, -10.0, 10.0, -10.0, 10.0, -10.0, 10.0);
         FrameOrientation finalOrientation = FrameOrientation.generateRandomFrameOrientation(random, worldFrame);
         FrameVector3D finalAngularVelocity = FrameVector3D.generateRandomFrameVector(random, worldFrame, -10.0, 10.0, -10.0, 10.0, -10.0, 10.0);

         traj.setInitialConditions(initialOrientation, initialAngularVelocity);
         traj.setFinalConditions(finalOrientation, finalAngularVelocity);
         traj.initialize();

         FrameOrientation currentOrientation = new FrameOrientation();
         FrameVector3D currentAngularVelocity = new FrameVector3D();
         FrameVector3D currentAngularAcceleration = new FrameVector3D();

         Quaternion currentQuaternion = new Quaternion();
         Quaternion previousQuaternion = new Quaternion();
         Vector3D delta = new Vector3D();

         FrameOrientation previousOrientation = new FrameOrientation();
         FrameVector3D previousAngularVelocity = new FrameVector3D();
         FrameVector3D previousAngularAcceleration = new FrameVector3D();

         double maxVelocity = 20.0;
         double maxAcceleration = 50.0;
         double maxJerk = 3000.0;

         traj.compute(0.0);
         traj.getAngularData(previousOrientation, previousAngularVelocity, previousAngularAcceleration);

         double dt = 1.0e-2;
         for (double time = dt; time <= trajectoryTime; time += dt)
         {
            traj.compute(time);
            traj.getAngularData(currentOrientation, currentAngularVelocity, currentAngularAcceleration);

            currentOrientation.getQuaternion(currentQuaternion);
            previousOrientation.getQuaternion(previousQuaternion);
            if (currentQuaternion.dot(previousQuaternion) < 0.0)
               previousQuaternion.negate();
            currentQuaternion.multiplyConjugateOther(previousQuaternion);
            quaternionCalculus.log(currentQuaternion, delta);
            double velocityFD = delta.length() / dt;
            maxVelocityRecorded = Math.max(maxVelocityRecorded, velocityFD);
            boolean velocityLow = velocityFD < maxVelocity;

            previousAngularVelocity.sub(currentAngularVelocity);
            double accelerationFD = previousAngularVelocity.length() / dt;
            maxAccelerationRecorded = Math.max(maxAccelerationRecorded, accelerationFD);
            boolean accelerationLow = accelerationFD < maxAcceleration;

            if (DEBUG)
            {
               if (!velocityLow)
               {
                  System.out.println("High velocity: " + velocityFD);
                  printLimitConditions(initialOrientation, initialAngularVelocity, finalOrientation, finalAngularVelocity);
               }

               if (!accelerationLow)
               {
                  System.out.println("High acceleration: " + accelerationFD);
                  printLimitConditions(initialOrientation, initialAngularVelocity, finalOrientation, finalAngularVelocity);
               }
            }
            assertTrue(velocityLow);
            assertTrue(accelerationLow);

            previousAngularAcceleration.sub(currentAngularAcceleration);
            double jerkFD = previousAngularAcceleration.length() / dt;
            maxJerkRecorded = Math.max(maxJerkRecorded, jerkFD);
            boolean jerkLow = jerkFD < maxJerk;

            if (DEBUG)
            {
               if (!jerkLow)
               {
                  System.out.println("High jerk: " + jerkFD);
                  printLimitConditions(initialOrientation, initialAngularVelocity, finalOrientation, finalAngularVelocity);
               }
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

   private void printLimitConditions(FrameOrientation initialOrientation, FrameVector3D initialAngularVelocity, FrameOrientation finalOrientation,
                                     FrameVector3D finalAngularVelocity)
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
      double qs = frameOrientation.getQuaternionCopy().getS();
      return " = new FrameOrientation(worldFrame, " + qx + ", " + qy + ", " + qz + ", " + qs + ");";
   }

   private String toStringFrameVectorForVizualizer(FrameVector3D frameVector)
   {
      double x = frameVector.getX();
      double y = frameVector.getY();
      double z = frameVector.getZ();
      return " = new FrameVector(worldFrame, " + x + ", " + y + ", " + z + ");";
   }
}
