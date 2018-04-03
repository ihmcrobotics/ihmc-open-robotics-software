package us.ihmc.robotics.math.trajectories;

import static org.junit.Assert.assertTrue;

import java.util.Random;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.math.QuaternionCalculus;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

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

      FrameQuaternion currentOrientation = new FrameQuaternion();
      FrameVector3D currentAngularVelocity = new FrameVector3D();
      FrameVector3D currentAngularAcceleration = new FrameVector3D();

      FrameQuaternion orientationFromIntegration = new FrameQuaternion();
      Vector3D angularVelocityVector = new Vector3D();
      Quaternion quaternionFromIntegration = new Quaternion();
      Quaternion integratedAngularVelocity = new Quaternion();

      FrameVector3D angularVelocityFromIntegration = new FrameVector3D();
      Vector3D integratedAngularAcceleration = new Vector3D();

      VelocityConstrainedOrientationTrajectoryGenerator traj = new VelocityConstrainedOrientationTrajectoryGenerator("traj", worldFrame, new YoVariableRegistry("null"));
      traj.setTrajectoryTime(trajectoryTime);

      for (int i = 0; i < 5; i++)
      {
         FrameQuaternion initialOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);
         FrameVector3D initialAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, worldFrame, -10.0, 10.0, -10.0, 10.0, -10.0, 10.0);
         FrameQuaternion finalOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);
         FrameVector3D finalAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, worldFrame, -10.0, 10.0, -10.0, 10.0, -10.0, 10.0);

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

            angularVelocityVector.set(currentAngularVelocity);
            RotationTools.integrateAngularVelocity(angularVelocityVector, dt, integratedAngularVelocity);
            quaternionFromIntegration.set(orientationFromIntegration);
            quaternionFromIntegration.multiply(integratedAngularVelocity, quaternionFromIntegration);
            orientationFromIntegration.set(quaternionFromIntegration);

            integratedAngularAcceleration.set(currentAngularAcceleration);
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
         FrameQuaternion initialOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);
         FrameVector3D initialAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, worldFrame, -10.0, 10.0, -10.0, 10.0, -10.0, 10.0);
         FrameQuaternion finalOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);
         FrameVector3D finalAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, worldFrame, -10.0, 10.0, -10.0, 10.0, -10.0, 10.0);

         FrameVector3D zeroAngularAcceleration = new FrameVector3D();

         traj.setInitialConditions(initialOrientation, initialAngularVelocity);
         traj.setFinalConditions(finalOrientation, finalAngularVelocity);
         traj.initialize();

         FrameQuaternion currentOrientation = new FrameQuaternion();
         FrameVector3D currentAngularVelocity = new FrameVector3D();
         FrameVector3D currentAngularAcceleration = new FrameVector3D();

         double dt = 1.0e-8;
         traj.compute(0.0 + dt);
         traj.getAngularData(currentOrientation, currentAngularVelocity, currentAngularAcceleration);

         double epsilon = 5.0e-5;
         double accelerationEpsilon = 5.0e-4;
         assertTrue(initialOrientation.epsilonEquals(currentOrientation, epsilon));
         boolean goodInitialVelocity = initialAngularVelocity.epsilonEquals(currentAngularVelocity, epsilon);
         if (DEBUG && !goodInitialVelocity)
         {
            FrameVector3D error = new FrameVector3D();
            error.sub(initialAngularVelocity, currentAngularVelocity);
            System.out.println("Bad initial velocity, error: " + error);
            printLimitConditions(initialOrientation, initialAngularVelocity, finalOrientation, finalAngularVelocity);
         }
         assertTrue(goodInitialVelocity);
         boolean goodInitialAngularAcceleration = zeroAngularAcceleration.epsilonEquals(currentAngularAcceleration, accelerationEpsilon);
         assertTrue(goodInitialAngularAcceleration);

         traj.compute(trajectoryTime - dt);
         traj.getAngularData(currentOrientation, currentAngularVelocity, currentAngularAcceleration);

         assertTrue(finalOrientation.geometricallyEquals(currentOrientation, epsilon));
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
         FrameQuaternion initialOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);
         FrameVector3D initialAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, worldFrame);
         FrameQuaternion finalOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);
         FrameVector3D finalAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, worldFrame);

         traj.setInitialConditions(initialOrientation, initialAngularVelocity);
         traj.setFinalConditions(finalOrientation, finalAngularVelocity);
         traj.initialize();

         FrameQuaternion currentOrientation = new FrameQuaternion();
         FrameVector3D currentAngularVelocity = new FrameVector3D();
         FrameVector3D currentAngularAcceleration = new FrameVector3D();

         Quaternion currentQuaternion = new Quaternion();
         Quaternion previousQuaternion = new Quaternion();
         Vector3D delta = new Vector3D();

         FrameQuaternion previousOrientation = new FrameQuaternion();
         FrameVector3D previousAngularVelocity = new FrameVector3D();
         FrameVector3D previousAngularAcceleration = new FrameVector3D();

         
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

            currentQuaternion.set(currentOrientation);
            previousQuaternion.set(previousOrientation);
            if (currentQuaternion.dot(previousQuaternion) < 0.0)
               previousQuaternion.negate();
            currentQuaternion.multiplyConjugateOther(previousQuaternion);
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
         FrameQuaternion initialOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);
         FrameVector3D initialAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, worldFrame, -10.0, 10.0, -10.0, 10.0, -10.0, 10.0);
         FrameQuaternion finalOrientation = EuclidFrameRandomTools.nextFrameQuaternion(random, worldFrame);
         FrameVector3D finalAngularVelocity = EuclidFrameRandomTools.nextFrameVector3D(random, worldFrame, -10.0, 10.0, -10.0, 10.0, -10.0, 10.0);

         traj.setInitialConditions(initialOrientation, initialAngularVelocity);
         traj.setFinalConditions(finalOrientation, finalAngularVelocity);
         traj.initialize();

         FrameQuaternion currentOrientation = new FrameQuaternion();
         FrameVector3D currentAngularVelocity = new FrameVector3D();
         FrameVector3D currentAngularAcceleration = new FrameVector3D();

         Quaternion currentQuaternion = new Quaternion();
         Quaternion previousQuaternion = new Quaternion();
         Vector3D delta = new Vector3D();

         FrameQuaternion previousOrientation = new FrameQuaternion();
         FrameVector3D previousAngularVelocity = new FrameVector3D();
         FrameVector3D previousAngularAcceleration = new FrameVector3D();

         
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

            currentQuaternion.set(currentOrientation);
            previousQuaternion.set(previousOrientation);
            if (currentQuaternion.dot(previousQuaternion) < 0.0)
               previousQuaternion.negate();
            currentQuaternion.multiplyConjugateOther(previousQuaternion);
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

   private void printLimitConditions(FrameQuaternion initialOrientation, FrameVector3D initialAngularVelocity, FrameQuaternion finalOrientation, FrameVector3D finalAngularVelocity)
   {
      System.out.println("FrameOrientation initialOrientation" + toStringFrameOrientationForVizualizer(initialOrientation));
      System.out.println("FrameVector initialAngularVelocity" + toStringFrameVectorForVizualizer(initialAngularVelocity));
      System.out.println("FrameOrientation finalOrientation" + toStringFrameOrientationForVizualizer(finalOrientation));
      System.out.println("FrameVector finalAngularVelocity" + toStringFrameVectorForVizualizer(finalAngularVelocity));
   }

   private String toStringFrameOrientationForVizualizer(FrameQuaternion frameOrientation)
   {
      double qx = frameOrientation.getX();
      double qy = frameOrientation.getY();
      double qz = frameOrientation.getZ();
      double qs = frameOrientation.getS();
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
