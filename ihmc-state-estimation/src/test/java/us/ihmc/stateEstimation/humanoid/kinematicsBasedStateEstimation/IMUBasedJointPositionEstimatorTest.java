package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import static us.ihmc.robotics.Assert.assertEquals;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.IMUBasedJointPositionEstimator.IMUBasedJointPositionEstimatorMode;
import us.ihmc.yoVariables.registry.YoRegistry;

class IMUBasedJointPositionEstimatorTest
{
   @Test
   void testIMUBasedJointPositionEstimatorAllZero()
   {
      double[] parentYawPitchRoll = new double[] {0.0, 0.0, 0.0};
      double[] jointYawRollPitch = new double[] {0.0, 0.0, 0.0};
      double parentIMUHeadingError = 0.0;
      double childIMUHeadingError = 0.0;

      testOne(parentYawPitchRoll, jointYawRollPitch, parentIMUHeadingError, childIMUHeadingError);
   }

   @Test
   void testIMUBasedJointPositionEstimatorParentPitchedAndJointJustYawed()
   {
      double parentYaw = 0.0;
      double parentRoll = 0.0;
      double parentPitch = 0.3;

      double jointYaw = 0.1;
      double jointRoll = 0.0;
      double jointPitch = 0.0;

      double[] parentYawPitchRoll = new double[] {parentYaw, parentPitch, parentRoll};
      double[] jointYawRollPitch = new double[] {jointYaw, jointRoll, jointPitch};
      double parentIMUHeadingError = 0.0;
      double childIMUHeadingError = 0.0;

      testOne(parentYawPitchRoll, jointYawRollPitch, parentIMUHeadingError, childIMUHeadingError);
   }

   @Test
   void testIMUBasedJointPositionEstimatorParentAtWorld()
   {
      double parentYaw = 0.0;
      double parentRoll = 0.0;
      double parentPitch = 0.0;

      double jointYaw = 0.37;
      double jointRoll = 0.17;
      double jointPitch = 0.23;

      double[] parentYawPitchRoll = new double[] {parentYaw, parentPitch, parentRoll};
      double[] jointYawRollPitch = new double[] {jointYaw, jointRoll, jointPitch};
      double parentIMUHeadingError = 0.0;
      double childIMUHeadingError = 0.0;

      testOne(parentYawPitchRoll, jointYawRollPitch, parentIMUHeadingError, childIMUHeadingError);
   }

   @Test
   void testIMUBasedJointPositionEstimatorJointsAtZero()
   {
      double parentYaw = 0.12;
      double parentRoll = 0.56;
      double parentPitch = 0.34;

      double jointYaw = 0.0;
      double jointRoll = 0.0;
      double jointPitch = 0.0;

      double[] parentYawPitchRoll = new double[] {parentYaw, parentPitch, parentRoll};
      double[] jointYawRollPitch = new double[] {jointYaw, jointRoll, jointPitch};
      double parentIMUHeadingError = 0.0;
      double childIMUHeadingError = 0.0;

      testOne(parentYawPitchRoll, jointYawRollPitch, parentIMUHeadingError, childIMUHeadingError);
   }

   @Test
   void testIMUBasedJointPositionEstimatorParentAtWorldNoJointYaw()
   {
      double parentYaw = 0.3;
      double parentRoll = -0.05;
      double parentPitch = 0.3;

      double jointYaw = 0.0;
      double jointRoll = 0.17;
      double jointPitch = 0.23;

      double[] parentYawPitchRoll = new double[] {parentYaw, parentPitch, parentRoll};
      double[] jointYawRollPitch = new double[] {jointYaw, jointRoll, jointPitch};
      double parentIMUHeadingError = -0.25;
      double childIMUHeadingError = 0.31;

      testOne(parentYawPitchRoll, jointYawRollPitch, parentIMUHeadingError, childIMUHeadingError);
   }

   @Test
   void testIMUBasedJointPositionEstimatorNoCompassError()
   {
      double parentYaw = 0.12;
      double parentRoll = 0.56;
      double parentPitch = 0.34;

      double jointYaw = 0.179;
      double jointRoll = -0.456;
      double jointPitch = -0.77;

      double[] parentYawPitchRoll = new double[] {parentYaw, parentPitch, parentRoll};
      double[] jointYawRollPitch = new double[] {jointYaw, jointRoll, jointPitch};
      double parentIMUHeadingError = 0.0;
      double childIMUHeadingError = 0.0;

      testOne(parentYawPitchRoll, jointYawRollPitch, parentIMUHeadingError, childIMUHeadingError);
   }

   @Test
   void testIMUBasedJointPositionEstimatorRandomCasesNoCompassError()
   {
      int numberOfTests = 10000;
      Random random = new Random(1776L);

      double min = -0.2 * Math.PI; //-0.49 * Math.PI;
      double max = 0.2 * Math.PI; //0.49 * Math.PI;

      for (int i = 0; i < numberOfTests; i++)
      {
         double[] parentYawPitchRoll = new double[] {EuclidCoreRandomTools.nextDouble(random, min, max), EuclidCoreRandomTools.nextDouble(random, min, max),
               EuclidCoreRandomTools.nextDouble(random, min, max)};

         double[] jointYawRollPitch = new double[] {EuclidCoreRandomTools.nextDouble(random, min, max), EuclidCoreRandomTools.nextDouble(random, min, max),
               EuclidCoreRandomTools.nextDouble(random, min, max)};

         testOne(parentYawPitchRoll, jointYawRollPitch, 0.0, 0.0);
      }
   }

   @Test
   void testIMUBasedJointPositionEstimatorJointsAtZeroWithChildCompassError()
   {
      double parentYaw = 0.12;
      double parentRoll = 0.56;
      double parentPitch = 0.34;

      double jointYaw = 0.0;
      double jointRoll = 0.0;
      double jointPitch = 0.0;

      double[] parentYawPitchRoll = new double[] {parentYaw, parentPitch, parentRoll};
      double[] jointYawRollPitch = new double[] {jointYaw, jointRoll, jointPitch};
      double parentIMUHeadingError = 0.0;
      double childIMUHeadingError = -0.2;

      testOne(parentYawPitchRoll, jointYawRollPitch, parentIMUHeadingError, childIMUHeadingError);
   }

   @Test
   void testIMUBasedJointPositionEstimatorWithParentCompassHeadingError()
   {
      double parentYaw = 0.12;
      double parentRoll = 0.3;
      double parentPitch = 0.34;

      double jointYaw = 0.02;
      double jointRoll = 0.18;
      double jointPitch = 0.04;

      double[] parentYawPitchRoll = new double[] {parentYaw, parentPitch, parentRoll};
      double[] jointYawRollPitch = new double[] {jointYaw, jointRoll, jointPitch};
      double parentIMUHeadingError = 0.2;
      double childIMUHeadingError = 0.0;

      testOne(parentYawPitchRoll, jointYawRollPitch, parentIMUHeadingError, childIMUHeadingError);
   }

   @Test
   void testIMUBasedJointPositionEstimatorWithChildCompassHeadingError()
   {
      double parentYaw = 0.12;
      double parentRoll = 0.2;
      double parentPitch = 0.3;

      double jointYaw = 0.5;
      double jointRoll = 0.2;
      double jointPitch = 0.33;

      double[] parentYawPitchRoll = new double[] {parentYaw, parentPitch, parentRoll};
      double[] jointYawRollPitch = new double[] {jointYaw, jointRoll, jointPitch};
      double parentIMUHeadingError = 0.0;
      double childIMUHeadingError = 0.8;

      testOne(parentYawPitchRoll, jointYawRollPitch, parentIMUHeadingError, childIMUHeadingError);
   }

   @Test
   void testIMUBasedJointPositionEstimatorWithChildAndParentCompassHeadingError()
   {
      double parentYaw = 0.12;
      double parentRoll = 0.2;
      double parentPitch = 0.3;

      double jointYaw = 0.5;
      double jointRoll = 0.2;
      double jointPitch = 0.33;

      double[] parentYawPitchRoll = new double[] {parentYaw, parentPitch, parentRoll};
      double[] jointYawRollPitch = new double[] {jointYaw, jointRoll, jointPitch};
      double parentIMUHeadingError = -0.6;
      double childIMUHeadingError = 0.8;

      testOne(parentYawPitchRoll, jointYawRollPitch, parentIMUHeadingError, childIMUHeadingError);
   }

   @Test
   void testIMUBasedJointPositionEstimatorRandomCasesWithCompassError()
   {
      int numberOfTests = 10000;
      Random random = new Random(1776L);

      double min = -0.2 * Math.PI; //-0.49 * Math.PI;
      double max = 0.2 * Math.PI; //0.49 * Math.PI;

      for (int i = 0; i < numberOfTests; i++)
      {
         double[] parentYawPitchRoll = new double[] {EuclidCoreRandomTools.nextDouble(random, min, max), EuclidCoreRandomTools.nextDouble(random, min, max),
               EuclidCoreRandomTools.nextDouble(random, min, max)};

         double[] jointYawRollPitch = new double[] {EuclidCoreRandomTools.nextDouble(random, min, max), EuclidCoreRandomTools.nextDouble(random, min, max),
               EuclidCoreRandomTools.nextDouble(random, min, max)};

         double parentCompassError = EuclidCoreRandomTools.nextDouble(random, min, max);
         double childCompassError = EuclidCoreRandomTools.nextDouble(random, min, max);

         testOne(parentYawPitchRoll, jointYawRollPitch, parentCompassError, childCompassError);
      }
   }

   private void testOne(double[] parentYawPitchRoll, double[] jointYawRollPitch, double parentIMUHeadingError, double childIMUHeadingError)
   {
      YoRegistry registry = new YoRegistry("test");
      IMUBasedJointPositionEstimator estimator = new IMUBasedJointPositionEstimator("test", registry);

      IMUBasedJointPositionEstimatorMode[] modes = IMUBasedJointPositionEstimatorMode.values();
      for (IMUBasedJointPositionEstimatorMode mode : modes)
      {
         estimator.setIMUBasedJointPositionEstimatorMode(mode);
         Quaternion parentIMUToWorld = new Quaternion();
         Quaternion parentIMUToWorldWithHeadingError = new Quaternion();
         Quaternion childIMUToWorld = new Quaternion();
         Quaternion childIMUToWorldWithHeadingError = new Quaternion();

         Quaternion jointYawQuaternion = new Quaternion();
         Quaternion jointRollQuaternion = new Quaternion();
         Quaternion jointPitchQuaternion = new Quaternion();

         parentIMUToWorld.setYawPitchRoll(parentYawPitchRoll[0], parentYawPitchRoll[1], parentYawPitchRoll[2]);

         Quaternion parentIMUHeadingErrorQuaternion = new Quaternion();
         parentIMUHeadingErrorQuaternion.setYawPitchRoll(parentIMUHeadingError, 0.0, 0.0);
         parentIMUToWorldWithHeadingError.set(parentIMUHeadingErrorQuaternion);
         parentIMUToWorldWithHeadingError.multiply(parentIMUToWorld);

         jointYawQuaternion.setYawPitchRoll(jointYawRollPitch[0], 0.0, 0.0);
         jointRollQuaternion.setYawPitchRoll(0.0, 0.0, jointYawRollPitch[1]);
         jointPitchQuaternion.setYawPitchRoll(0.0, jointYawRollPitch[2], 0.0);

         childIMUToWorld.set(parentIMUToWorld);
         childIMUToWorld.multiply(jointYawQuaternion);
         childIMUToWorld.multiply(jointRollQuaternion);
         childIMUToWorld.multiply(jointPitchQuaternion);

         Quaternion childIMUHeadingErrorQuaternion = new Quaternion();
         childIMUHeadingErrorQuaternion.setYawPitchRoll(childIMUHeadingError, 0.0, 0.0);

         childIMUToWorldWithHeadingError.set(childIMUHeadingErrorQuaternion);
         childIMUToWorldWithHeadingError.multiply(childIMUToWorld);

         estimator.compute(jointYawRollPitch[0], parentIMUToWorldWithHeadingError, childIMUToWorldWithHeadingError);

         double yawSolutionShouldBeZero = estimator.getYawSolutionShouldBeZero();
         double estimatedRoll = estimator.getEstimatedJointRoll();
         double estimatedPitch = estimator.getEstimatedJointPitch();

         assertEquals(0.0, yawSolutionShouldBeZero, 1e-7);
         assertEquals(jointYawRollPitch[1], estimatedRoll, 1e-7);
         assertEquals(jointYawRollPitch[2], estimatedPitch, 1e-7);
      }
   }

}
