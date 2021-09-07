package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import static us.ihmc.robotics.Assert.assertEquals;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;

class CompassYawRemoverTest
{

   @Test
   void testCompassYawRemoverAssumingYawRollPitchOrderingUsingQuaternions()
   {
      CompassYawRemover compassYawRemover = new CompassYawRemover();

      double yaw = 1.0;
      double pitch = 0.378;
      double roll = -0.373;

      Quaternion yawRotation = new Quaternion();
      Quaternion pitchRotation = new Quaternion();
      Quaternion rollRotation = new Quaternion();

      yawRotation.setYawPitchRoll(yaw, 0.0, 0.0);
      pitchRotation.setYawPitchRoll(0.0, pitch, 0.0);
      rollRotation.setYawPitchRoll(0.0, 0.0, roll);

      Quaternion orientationToRemoveYawFrom = new Quaternion();
      orientationToRemoveYawFrom.set(yawRotation);
      orientationToRemoveYawFrom.multiply(rollRotation);
      orientationToRemoveYawFrom.multiply(pitchRotation);

      compassYawRemover.removeCompassYawAssumingYawRollPitchOrdering(orientationToRemoveYawFrom);

      Quaternion expectedAnswer = new Quaternion();
      expectedAnswer.set(rollRotation);
      expectedAnswer.multiply(pitchRotation);

      EuclidCoreTestTools.assertQuaternionEquals(expectedAnswer, orientationToRemoveYawFrom, 1e-7);
   }

   @Test
   void testCompassYawRemoverAssumingYawRollPitchOrderingUsingRotationMatrices()
   {
      CompassYawRemover compassYawRemover = new CompassYawRemover();

      double yaw = 1.0;
      double pitch = 0.378;
      double roll = -0.373;

      RotationMatrix yawRotation = new RotationMatrix();
      RotationMatrix pitchRotation = new RotationMatrix();
      RotationMatrix rollRotation = new RotationMatrix();

      yawRotation.setYawPitchRoll(yaw, 0.0, 0.0);
      pitchRotation.setYawPitchRoll(0.0, pitch, 0.0);
      rollRotation.setYawPitchRoll(0.0, 0.0, roll);

      RotationMatrix orientationToRemoveYawFrom = new RotationMatrix();
      orientationToRemoveYawFrom.set(yawRotation);
      orientationToRemoveYawFrom.multiply(rollRotation);
      orientationToRemoveYawFrom.multiply(pitchRotation);

      compassYawRemover.removeCompassYawAssumingYawRollPitchOrdering(orientationToRemoveYawFrom);

      RotationMatrix expectedAnswer = new RotationMatrix();
      expectedAnswer.set(rollRotation);
      expectedAnswer.multiply(pitchRotation);

      EuclidCoreTestTools.assertRotationMatrixGeometricallyEquals(expectedAnswer, orientationToRemoveYawFrom, 1e-7);
   }

   @Test
   void testCompassYawRemoverAssumingYawPitchRollOrderingUsingQuaternions()
   {
      CompassYawRemover compassYawRemover = new CompassYawRemover();

      double yaw = -0.58;
      double pitch = 0.178;
      double roll = -0.673;

      Quaternion yawRotation = new Quaternion();
      Quaternion pitchRotation = new Quaternion();
      Quaternion rollRotation = new Quaternion();

      yawRotation.setYawPitchRoll(yaw, 0.0, 0.0);
      pitchRotation.setYawPitchRoll(0.0, pitch, 0.0);
      rollRotation.setYawPitchRoll(0.0, 0.0, roll);

      Quaternion orientationToRemoveYawFrom = new Quaternion();
      orientationToRemoveYawFrom.set(yawRotation);
      orientationToRemoveYawFrom.multiply(pitchRotation);
      orientationToRemoveYawFrom.multiply(rollRotation);

      compassYawRemover.removeCompassYawAssumingYawPitchRollOrdering(orientationToRemoveYawFrom);

      Quaternion expectedAnswer = new Quaternion();
      expectedAnswer.set(pitchRotation);
      expectedAnswer.multiply(rollRotation);

      EuclidCoreTestTools.assertQuaternionEquals(expectedAnswer, orientationToRemoveYawFrom, 1e-7);
   }

   @Test
   void testCompassYawRemoverFromMiddleAssumingRollPitchOrderingUsingQuaternions()
   {
      CompassYawRemover compassYawRemover = new CompassYawRemover();

      Quaternion parentToWorld = new Quaternion();
      Quaternion parentHeadingError = new Quaternion();
      Quaternion jointYaw = new Quaternion();
      Quaternion jointRoll = new Quaternion();
      Quaternion jointPitch = new Quaternion();

      parentHeadingError.setYawPitchRoll(-0.7, 0.0, 0.0);
      parentToWorld.setYawPitchRoll(0.1, 0.2, 0.3);
      jointYaw.setYawPitchRoll(0.5, 0.0, 0.0);
      jointRoll.setYawPitchRoll(0.0, 0.0, -0.3);
      jointPitch.setYawPitchRoll(0.0, 0.68, 0.0);

      Quaternion childToWorld = new Quaternion();
      Quaternion childHeadingError = new Quaternion();
      childHeadingError.setYawPitchRoll(0.1, 0.0, 0.0);

      childToWorld.set(parentToWorld);
      childToWorld.multiply(jointYaw);
      childToWorld.multiply(jointRoll);
      childToWorld.multiply(jointPitch);

      Quaternion parentIMUMeasuredWithHeadingError = new Quaternion();
      parentIMUMeasuredWithHeadingError.set(parentHeadingError);
      parentIMUMeasuredWithHeadingError.multiply(parentToWorld);

      Quaternion childIMUMeasuredWithHeadingError = new Quaternion();
      childIMUMeasuredWithHeadingError.set(childHeadingError);
      childIMUMeasuredWithHeadingError.multiply(childToWorld);

      Quaternion rotationOne = new Quaternion(parentIMUMeasuredWithHeadingError);
      rotationOne.multiply(jointYaw);
      rotationOne.inverse();

      Quaternion rotationTwo = new Quaternion();
      rotationTwo.set(childIMUMeasuredWithHeadingError);

      Quaternion rollPitchSolution = new Quaternion();

      compassYawRemover.removeCompassYawFromMiddleAssumingRollPitchOrdering(rotationOne, rotationTwo, rollPitchSolution);

      Quaternion expectedAnswer = new Quaternion();
      expectedAnswer.set(jointRoll);
      expectedAnswer.multiply(jointPitch);

      EuclidCoreTestTools.assertQuaternionEquals(expectedAnswer, rollPitchSolution, 1e-7);
   }

   @Test
   public void testComputeRollAndPitchGivenTwoVectors()
   {
      CompassYawRemover compassYawRemover = new CompassYawRemover();

      Vector3D vectorOne = new Vector3D(0.0, 0.0, -1.0);
      Vector3D vectorTwo = new Vector3D(vectorOne);

      double[] rollAndPitchToPack = new double[2];

      compassYawRemover.computeRollAndPitchGivenTwoVectors(vectorOne, vectorTwo, rollAndPitchToPack);

      assertEquals(0.0, rollAndPitchToPack[0], 1e-7);
      assertEquals(0.0, rollAndPitchToPack[1], 1e-7);

      vectorOne = new Vector3D(0.3, 0.5, -1.0);
      vectorTwo.set(vectorOne);

      compassYawRemover.computeRollAndPitchGivenTwoVectors(vectorOne, vectorTwo, rollAndPitchToPack);

      assertEquals(0.0, rollAndPitchToPack[0], 1e-7);
      assertEquals(0.0, rollAndPitchToPack[1], 1e-7);

      RigidBodyTransform rollTransform = new RigidBodyTransform();
      RigidBodyTransform pitchTransform = new RigidBodyTransform();

      vectorTwo.set(0.3, 0.1, -2.0);
      vectorOne.set(vectorTwo);

      double pitch = 0.2;
      double roll = 0.0;

      pitchTransform.getRotation().setToPitchOrientation(pitch);
      rollTransform.getRotation().setToRollOrientation(roll);

      pitchTransform.transform(vectorOne);
      rollTransform.transform(vectorOne);

      compassYawRemover.computeRollAndPitchGivenTwoVectors(vectorOne, vectorTwo, rollAndPitchToPack);

      assertEquals(roll, rollAndPitchToPack[0], 1e-7);
      assertEquals(pitch, rollAndPitchToPack[1], 1e-7);

      vectorTwo.set(1.2, -2.3, -9.81);
      vectorOne.set(vectorTwo);

      pitch = 0.0;
      roll = 0.123;

      pitchTransform.getRotation().setToPitchOrientation(pitch);
      rollTransform.getRotation().setToRollOrientation(roll);

      pitchTransform.transform(vectorOne);
      rollTransform.transform(vectorOne);

      compassYawRemover.computeRollAndPitchGivenTwoVectors(vectorOne, vectorTwo, rollAndPitchToPack);

      assertEquals(roll, rollAndPitchToPack[0], 2e-3);
      assertEquals(pitch, rollAndPitchToPack[1], 1e-7);

      vectorTwo.set(0.435, 0.000, -0.900);
      vectorOne.set(vectorTwo);

      pitch = 0.0;
      roll = 0.33;

      pitchTransform.getRotation().setToPitchOrientation(pitch);
      rollTransform.getRotation().setToRollOrientation(roll);

      pitchTransform.transform(vectorOne);
      rollTransform.transform(vectorOne);

      compassYawRemover.computeRollAndPitchGivenTwoVectors(vectorOne, vectorTwo, rollAndPitchToPack);

      assertEquals(roll, rollAndPitchToPack[0], 1e-7);
      assertEquals(pitch, rollAndPitchToPack[1], 1e-7);

      vectorTwo.set(0.0, 0.0, -1.0);
      vectorOne.set(vectorTwo);

      pitch = -0.45;
      roll = 0.33;

      pitchTransform.getRotation().setToPitchOrientation(pitch);
      rollTransform.getRotation().setToRollOrientation(roll);

      pitchTransform.transform(vectorOne);
      rollTransform.transform(vectorOne);

      compassYawRemover.computeRollAndPitchGivenTwoVectors(vectorOne, vectorTwo, rollAndPitchToPack);

      assertEquals(pitch, rollAndPitchToPack[1], 1e-7);
      assertEquals(roll, rollAndPitchToPack[0], 1e-7);

      vectorTwo.set(0.0, 0.0, -1.0);
      vectorOne.set(1.0, 0.0, 0.0);

      pitch = -Math.PI / 2.0;
      roll = 0.0;
      compassYawRemover.computeRollAndPitchGivenTwoVectors(vectorOne, vectorTwo, rollAndPitchToPack);

      assertEquals(pitch, rollAndPitchToPack[1], 1e-7);
      assertEquals(roll, rollAndPitchToPack[0], 1e-7);

      vectorTwo.set(0.0, 0.0, -1.0);
      vectorOne.set(0.0, 1.0, 0.0);

      pitch = 0.0;
      roll = Math.PI / 2.0;
      compassYawRemover.computeRollAndPitchGivenTwoVectors(vectorOne, vectorTwo, rollAndPitchToPack);

      assertEquals(pitch, rollAndPitchToPack[1], 1e-7);
      assertEquals(roll, rollAndPitchToPack[0], 1e-7);
   }

   @Test
   public void testComputeRollAndPitchGivenTwoVectorsWithRandomSamples()
   {
      CompassYawRemover compassYawRemover = new CompassYawRemover();
      double[] rollAndPitchToPack = new double[2];
      RigidBodyTransform rollTransform = new RigidBodyTransform();
      RigidBodyTransform pitchTransform = new RigidBodyTransform();

      Random random = new Random(1776L);
      int numberOfTests = 10000;

      for (int i = 0; i < numberOfTests; i++)
      {
         // Answers seem to only be valid if some combination of pitch and roll is less than PI/2.0 total.

         Vector3D vectorA = new Vector3D(0.0, 0.0, -1.0);
         pitchTransform.getRotation().setToPitchOrientation(EuclidCoreRandomTools.nextDouble(random, 0.35 * Math.PI));
         rollTransform.getRotation().setToRollOrientation(EuclidCoreRandomTools.nextDouble(random, 0.15 * Math.PI));

         pitchTransform.transform(vectorA);
         rollTransform.transform(vectorA);

         Vector3D vectorB = new Vector3D(0.0, 0.0, -1.0);
         pitchTransform.getRotation().setToPitchOrientation(EuclidCoreRandomTools.nextDouble(random, 0.35 * Math.PI));
         rollTransform.getRotation().setToRollOrientation(EuclidCoreRandomTools.nextDouble(random, 0.15 * Math.PI));

         pitchTransform.transform(vectorB);
         rollTransform.transform(vectorB);

         Vector3D cross = new Vector3D(vectorA);
         cross.cross(vectorB);

         compassYawRemover.computeRollAndPitchGivenTwoVectors(vectorA, vectorB, rollAndPitchToPack);
         if ((Math.abs(rollAndPitchToPack[0]) < Math.PI * 0.4) && (Math.abs(rollAndPitchToPack[1]) < Math.PI * 0.4))
         {
            pitchTransform.getRotation().setToPitchOrientation(rollAndPitchToPack[1]);
            rollTransform.getRotation().setToRollOrientation(rollAndPitchToPack[0]);

            Vector3D vectorC = new Vector3D(vectorB);
            pitchTransform.transform(vectorC);
            rollTransform.transform(vectorC);

            EuclidCoreTestTools.assertTuple3DEquals(vectorA, vectorC, 1e-3);
         }
      }

   }

   @Test
   public void testSolveASinThetaPlusBCosThetaEqualsC()
   {
      CompassYawRemover compassYawRemover = new CompassYawRemover();

      double theta = 0.5;
      double a = 0.7;
      double b = -0.2;

      double aCosThetaPlusBSinTheta = a * Math.cos(theta) + b * Math.sin(theta);
      double thetaCheck = compassYawRemover.solveACosThetaPlusBSinThetaEqualsC(a, b, aCosThetaPlusBSinTheta);

      assertEquals(theta, thetaCheck, 1e-7);
   }

}
