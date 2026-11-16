package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.ekf;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.yoVariables.registry.YoRegistry;

import static org.junit.jupiter.api.Assertions.*;

public class EKFOutputSmootherTest
{
   private static final double MAXIMUM_SPEED = 0.5;
   private static final double DT = 0.001;

   private static EKFOutputSmoother createSmoother()
   {
      return new EKFOutputSmoother(MAXIMUM_SPEED, MAXIMUM_SPEED, DT, new YoRegistry("test"));
   }

   @Test
   public void consumesPendingTranslationAndRotationEveryTickWithoutOvershoot()
   {
      EKFOutputSmoother smoother = createSmoother();
      RigidBodyTransform kinematicPose = new RigidBodyTransform();
      RigidBodyTransform target = new RigidBodyTransform();
      RigidBodyTransform output = new RigidBodyTransform();
      target.getTranslation().setX(0.1);
      target.getRotation().setToYawOrientation(0.1);
      smoother.update(kinematicPose, kinematicPose, output);
      for (int tick = 1; tick <= 300; tick++)
      {
         smoother.update(kinematicPose, target, output);
         assertEquals(Math.min(0.1, tick * MAXIMUM_SPEED * DT), output.getTranslationX(), 1.0e-10);
         assertEquals(Math.min(0.1, tick * MAXIMUM_SPEED * DT), output.getRotation().getYaw(), 1.0e-10);
      }
      assertTrue(kinematicPose.epsilonEquals(new RigidBodyTransform(), 0));
      assertEquals(0.1, target.getTranslationX(), 0);
   }

   @Test
   public void boundsOnlyLocalizationChangeWhilePreservingKinematicMotionFarFromWorldOrigin()
   {
      EKFOutputSmoother smoother = createSmoother();
      RigidBodyTransform kinematicPose = new RigidBodyTransform();
      kinematicPose.getTranslation().set(100, 20, 1);
      RigidBodyTransform previousKinematicPose = new RigidBodyTransform(kinematicPose);
      RigidBodyTransform previousOutput = new RigidBodyTransform();
      RigidBodyTransform output = new RigidBodyTransform();
      smoother.update(kinematicPose, kinematicPose, previousOutput);
      for (int tick = 1; tick <= 100; tick++)
      {
         kinematicPose.getTranslation().add(0.002, 0.001, 0); // Physical motion exceeds the localization speed limit.
         kinematicPose.getRotation().setToYawOrientation(tick * 0.002);
         RigidBodyTransform target = new RigidBodyTransform(kinematicPose);
         target.getTranslation().add(0.1, 0, 0);
         Quaternion targetOrientation = new Quaternion(kinematicPose.getRotation());
         Quaternion orientationOffset = new Quaternion();
         orientationOffset.setYawPitchRoll(0.1, 0.05, 0);
         targetOrientation.preMultiply(orientationOffset);
         target.getRotation().set(targetOrientation);
         RigidBodyTransform kinematicMotion = new RigidBodyTransform(previousKinematicPose);
         kinematicMotion.invert();
         kinematicMotion.multiply(kinematicPose);
         RigidBodyTransform predictedOutput = new RigidBodyTransform(previousOutput);
         predictedOutput.multiply(kinematicMotion);

         smoother.update(kinematicPose, target, output);

         Vector3D translationChange = new Vector3D(output.getTranslation());
         translationChange.sub(predictedOutput.getTranslation());
         assertTrue(translationChange.length() <= MAXIMUM_SPEED * DT + 1.0e-10);
         Quaternion rotationChange = new Quaternion(output.getRotation());
         rotationChange.multiplyConjugateOther(new Quaternion(predictedOutput.getRotation()));
         Vector3D rotationChangeVector = new Vector3D();
         rotationChange.getRotationVector(rotationChangeVector);
         assertTrue(rotationChangeVector.length() <= MAXIMUM_SPEED * DT + 1.0e-10);
         previousKinematicPose.set(kinematicPose);
         previousOutput.set(output);
      }
   }

   @Test
   public void resetDropsPendingOffsetAndInvalidInputsCannotAdvanceOutput()
   {
      EKFOutputSmoother smoother = createSmoother();
      RigidBodyTransform kinematicPose = new RigidBodyTransform();
      RigidBodyTransform target = new RigidBodyTransform();
      RigidBodyTransform output = new RigidBodyTransform();
      target.getTranslation().setX(0.1);
      smoother.update(kinematicPose, kinematicPose, output);
      smoother.update(kinematicPose, target, output);
      RigidBodyTransform invalidTarget = new RigidBodyTransform();
      invalidTarget.getTranslation().setX(Double.NaN);
      assertThrows(IllegalArgumentException.class, () -> smoother.update(kinematicPose, invalidTarget, output));
      smoother.reset();
      smoother.update(kinematicPose, kinematicPose, output);
      assertTrue(output.epsilonEquals(kinematicPose, 0));
   }
}
