package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation.ekf;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;

import static org.junit.jupiter.api.Assertions.*;

public class EKFOutputSmootherTest
{
   @Test
   public void consumesPendingTranslationAndRotationEveryTickWithoutOvershoot()
   {
      var smoother = new EKFOutputSmoother(.5, .5);
      var raw = new RigidBodyTransform();
      var target = new RigidBodyTransform();
      var output = new RigidBodyTransform();
      target.getTranslation().setX(.1);
      target.getRotation().setToYawOrientation(.1);
      smoother.update(1_000_000_000L, raw, raw, output);
      for (int i = 1; i <= 300; i++)
      {
         smoother.update(1_000_000_000L + i * 1_000_000L, raw, target, output);
         assertEquals(Math.min(.1, i * .0005), output.getTranslationX(), 1e-10);
         assertEquals(Math.min(.1, i * .0005), output.getRotation().getYaw(), 1e-10);
      }
      assertTrue(raw.epsilonEquals(new RigidBodyTransform(), 0));
      assertEquals(.1, target.getTranslationX(), 0);
   }

   @Test
   public void boundsOnlyLocalizationChangeWhilePreservingRawMotionFarFromWorldOrigin()
   {
      var smoother = new EKFOutputSmoother(.5, .5);
      var raw = new RigidBodyTransform();
      raw.getTranslation().set(100, 20, 1);
      var previousRaw = new RigidBodyTransform(raw);
      var previousOutput = new RigidBodyTransform();
      var output = new RigidBodyTransform();
      smoother.update(1_000_000_000L, raw, raw, previousOutput);
      for (int i = 1; i <= 100; i++)
      {
         raw.getTranslation().add(.002, .001, 0); // Physical motion exceeds the localization speed limit.
         raw.getRotation().setToYawOrientation(i * .002);
         var target = new RigidBodyTransform(raw);
         target.getTranslation().add(.1, 0, 0);
         var orientation = new Quaternion(raw.getRotation());
         var offset = new Quaternion();
         offset.setYawPitchRoll(.1, .05, 0);
         orientation.preMultiply(offset);
         target.getRotation().set(orientation);
         var motion = new RigidBodyTransform(previousRaw);
         motion.invert();
         motion.multiply(raw);
         var predicted = new RigidBodyTransform(previousOutput);
         predicted.multiply(motion);
         smoother.update(1_000_000_000L + i * 1_000_000L, raw, target, output);
         var translation = new Vector3D(output.getTranslation());
         translation.sub(predicted.getTranslation());
         assertTrue(translation.length() <= .0005 + 1e-10);
         var rotation = new Quaternion(output.getRotation());
         rotation.multiplyConjugateOther(new Quaternion(predicted.getRotation()));
         var angle = new Vector3D();
         rotation.getRotationVector(angle);
         assertTrue(angle.length() <= .0005 + 1e-10);
         previousRaw.set(raw);
         previousOutput.set(output);
      }
   }

   @Test
   public void resetDropsPendingOffsetAndInvalidInputsCannotAdvanceOutput()
   {
      var smoother = new EKFOutputSmoother(.5, .5);
      var raw = new RigidBodyTransform();
      var target = new RigidBodyTransform();
      var output = new RigidBodyTransform();
      target.getTranslation().setX(.1);
      smoother.update(100, raw, raw, output);
      smoother.update(1_000_100, raw, target, output);
      assertThrows(IllegalArgumentException.class, () -> smoother.update(1_000_100, raw, target, output));
      var invalid = new RigidBodyTransform();
      invalid.getTranslation().setX(Double.NaN);
      assertThrows(IllegalArgumentException.class, () -> smoother.update(2_000_100, raw, invalid, output));
      smoother.reset();
      smoother.update(100, raw, raw, output);
      assertTrue(output.epsilonEquals(raw, 0));
   }
}
