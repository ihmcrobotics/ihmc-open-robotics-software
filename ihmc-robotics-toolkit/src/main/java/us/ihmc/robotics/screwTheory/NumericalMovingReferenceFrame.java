package us.ihmc.robotics.screwTheory;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;

/**
 * {@code NumericalMovingReferenceFrame} can be used to extend a simple {@code ReferenceFrame} to a
 * {@code MovingReferenceFrame} by computing the frame velocity by using finite difference on its
 * transform.
 * <p>
 * For this to work properly, the update method has to be called at a constant rate consistent with
 * the {@code updateDT} given at construction time.
 * </p>
 * <p>
 * Note that it is preferable to use this class only for test purposes or environment for which the
 * noise on the frame transform is really small. The resulting velocity is not filtered, so any
 * noise in the transform will result in important peaks in the computed velocity.
 * </p>
 */
public class NumericalMovingReferenceFrame extends MovingReferenceFrame
{

   private final ReferenceFrame originalFrame;

   private final Quaternion previousRotation = new Quaternion();
   private final Vector3D previousTranslation = new Vector3D();

   private final Quaternion rotation = new Quaternion();
   private final Vector3D translation = new Vector3D();

   private final Vector3D angularVelocity = new Vector3D();
   private final Vector3D linearVelocity = new Vector3D();

   private final double updateDT;

   public NumericalMovingReferenceFrame(ReferenceFrame originalFrame, double updateDT)
   {
      this("", originalFrame, updateDT);
   }

   public NumericalMovingReferenceFrame(String nameSuffix, ReferenceFrame originalFrame, double updateDT)
   {
      super(originalFrame.getName() + nameSuffix, originalFrame.getRootFrame());
      this.originalFrame = originalFrame;
      this.updateDT = updateDT;

      previousRotation.setToNaN();
      previousTranslation.setToNaN();
   }

   @Override
   protected void updateTransformToParent(RigidBodyTransform transformToParent)
   {
      transformToParent.set(originalFrame.getTransformToRoot());
   }

   @Override
   protected void updateTwistRelativeToParent(Twist twistRelativeToParentToPack)
   {
      RigidBodyTransform transformToRoot = originalFrame.getTransformToRoot();

      if (previousRotation.containsNaN() || previousTranslation.containsNaN())
      {
         transformToRoot.get(previousRotation, previousTranslation);

         angularVelocity.setToZero();
         linearVelocity.setToZero();
      }
      else
      {
         transformToRoot.get(rotation, translation);
         rotation.multiplyConjugateOther(previousRotation);
         rotation.get(angularVelocity);
         angularVelocity.scale(1.0 / updateDT);
         linearVelocity.sub(translation, previousTranslation);
         linearVelocity.scale(1.0 / updateDT);

         transformToRoot.get(previousRotation, previousTranslation);

         transformToRoot.inverseTransform(angularVelocity);
         transformToRoot.inverseTransform(linearVelocity);
      }

      twistRelativeToParentToPack.set(this, parentFrame, this, linearVelocity, angularVelocity);
   }
}
