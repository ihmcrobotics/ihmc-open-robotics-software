package us.ihmc.robotics.screwTheory;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class NumericalMovingReferenceFrame extends MovingReferenceFrame
{
   private static final long serialVersionUID = 1792258730231152017L;

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
      super(originalFrame.getName(), originalFrame.getRootFrame());
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

         RigidBodyTransform inverseTransformToRoot = originalFrame.getInverseTransformToRoot();
         inverseTransformToRoot.transform(angularVelocity);
         inverseTransformToRoot.transform(linearVelocity);
      }

      twistRelativeToParentToPack.set(this, parentFrame, this, linearVelocity, angularVelocity);
   }
}
