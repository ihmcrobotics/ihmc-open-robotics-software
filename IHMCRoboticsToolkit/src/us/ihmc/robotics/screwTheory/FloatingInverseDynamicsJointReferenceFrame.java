package us.ihmc.robotics.screwTheory;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class FloatingInverseDynamicsJointReferenceFrame extends ReferenceFrame
{
   private static final long serialVersionUID = 8238436678986979988L;
   private final Quaternion rotation = new Quaternion();
   private final Vector3D translation = new Vector3D();

   public FloatingInverseDynamicsJointReferenceFrame(String frameName, ReferenceFrame parentFrame)
   {
      super(frameName, parentFrame);
   }

   public void setRotation(QuaternionReadOnly rotation)
   {
      this.rotation.set(rotation);
   }

   public void setTranslation(Vector3DReadOnly translation)
   {
      this.translation.set(translation);
   }

   public void getRotation(QuaternionBasics rotationToPack)
   {
      rotationToPack.set(rotation);
   }

   public void getTranslation(Tuple3DBasics translationToPack)
   {
      translationToPack.set(translation);
   }

   @Override
   protected void updateTransformToParent(RigidBodyTransform transformToParent)
   {
      transformToParent.set(rotation, translation);
   }
}
