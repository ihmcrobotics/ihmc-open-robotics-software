package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.collision.EuclidFrameShape3DCollisionResult;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.EuclidCoreMissingTools;
import us.ihmc.robotics.physics.CollisionResult;

public class KinematicsCollisionFrame extends ReferenceFrame
{
   private final RigidBodyTransform transformToParent = new RigidBodyTransform();

   public KinematicsCollisionFrame(String name, ReferenceFrame parentFrame)
   {
      super(name, parentFrame);
   }

   private final FramePoint3D collisionPoint0 = new FramePoint3D();
   private final FramePoint3D collisionPoint1 = new FramePoint3D();
   private final FrameVector3D collisionNormal0 = new FrameVector3D();
   private final FrameVector3D collisionNormal1 = new FrameVector3D();
   private final FrameVector3D collisionAxis = new FrameVector3D();

   public void update(CollisionResult collision, boolean centerFrameAtA)
   {
      EuclidFrameShape3DCollisionResult collisionData = collision.getCollisionData();

      if (centerFrameAtA)
      {
         collisionPoint0.setIncludingFrame(collisionData.getPointOnA());
         collisionPoint1.setIncludingFrame(collisionData.getPointOnB());
         collisionNormal0.setIncludingFrame(collisionData.getNormalOnA());
         collisionNormal1.setIncludingFrame(collisionData.getNormalOnB());
      }
      else
      {
         collisionPoint0.setIncludingFrame(collisionData.getPointOnB());
         collisionPoint1.setIncludingFrame(collisionData.getPointOnA());
         collisionNormal0.setIncludingFrame(collisionData.getNormalOnB());
         collisionNormal1.setIncludingFrame(collisionData.getNormalOnA());
      }

      collisionAxis.setToZero(getParent());

      if (!collisionNormal0.containsNaN())
      {
         collisionNormal0.changeFrame(getParent());
         collisionAxis.set(collisionNormal0);
      }
      else if (!collisionNormal1.containsNaN())
      {
         collisionNormal1.changeFrame(getParent());
         collisionAxis.set(collisionNormal1);
         collisionAxis.negate();
      }
      else
      {
         collisionPoint0.changeFrame(getParent());
         collisionPoint1.changeFrame(getParent());
         collisionAxis.sub(collisionPoint1, collisionPoint0);
         if (collisionData.areShapesColliding())
            collisionAxis.negate();
      }

      update(collisionPoint0, collisionAxis);
   }

   private final FramePoint3D origin = new FramePoint3D();
   private final FrameVector3D zAxis = new FrameVector3D();

   public void update(FramePoint3DReadOnly origin, FrameVector3DReadOnly zAxis)
   {
      this.origin.setIncludingFrame(origin);
      this.zAxis.setIncludingFrame(zAxis);
      this.origin.changeFrame(getParent());
      this.zAxis.changeFrame(getParent());

      transformToParent.getTranslation().set(this.origin);
      EuclidCoreMissingTools.rotationMatrix3DFromZUpToVector3D(this.zAxis, transformToParent.getRotation());
      update();
   }

   @Override
   protected void updateTransformToParent(RigidBodyTransform transformToParent)
   {
      transformToParent.set(this.transformToParent);
   }
}
