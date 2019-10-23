package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.collision.KinematicsCollisionResult;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.EuclidCoreMissingTools;

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

   public void update(KinematicsCollisionResult collision, boolean centerFrameAtA)
   {
      if (centerFrameAtA)
      {
         collisionPoint0.setIncludingFrame(collision.getPointOnA());
         collisionPoint1.setIncludingFrame(collision.getPointOnB());
         collisionNormal0.setIncludingFrame(collision.getNormalOnA());
         collisionNormal1.setIncludingFrame(collision.getNormalOnB());
      }
      else
      {
         collisionPoint0.setIncludingFrame(collision.getPointOnB());
         collisionPoint1.setIncludingFrame(collision.getPointOnA());
         collisionNormal0.setIncludingFrame(collision.getNormalOnB());
         collisionNormal1.setIncludingFrame(collision.getNormalOnA());
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
         if (collision.areShapesColliding())
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

      transformToParent.setTranslation(this.origin);
      EuclidCoreMissingTools.rotationMatrix3DFromZUpToVector3D(this.zAxis, transformToParent.getRotation());
      update();
   }

   @Override
   protected void updateTransformToParent(RigidBodyTransform transformToParent)
   {
      transformToParent.set(this.transformToParent);
   }
}
