package us.ihmc.rdx.behaviorTree;

import com.badlogic.gdx.math.Matrix4;
import controller_msgs.RigidBodyTransformMessage;
import us.ihmc.communication.crdt.CRDTBidirectionalPoint3D;
import us.ihmc.communication.crdt.CRDTBidirectionalRigidBodyTransform;
import us.ihmc.communication.crdt.LatestTimestampModifiable;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;

public class RDXCRDTTools
{
   /**
    * Update and modify the field when the gizmo is moved and/or the parent frame is changed.
    * Otherwise, make sure the current state reflects the synced field because it can be changed
    * by other CRDT actors.
    */
   public static void syncGizmoWithBidirectionalField(RDXPose3DGizmo gizmo,
                                                      CRDTBidirectionalRigidBodyTransform transform,
                                                      LatestTimestampModifiable latestTimestampModifiable)
   {
      if (gizmo.getGizmoModifiedByUser().poll())
      {
         transform.getValueAndModify().set(gizmo.getTransformToParent());
      }
      else if (latestTimestampModifiable.isModified())
      {
         gizmo.getTransformToParent().set(transform.getValueReadOnly());
      }

      gizmo.update();
   }

   /**
    * Update and modify the field when the gizmo is moved and/or the parent frame is changed.
    * Otherwise, make sure the current state reflects the synced field because it can be changed
    * by other CRDT actors.
    */
   public static void syncGizmoWithBidirectionalField(RDXPose3DGizmo gizmo, CRDTBidirectionalPoint3D point, LatestTimestampModifiable latestTimestampModifiable)
   {
      if (gizmo.getGizmoModifiedByUser().poll())
      {
         point.getValueAndModify().set(gizmo.getTransformToParent().getTranslation());
      }
      else if (latestTimestampModifiable.isModified())
      {
         gizmo.getTransformToParent().getTranslation().set(point.getValueReadOnly());
      }

      gizmo.update();
   }

   public static void toLibGDX(RigidBodyTransformMessage rigidBodyTransformMessage, Matrix4 gdxAffineToPack)
   {
      gdxAffineToPack.val[Matrix4.M00] = (float) rigidBodyTransformMessage.getM00();
      gdxAffineToPack.val[Matrix4.M01] = (float) rigidBodyTransformMessage.getM01();
      gdxAffineToPack.val[Matrix4.M02] = (float) rigidBodyTransformMessage.getM02();
      gdxAffineToPack.val[Matrix4.M10] = (float) rigidBodyTransformMessage.getM10();
      gdxAffineToPack.val[Matrix4.M11] = (float) rigidBodyTransformMessage.getM11();
      gdxAffineToPack.val[Matrix4.M12] = (float) rigidBodyTransformMessage.getM12();
      gdxAffineToPack.val[Matrix4.M20] = (float) rigidBodyTransformMessage.getM20();
      gdxAffineToPack.val[Matrix4.M21] = (float) rigidBodyTransformMessage.getM21();
      gdxAffineToPack.val[Matrix4.M22] = (float) rigidBodyTransformMessage.getM22();
      gdxAffineToPack.val[Matrix4.M03] = (float) rigidBodyTransformMessage.getX();
      gdxAffineToPack.val[Matrix4.M13] = (float) rigidBodyTransformMessage.getY();
      gdxAffineToPack.val[Matrix4.M23] = (float) rigidBodyTransformMessage.getZ();
   }
}
