package us.ihmc.rdx.ui.behavior.tools;

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
}
