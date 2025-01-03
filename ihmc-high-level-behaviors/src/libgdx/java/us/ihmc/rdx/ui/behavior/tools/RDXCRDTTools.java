package us.ihmc.rdx.ui.behavior.tools;

import us.ihmc.communication.crdt.CRDTBidirectionalPoint3D;
import us.ihmc.communication.crdt.CRDTBidirectionalRigidBodyTransform;
import us.ihmc.communication.crdt.RequestConfirmFreezable;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;

public class RDXCRDTTools
{
   /**
    * Update and freeze the field when the gizmo is moved and/or the parent frame is changed.
    * Otherwise, make sure the current state reflects the synced field because it can be changed
    * by other CRDT actors.
    */
   public static void syncGizmoWithBidirectionalField(RDXPose3DGizmo gizmo,
                                                      CRDTBidirectionalRigidBodyTransform transform,
                                                      RequestConfirmFreezable requestConfirmFreezable)
   {
      if (gizmo.getGizmoModifiedByUser().poll())
      {
         transform.getValueAndFreeze().set(gizmo.getTransformToParent());
      }
      else if (!requestConfirmFreezable.isFrozen())
      {
         gizmo.getTransformToParent().set(transform.getValueReadOnly());
      }

      gizmo.update();
   }

   /**
    * Update and freeze the field when the gizmo is moved and/or the parent frame is changed.
    * Otherwise, make sure the current state reflects the synced field because it can be changed
    * by other CRDT actors.
    */
   public static void syncGizmoWithBidirectionalField(RDXPose3DGizmo gizmo, CRDTBidirectionalPoint3D point, RequestConfirmFreezable requestConfirmFreezable)
   {
      if (gizmo.getGizmoModifiedByUser().poll())
      {
         point.getValueAndFreeze().set(gizmo.getTransformToParent().getTranslation());
      }
      else if (!requestConfirmFreezable.isFrozen())
      {
         gizmo.getTransformToParent().getTranslation().set(point.getValueReadOnly());
      }

      gizmo.update();
   }
}
