package us.ihmc.rdx.ui.interactable;

import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.robotics.interaction.CylinderRayIntersection;

public class RDXInteractableOuster extends RDXInteractableSensor
{
   private final CylinderRayIntersection cylinderIntersection = new CylinderRayIntersection();

   public RDXInteractableOuster(RDX3DPanel panel3D)
   {
      super(panel3D, "environmentObjects/ousterSensor/Ouster.g3dj");
   }

   public RDXInteractableOuster(RDX3DPanel panel3D, ReferenceFrame referenceFrameToRepresent, RigidBodyTransform transformToParentToModify)
   {
      super(panel3D, referenceFrameToRepresent, transformToParentToModify, "environmentObjects/ousterSensor/Ouster.g3dj");
   }

   @Override
   protected double calculateClosestCollision(Line3DReadOnly mousePickRay)
   {
      cylinderIntersection.update(0.0734, 0.04, -0.0372, interactableFrameModel.getReferenceFrame().getTransformToWorldFrame());
      return cylinderIntersection.intersect(mousePickRay);
   }
}
