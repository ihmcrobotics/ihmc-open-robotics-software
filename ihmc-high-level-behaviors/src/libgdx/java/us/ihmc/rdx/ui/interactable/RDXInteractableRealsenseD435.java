package us.ihmc.rdx.ui.interactable;

import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.robotics.interaction.BoxRayIntersection;

public class RDXInteractableRealsenseD435 extends RDXInteractableSensor
{
   private final BoxRayIntersection boxRayIntersection = new BoxRayIntersection();

   public RDXInteractableRealsenseD435(RDX3DPanel panel3D)
   {
      super(panel3D, "environmentObjects/d435Sensor/D435.g3dj");
   }

   public RDXInteractableRealsenseD435(RDX3DPanel panel3D, ReferenceFrame referenceFrameToRepresent, RigidBodyTransform transformToParentToModify)
   {
      super(panel3D, referenceFrameToRepresent, transformToParentToModify, "environmentObjects/d435Sensor/D435.g3dj");
   }

   @Override
   protected double calculateClosestCollision(Line3DReadOnly mousePickRay)
   {
      if (boxRayIntersection.intersect(0.025, 0.09, 0.025, interactableFrameModel.getReferenceFrame().getTransformToWorldFrame(), mousePickRay))
      {
         return mousePickRay.getPoint().distance(boxRayIntersection.getFirstIntersectionToPack());
      }
      else
      {
         return Double.NaN;
      }
   }
}
