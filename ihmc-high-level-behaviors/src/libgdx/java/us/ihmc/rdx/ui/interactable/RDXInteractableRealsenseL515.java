package us.ihmc.rdx.ui.interactable;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.robotics.interaction.CylinderRayIntersection;

public class RDXInteractableRealsenseL515 extends RDXInteractableSensor
{
   private final CylinderRayIntersection cylinderIntersection = new CylinderRayIntersection();
   private final Point3D offset = new Point3D();

   public RDXInteractableRealsenseL515(RDX3DPanel panel3D)
   {
      super(panel3D, "environmentObjects/l515Sensor/L515Sensor.g3dj");
   }

   public RDXInteractableRealsenseL515(RDX3DPanel panel3D, ReferenceFrame referenceFrameToRepresent, RigidBodyTransform transformToParentToModify)
   {
      super(panel3D, referenceFrameToRepresent, transformToParentToModify, "environmentObjects/l515Sensor/L515Sensor.g3dj");
   }

   @Override
   protected double calculateClosestCollision(Line3DReadOnly mousePickRay)
   {
      cylinderIntersection.update(0.028, 0.031, offset, Axis3D.X, interactableFrameModel.getReferenceFrame());
      return cylinderIntersection.intersect(mousePickRay);
   }
}
