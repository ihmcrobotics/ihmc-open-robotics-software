package us.ihmc.rdx.ui.interactable;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.robotics.interaction.CylinderRayIntersection;

/**
 * We model the collision for the sensor as a cylinder.
 */
public class RDXInteractableRealsenseD455 extends RDXInteractableSensor
{
   private static final double LENGTH = 0.1;
   private static final double RADIUS = 0.025 / 2.0;
   private final CylinderRayIntersection cylinderIntersection = new CylinderRayIntersection();
   private final Point3D offset = new Point3D(-RADIUS, 0.0, 0.0);

   public RDXInteractableRealsenseD455(RDX3DPanel panel3D)
   {
      super(panel3D, "environmentObjects/d455Sensor/D455.g3dj");
   }

   public RDXInteractableRealsenseD455(RDX3DPanel panel3D, ReferenceFrame referenceFrameToRepresent, RigidBodyTransform transformToParentToModify)
   {
      super(panel3D, referenceFrameToRepresent, transformToParentToModify, "environmentObjects/d455Sensor/D455.g3dj");
   }

   @Override
   protected double calculateClosestCollision(Line3DReadOnly mousePickRay)
   {
      cylinderIntersection.update(LENGTH, RADIUS, offset, Axis3D.Y, interactableFrameModel.getReferenceFrame());
      return cylinderIntersection.intersect(mousePickRay);
   }
}
