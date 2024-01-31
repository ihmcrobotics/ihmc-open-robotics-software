package us.ihmc.rdx.ui.interactable;

import com.badlogic.gdx.graphics.g3d.model.data.ModelData;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.rdx.tools.RDXModelLoader;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.affordances.RDXInteractableFrameModel;
import us.ihmc.robotics.interaction.CylinderRayIntersection;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;

/**
 * We model the collision for the sensor as a cylinder.
 */
public class RDXInteractableRealsenseD455
{
   private static final double LENGTH = 0.1;
   private static final double RADIUS = 0.025 / 2.0;

   private final RDXInteractableFrameModel interactableFrameModel = new RDXInteractableFrameModel();
   private final CylinderRayIntersection cylinderIntersection = new CylinderRayIntersection();
   private final Point3D offset = new Point3D(-RADIUS, 0.0, 0.0);

   public RDXInteractableRealsenseD455(RDX3DPanel panel3D)
   {
      RigidBodyTransform transform = new RigidBodyTransform();
      ReferenceFrame referenceFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(ReferenceFrame.getWorldFrame(), transform);
      create(panel3D, referenceFrame, transform);
   }

   public RDXInteractableRealsenseD455(RDX3DPanel panel3D, ReferenceFrame referenceFrameToRepresent, RigidBodyTransform transformToParentToModify)
   {
      create(panel3D, referenceFrameToRepresent, transformToParentToModify);
   }

   private void create(RDX3DPanel panel3D, ReferenceFrame referenceFrameToRepresent, RigidBodyTransform transformToParentToModify)
   {
      ModelData d455SensorModel = RDXModelLoader.loadModelData("environmentObjects/d455Sensor/D455.g3dj");
      interactableFrameModel.create(referenceFrameToRepresent, transformToParentToModify, panel3D, d455SensorModel, this::calculateClosestCollision);
   }

   private double calculateClosestCollision(Line3DReadOnly mousePickRay)
   {
      cylinderIntersection.update(LENGTH, RADIUS, offset, Axis3D.Y, interactableFrameModel.getReferenceFrame());
      return cylinderIntersection.intersect(mousePickRay);
   }

   public RDXInteractableFrameModel getInteractableFrameModel()
   {
      return interactableFrameModel;
   }
}
