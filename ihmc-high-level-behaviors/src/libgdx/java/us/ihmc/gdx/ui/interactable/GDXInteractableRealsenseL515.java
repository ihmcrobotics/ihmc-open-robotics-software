package us.ihmc.gdx.ui.interactable;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.gdx.tools.GDXModelInstance;
import us.ihmc.gdx.tools.GDXModelLoader;
import us.ihmc.gdx.ui.GDX3DPanel;
import us.ihmc.gdx.ui.affordances.GDXInteractableFrameModel;
import us.ihmc.gdx.ui.gizmo.CylinderRayIntersection;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;

public class GDXInteractableRealsenseL515
{
   private final GDXInteractableFrameModel interactableFrameModel = new GDXInteractableFrameModel();
   private final CylinderRayIntersection cylinderIntersection = new CylinderRayIntersection();
   private final Point3D offset = new Point3D();

   public GDXInteractableRealsenseL515(GDX3DPanel panel3D)
   {
      RigidBodyTransform transform = new RigidBodyTransform();
      ReferenceFrame referenceFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(ReferenceFrame.getWorldFrame(), transform);
      create(panel3D, referenceFrame, transform);
   }

   public GDXInteractableRealsenseL515(GDX3DPanel panel3D, ReferenceFrame referenceFrameToRepresent, RigidBodyTransform transformToParentToModify)
   {
      create(panel3D, referenceFrameToRepresent, transformToParentToModify);
   }

   private void create(GDX3DPanel panel3D, ReferenceFrame referenceFrameToRepresent, RigidBodyTransform transformToParentToModify)
   {
      GDXModelInstance l515SensorModel = new GDXModelInstance(GDXModelLoader.load("environmentObjects/l515Sensor/L515Sensor.g3dj"));
      interactableFrameModel.create(referenceFrameToRepresent, transformToParentToModify, panel3D, l515SensorModel, this::calculateClosestCollision);
   }

   private double calculateClosestCollision(Line3DReadOnly mousePickRay)
   {
      cylinderIntersection.setup(0.028, 0.031, offset, Axis3D.X, interactableFrameModel.getReferenceFrame());
      return cylinderIntersection.intersect(mousePickRay);
   }

   public GDXInteractableFrameModel getInteractableFrameModel()
   {
      return interactableFrameModel;
   }
}
