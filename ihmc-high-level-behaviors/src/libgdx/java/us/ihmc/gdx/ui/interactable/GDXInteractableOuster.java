package us.ihmc.gdx.ui.interactable;

import com.badlogic.gdx.graphics.g3d.model.data.ModelData;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.gdx.tools.GDXModelLoader;
import us.ihmc.gdx.ui.GDX3DPanel;
import us.ihmc.gdx.ui.affordances.GDXInteractableFrameModel;
import us.ihmc.gdx.ui.gizmo.CylinderRayIntersection;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;

public class GDXInteractableOuster
{
   private final GDXInteractableFrameModel interactableFrameModel = new GDXInteractableFrameModel();
   private final CylinderRayIntersection cylinderIntersection = new CylinderRayIntersection();

   public GDXInteractableOuster(GDX3DPanel panel3D)
   {
      RigidBodyTransform transform = new RigidBodyTransform();
      ReferenceFrame referenceFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(ReferenceFrame.getWorldFrame(), transform);
      create(panel3D, referenceFrame, transform);
   }

   public GDXInteractableOuster(GDX3DPanel panel3D, ReferenceFrame referenceFrameToRepresent, RigidBodyTransform transformToParentToModify)
   {
      create(panel3D, referenceFrameToRepresent, transformToParentToModify);
   }

   private void create(GDX3DPanel panel3D, ReferenceFrame referenceFrameToRepresent, RigidBodyTransform transformToParentToModify)
   {
      ModelData sensorModel = GDXModelLoader.loadModelData("environmentObjects/ousterSensor/Ouster.g3dj");
      interactableFrameModel.create(referenceFrameToRepresent, transformToParentToModify, panel3D, sensorModel, this::calculateClosestCollision);
   }

   private double calculateClosestCollision(Line3DReadOnly mousePickRay)
   {
      cylinderIntersection.setup(0.0734, 0.04, -0.0372, interactableFrameModel.getReferenceFrame().getTransformToWorldFrame());
      return cylinderIntersection.intersect(mousePickRay);
   }

   public GDXInteractableFrameModel getInteractableFrameModel()
   {
      return interactableFrameModel;
   }
}
