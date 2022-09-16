package us.ihmc.gdx.ui.interactable;

import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.gdx.tools.GDXModelInstance;
import us.ihmc.gdx.tools.GDXModelLoader;
import us.ihmc.gdx.ui.GDX3DPanel;
import us.ihmc.gdx.ui.affordances.GDXInteractableFrameModel;
import us.ihmc.gdx.ui.gizmo.BoxRayIntersection;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;

public class GDXInteractableRealsenseD435
{
   private final GDXInteractableFrameModel interactableFrameModel = new GDXInteractableFrameModel();
   private final BoxRayIntersection boxRayIntersection = new BoxRayIntersection();

   public GDXInteractableRealsenseD435(GDX3DPanel panel3D)
   {
      RigidBodyTransform transform = new RigidBodyTransform();
      ReferenceFrame referenceFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(ReferenceFrame.getWorldFrame(), transform);
      create(panel3D, referenceFrame, transform);
   }

   public GDXInteractableRealsenseD435(GDX3DPanel panel3D, ReferenceFrame referenceFrameToRepresent, RigidBodyTransform transformToParentToModify)
   {
      create(panel3D, referenceFrameToRepresent, transformToParentToModify);
   }

   private void create(GDX3DPanel panel3D, ReferenceFrame referenceFrameToRepresent, RigidBodyTransform transformToParentToModify)
   {
      GDXModelInstance sensorModel = new GDXModelInstance(GDXModelLoader.load("environmentObjects/d435Sensor/D435.g3dj"));
      interactableFrameModel.create(referenceFrameToRepresent, transformToParentToModify, panel3D, sensorModel, this::calculateClosestCollision);
   }

   private double calculateClosestCollision(Line3DReadOnly mousePickRay)
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

   public GDXInteractableFrameModel getInteractableFrameModel()
   {
      return interactableFrameModel;
   }
}
