package us.ihmc.rdx.ui.interactable;

import com.badlogic.gdx.graphics.g3d.model.data.ModelData;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.rdx.tools.RDXModelLoader;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.affordances.RDXInteractableFrameModel;
import us.ihmc.robotics.interaction.BoxRayIntersection;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;

public class RDXInteractableRealsenseD435
{
   private final RDXInteractableFrameModel interactableFrameModel = new RDXInteractableFrameModel();
   private final BoxRayIntersection boxRayIntersection = new BoxRayIntersection();

   public RDXInteractableRealsenseD435(RDX3DPanel panel3D)
   {
      RigidBodyTransform transform = new RigidBodyTransform();
      ReferenceFrame referenceFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(ReferenceFrame.getWorldFrame(), transform);
      create(panel3D, referenceFrame, transform);
   }

   public RDXInteractableRealsenseD435(RDX3DPanel panel3D, ReferenceFrame referenceFrameToRepresent, RigidBodyTransform transformToParentToModify)
   {
      create(panel3D, referenceFrameToRepresent, transformToParentToModify);
   }

   private void create(RDX3DPanel panel3D, ReferenceFrame referenceFrameToRepresent, RigidBodyTransform transformToParentToModify)
   {
      ModelData sensorModel = RDXModelLoader.loadModelData("environmentObjects/d435Sensor/D435.g3dj");
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

   public RDXInteractableFrameModel getInteractableFrameModel()
   {
      return interactableFrameModel;
   }
}
