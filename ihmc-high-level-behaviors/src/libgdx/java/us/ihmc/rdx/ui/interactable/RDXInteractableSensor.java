package us.ihmc.rdx.ui.interactable;

import com.badlogic.gdx.graphics.g3d.model.data.ModelData;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.rdx.tools.RDXModelLoader;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.affordances.RDXInteractableFrameModel;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;

public abstract class RDXInteractableSensor
{
   protected final RDXInteractableFrameModel interactableFrameModel = new RDXInteractableFrameModel();

   public RDXInteractableSensor(RDX3DPanel panel3D, String modelPath)
   {
      RigidBodyTransform transform = new RigidBodyTransform();
      ReferenceFrame referenceFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(ReferenceFrame.getWorldFrame(), transform);
      create(panel3D, referenceFrame, transform, modelPath);
   }

   public RDXInteractableSensor(RDX3DPanel panel3D, ReferenceFrame referenceFrameToRepresent, RigidBodyTransform transformToParentToModify, String modelPath)
   {
      create(panel3D, referenceFrameToRepresent, transformToParentToModify, modelPath);
   }

   private void create(RDX3DPanel panel3D, ReferenceFrame referenceFrameToRepresent, RigidBodyTransform transformToParentToModify, String modelPath)
   {
      ModelData sensorModel = RDXModelLoader.loadModelData(modelPath);
      interactableFrameModel.create(referenceFrameToRepresent, transformToParentToModify, panel3D, sensorModel, this::calculateClosestCollision);
   }

   protected abstract double calculateClosestCollision(Line3DReadOnly mousePickRay);

   public RDXInteractableFrameModel getInteractableFrameModel()
   {
      return interactableFrameModel;
   }

   public void setOpacity(float opacity)
   {
      interactableFrameModel.getModelInstance().setOpacity(opacity);
   }
}
