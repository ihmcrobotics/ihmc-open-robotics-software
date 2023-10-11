package us.ihmc.rdx.ui.interactable;

import com.badlogic.gdx.graphics.g3d.model.data.ModelData;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.rdx.tools.RDXModelLoader;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.affordances.RDXInteractableFrameModel;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.robotics.interaction.BoxRayIntersection;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;
import us.ihmc.scs2.definition.visual.ColorDefinition;

public class RDXInteractableBumper implements RDXInteractableAffordanceTemplateHand
{
   private final RDXInteractableFrameModel interactableHandFrameModel = new RDXInteractableFrameModel();
   private final ReferenceFrame referenceFrameHand;
   private final BoxRayIntersection boxRayIntersection = new BoxRayIntersection();

   public RDXInteractableBumper(RDX3DPanel panel3D, RigidBodyTransform transformToParentToModify, ColorDefinition color)
   {
      this.referenceFrameHand = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(ReferenceFrame.getWorldFrame(),
                                                                                                       transformToParentToModify);
      ModelData handModel = RDXModelLoader.loadModelData("environmentObjects/bumperCycloidalArms/bumper.g3dj");
      interactableHandFrameModel.create(referenceFrameHand, transformToParentToModify, panel3D, handModel, this::calculateClosestCollision);
      interactableHandFrameModel.getModelInstance().setColor(color);
   }

   private double calculateClosestCollision(Line3DReadOnly mousePickRay)
   {
      RigidBodyTransform intersectionHandOrigin = new RigidBodyTransform(interactableHandFrameModel.getReferenceFrame().getTransformToWorldFrame());
      intersectionHandOrigin.getTranslation()
                            .set(new Vector3D(intersectionHandOrigin.getTranslationX(),
                                              intersectionHandOrigin.getTranslationY(),
                                              interactableHandFrameModel.getReferenceFrame().getTransformToWorldFrame().getTranslationZ() + 0.02));
      if (boxRayIntersection.intersect(0.06, 0.06, 0.05, intersectionHandOrigin, mousePickRay))
      {
         return mousePickRay.getPoint().distance(boxRayIntersection.getFirstIntersectionToPack());
      }
      else
      {
         return Double.NaN;
      }
   }

   @Override
   public RDXPose3DGizmo getPose3DGizmo()
   {
      return interactableHandFrameModel.getPoseGizmo();
   }

   @Override
   public ReferenceFrame getReferenceFrameHand()
   {
      return referenceFrameHand;
   }

   @Override
   public boolean isSelected()
   {
      return interactableHandFrameModel.isSelected();
   }

   @Override
   public void setSelected(boolean selected)
   {
      interactableHandFrameModel.setSelected(selected);
   }

   @Override
   public void setShowing(boolean showing)
   {
      interactableHandFrameModel.setShowing(showing);
   }
}
