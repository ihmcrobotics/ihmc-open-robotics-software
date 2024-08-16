package us.ihmc.rdx.ui.interactable;

import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.graphics.g3d.model.data.ModelData;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.tools.RDXModelLoader;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.gizmo.RDXSelectablePose3DGizmo;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;

public class RDXInteractableObject implements RenderableProvider
{
   private RDXModelInstance modelInstance;
   private RigidBodyTransform objectTransform = new RigidBodyTransform();
   private final FramePose3D initialObjectPose = new FramePose3D();
   private final ReferenceFrame objectFrame;
   private ReferenceFrame modelInstanceFrame;
   private RDXSelectablePose3DGizmo selectablePose3DGizmo = new RDXSelectablePose3DGizmo(ReferenceFrame.getWorldFrame(), objectTransform);

   public RDXInteractableObject(RDXBaseUI baseUI)
   {
      this.objectFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(ReferenceFrame.getWorldFrame(), objectTransform);

      selectablePose3DGizmo.setParentFrame(ReferenceFrame.getWorldFrame());
      selectablePose3DGizmo.createAndSetupDefault(baseUI.getPrimary3DPanel());
      selectablePose3DGizmo.getTransformToParent().getTranslation().set(objectTransform.getTranslation());
      selectablePose3DGizmo.getTransformToParent().getRotation().set(objectTransform.getRotation());

      baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(selectablePose3DGizmo::calculate3DViewPick);
      baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(selectablePose3DGizmo::process3DViewInput);
      baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(this::update);
      baseUI.getImGuiPanelManager().addPanel(selectablePose3DGizmo.createTunerPanel("object"));
   }

   public void load(String modelFileName)
   {
      ModelData objectModel = RDXModelLoader.loadModelData(modelFileName);
      modelInstance = new RDXModelInstance(new Model(objectModel));
      selectablePose3DGizmo.getSelected().set(true);
   }

   public void load(String modelFileName, RigidBodyTransform visualModelTransform)
   {
      ModelData objectModel = RDXModelLoader.loadModelData(modelFileName);
      modelInstance = new RDXModelInstance(new Model(objectModel));
      modelInstanceFrame = ReferenceFrameMissingTools.constructFrameWithUnchangingTransformToParent(objectFrame, visualModelTransform);
      selectablePose3DGizmo.getSelected().set(true);
   }

   public void clear()
   {
      modelInstance = null;
      selectablePose3DGizmo.getSelected().set(false);
   }

   public void update(ImGui3DViewInput input)
   {
      selectablePose3DGizmo.update();
      objectTransform.set(selectablePose3DGizmo.getTransformToParent());
      objectFrame.update();
      if (modelInstance != null)
         LibGDXTools.toLibGDX(modelInstanceFrame.getTransformToRoot(), modelInstance.transform);
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (modelInstance != null)
         modelInstance.getRenderables(renderables, pool);

      if (selectablePose3DGizmo.getSelected().get())
         selectablePose3DGizmo.getVirtualRenderables(renderables, pool);
   }

   public RigidBodyTransform getTransformToWorld()
   {
      return objectTransform;
   }

   public FramePose3D getInitialPose()
   {
      return initialObjectPose;
   }

   public RigidBodyTransform getInitialTransformToWorld()
   {
      RigidBodyTransform initialObjectTransformToWorld = new RigidBodyTransform();
      initialObjectPose.get(initialObjectTransformToWorld);
      return initialObjectTransformToWorld;
   }

   public ModelInstance getModelInstance()
   {
      return modelInstance;
   }

   public void setInitialPose()
   {
      initialObjectPose.set(selectablePose3DGizmo.getTransformToParent());
   }

   public void setPose(RigidBodyTransform transformToWorld)
   {
      initialObjectPose.set(transformToWorld);
      selectablePose3DGizmo.getTransformToParent().set(initialObjectPose);
   }

   public void resetToInitialPose()
   {
      selectablePose3DGizmo.getTransformToParent().set(initialObjectPose);
   }

   public void resetPose()
   {
      selectablePose3DGizmo.getTransformToParent().setToZero();
      initialObjectPose.setToZero();
   }

   public void switchGizmoVisualization()
   {
      if (selectablePose3DGizmo.getSelected().get())
         selectablePose3DGizmo.getSelected().set(false);
      else
         selectablePose3DGizmo.getSelected().set(true);
   }
}