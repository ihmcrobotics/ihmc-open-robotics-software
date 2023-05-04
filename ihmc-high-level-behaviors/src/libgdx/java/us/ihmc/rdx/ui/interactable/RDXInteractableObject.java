package us.ihmc.rdx.ui.interactable;

import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.graphics.g3d.model.data.ModelData;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.tools.RDXModelLoader;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;

public class RDXInteractableObject implements RenderableProvider
{
   private ModelInstance modelInstance;
   private RigidBodyTransform objectTransform = new RigidBodyTransform();
   private final ReferenceFrame objectFrame;
   private final RDXPose3DGizmo pose3DGizmo = new RDXPose3DGizmo();

   public RDXInteractableObject(RDXBaseUI baseUI)
   {
      this.objectFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(ReferenceFrame.getWorldFrame(), objectTransform);

      pose3DGizmo.create(baseUI.getPrimary3DPanel());
      pose3DGizmo.getTransformToParent().getTranslation().set(objectTransform.getTranslation());
      pose3DGizmo.getTransformToParent().getRotation().set(objectTransform.getRotation());
      pose3DGizmo.update();

      baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(pose3DGizmo::calculate3DViewPick);
      baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(pose3DGizmo::process3DViewInput);
      baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(this::update);
      baseUI.getPrimaryScene().addRenderableProvider(pose3DGizmo);
      baseUI.getImGuiPanelManager().addPanel(pose3DGizmo.createTunerPanel("object"));
   }

   public void load(String modelFileName)
   {
      ModelData objectModel = RDXModelLoader.loadModelData(modelFileName);
      this.modelInstance = new RDXModelInstance(new Model(objectModel));
   }

   public void clear()
   {
      modelInstance = null;
   }

   public void update(ImGui3DViewInput input)
   {
      pose3DGizmo.update();
      objectTransform.set(pose3DGizmo.getTransformToParent());
      objectFrame.update();
      if (modelInstance != null)
         LibGDXTools.toLibGDX(objectFrame.getTransformToRoot(), modelInstance.transform);
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (modelInstance != null)
         modelInstance.getRenderables(renderables, pool);

      if (pose3DGizmo != null)
         pose3DGizmo.getRenderables(renderables, pool);
   }

   public RigidBodyTransform getTransformToWorld()
   {
      return pose3DGizmo.getTransformToWorld();
   }

   public ModelInstance getModelInstance()
   {
      return modelInstance;
   }

   public RDXPose3DGizmo getPose3DGizmo()
   {
      return pose3DGizmo;
   }

   public ReferenceFrame getObjectFrame()
   {
      return objectFrame;
   }
}
