package us.ihmc.rdx.behaviorTree.scene;

import com.badlogic.gdx.graphics.g3d.Model;
import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneObject;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.rdx.sceneManager.RDXRenderableAdapter;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.tools.RDXModelLoader;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.gizmo.RDXSelectablePose3DGizmo;

public class RDXBehaviorTreeSceneObject extends BehaviorTreeSceneObject
{
   private final RDXBaseUI baseUI;

   private final RDXSelectablePose3DGizmo gizmo;
   private final Model model;
   private final RDXModelInstance modelInstance;
   private final RDXRenderableAdapter modelRenderableAdapter;

   // TODO: Instead give an object type with the object name stored centrally
   public RDXBehaviorTreeSceneObject(String modelName, RDXBaseUI baseUI)
   {
      this.baseUI = baseUI;
      gizmo = new RDXSelectablePose3DGizmo(transform, ReferenceFrame.getWorldFrame());

      gizmo.createAndSetupDefault(baseUI.getPrimary3DPanel());
      gizmo.setSelected(true);

      model = RDXModelLoader.load(modelName);
      modelInstance = new RDXModelInstance(model);
      modelRenderableAdapter = baseUI.getPrimaryScene().addModelInstance(modelInstance);
   }

   public void update()
   {
      modelInstance.setTransformToWorldFrame(transform);
   }

   public void destroy()
   {
      baseUI.getPrimaryScene().removeRenderableAdapter(modelRenderableAdapter);
      gizmo.setSelected(false);
      gizmo.destroyDefault(baseUI.getPrimary3DPanel());
      model.dispose();
   }
}
