package us.ihmc.rdx.behaviorTree.scene;

import behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessage;
import behavior_msgs.msg.dds.BehaviorTreeSceneObjectStateMessage;
import behavior_msgs.msg.dds.PersistentDetectionStatusMessage;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneObjectState;
import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneObjectType;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.rdx.behaviorTree.RDXCRDTTools;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.tools.RDXModelLoader;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.gizmo.RDXSelectablePose3DGizmo;

public class RDXBehaviorTreeSceneObject extends BehaviorTreeSceneObjectState
{
   private final RDXBaseUI baseUI;

   private final RDXSelectablePose3DGizmo gizmo;
   protected Model model;
   protected RDXModelInstance modelInstance;

   private final PersistentDetectionStatusMessage persistentDetection = new PersistentDetectionStatusMessage();

   public RDXBehaviorTreeSceneObject(long id, CRDTInfo crdtInfo, BehaviorTreeSceneObjectDefinitionMessage definition, RDXBaseUI baseUI)
   {
      super(id, crdtInfo, definition);

      this.baseUI = baseUI;

      gizmo = new RDXSelectablePose3DGizmo();
      gizmo.createAndSetupDefault(baseUI.getPrimary3DPanel());
      gizmo.getPoseGizmo().setGizmoFrame(referenceFrame);

      if (getObjectType() == BehaviorTreeSceneObjectType.FOUNDATION_POSE)
      {
         String modelName = getFoundationPoseObjectType().meshDirectory;
         String modelPath = "environmentObjects/" + modelName + "/" + modelName + ".glb";
         model = RDXModelLoader.load(modelPath);
         modelInstance = new RDXModelInstance(model);
      }
   }

   public void update()
   {
      RDXCRDTTools.syncGizmoWithBidirectionalField(gizmo.getPoseGizmo(), transform, this);
      if (model != null)
         modelInstance.setTransformToWorldFrame(transform.getValueUnsafe());
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (model != null)
         modelInstance.getRenderables(renderables, pool);
   }

   public void destroy()
   {
      gizmo.setSelected(false);
      gizmo.destroyDefault(baseUI.getPrimary3DPanel());
      if (model != null)
         model.dispose();
   }

   @Override
   public void fromMessage(BehaviorTreeSceneObjectStateMessage message)
   {
      super.fromMessage(message);

      persistentDetection.set(message.getPersistentDetection());
   }

   public RDXSelectablePose3DGizmo getGizmo()
   {
      return gizmo;
   }

   public PersistentDetectionStatusMessage getPersistentDetection()
   {
      return persistentDetection;
   }
}
