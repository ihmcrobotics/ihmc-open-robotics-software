package us.ihmc.rdx.behaviorTree.scene;

import behavior_msgs.BehaviorTreeSceneObjectDefinitionMessage;
import behavior_msgs.BehaviorTreeSceneObjectStateMessage;
import behavior_msgs.PersistentDetectionStatusMessage;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.tools.RDXModelLoader;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;
import us.ihmc.scs2.definition.visual.ColorDefinitions;

/**
 * Renders a door panel in the right spot.
 */
public class RDXBehaviorTreeSceneDoorPanel extends RDXBehaviorTreeSceneObject
{
   private final PersistentDetectionStatusMessage doorPanelPersistentDetection = new PersistentDetectionStatusMessage();
   private final ReferenceFrame detectionFrame
         = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(ReferenceFrame.getWorldFrame(), transform.getValueUnsafe());
   private final RigidBodyTransform modelOffset = new RigidBodyTransform();
   private final ReferenceFrame modelFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(detectionFrame, modelOffset);

   public RDXBehaviorTreeSceneDoorPanel(long id, CRDTInfo crdtInfo, BehaviorTreeSceneObjectDefinitionMessage definition, RDXBaseUI baseUI)
   {
      super(id, crdtInfo, definition, baseUI);

      model = RDXModelLoader.load("environmentObjects/door_panel/door_panel.glb");
      modelInstance = new RDXModelInstance(model);
      modelInstance.setColor(ColorDefinitions.GreenYellow());
      modelInstance.setOpacity(0.3f);
      modelOffset.setToZero();
      modelOffset.appendTranslation(0.4, 0.0, 0.0); // Tune the door to roughly match the real one, for visual reference only
   }

   @Override
   public void update()
   {
      super.update();

      detectionFrame.update();
      modelFrame.update();

      modelInstance.setTransformToReferenceFrame(modelFrame);
   }

   @Override
   public void renderDetectionInfo()
   {
      RDXBehaviorTreeScene.renderPersistentDetection(getPersistentDetection());
      RDXBehaviorTreeScene.renderPersistentDetection(getDoorPanelPersistentDetection());
   }

   @Override
   public void fromMessage(BehaviorTreeSceneObjectStateMessage message)
   {
      super.fromMessage(message);

      doorPanelPersistentDetection.set(message.getDoorPanelDetection());
   }

   public PersistentDetectionStatusMessage getDoorPanelPersistentDetection()
   {
      return doorPanelPersistentDetection;
   }
}
