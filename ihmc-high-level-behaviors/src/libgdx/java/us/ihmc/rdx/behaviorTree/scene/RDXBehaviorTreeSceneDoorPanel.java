package us.ihmc.rdx.behaviorTree.scene;

import behavior_msgs.msg.dds.BehaviorTreeSceneObjectDefinitionMessage;
import behavior_msgs.msg.dds.BehaviorTreeSceneObjectStateMessage;
import behavior_msgs.msg.dds.PersistentDetectionStatusMessage;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.rdx.tools.RDXModelInstance;
import us.ihmc.rdx.tools.RDXModelLoader;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;

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
   }

   @Override
   public void update()
   {
      super.update();

      modelOffset.getRotation().setToPitchOrientation(Math.PI / 2.0); // TODO Tune this

      detectionFrame.update();
      modelFrame.update();

      modelInstance.setTransformToReferenceFrame(modelFrame);
   }

   @Override
   public void fromMessage(BehaviorTreeSceneObjectStateMessage message)
   {
      super.fromMessage(message);

      doorPanelPersistentDetection.set(message.getPersistentDetection());
   }

   public PersistentDetectionStatusMessage getDoorPanelPersistentDetection()
   {
      return doorPanelPersistentDetection;
   }
}
