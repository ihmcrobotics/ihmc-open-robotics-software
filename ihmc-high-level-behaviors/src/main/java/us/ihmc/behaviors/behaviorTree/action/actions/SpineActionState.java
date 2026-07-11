package us.ihmc.behaviors.behaviorTree.action.actions;

import behavior_msgs.SpineActionStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeState;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeState;
import us.ihmc.communication.crdt.CRDTDetachableReferenceFrame;
import us.ihmc.communication.crdt.CRDTStatusRigidBodyTransform;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;

public class SpineActionState extends ActionNodeState<SpineActionDefinition>
{
   private final CRDTDetachableReferenceFrame chestFrame;
   /**
    * This is the estimated goal pelvis frame as the robot executes a potential whole body action.
    * This is used to compute joint angles that achieve the desired and previewed end pose
    * even when the pelvis and/or chest might also move.
    */
   private final CRDTStatusRigidBodyTransform goalPelvisToWorldTransform;
   private final ReferenceFrame goalPelvisFrame;

   public SpineActionState(long id, BehaviorTreeRootNodeState rootNode)
   {
      super(id, new SpineActionDefinition(rootNode.getDefinition()), rootNode);

      chestFrame = new CRDTDetachableReferenceFrame(scene::findFrameByName,
                                                    definition.getCRDTParentFrameName(),
                                                    definition.getChestToParentTransform());
      goalPelvisToWorldTransform = new CRDTStatusRigidBodyTransform(ROS2ActorDesignation.ROBOT, crdtInfo);
      goalPelvisFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(ReferenceFrame.getWorldFrame(),
                                                                                               goalPelvisToWorldTransform.getValueReadOnly());
   }

   @Override
   public void update()
   {
      chestFrame.update();
   }

   @Override
   public boolean hasStatus()
   {
      boolean hasStatus = super.hasStatus();
      hasStatus |= goalPelvisToWorldTransform.pollHasStatus();
      return hasStatus;
   }

   public void toMessage(SpineActionStateMessage message)
   {
      definition.toMessage(message.getDefinition());

      super.toMessage(message.getState());

      goalPelvisToWorldTransform.toMessage(message.getGoalPelvisTransformToWorld());
   }

   public void fromMessage(SpineActionStateMessage message)
   {
      definition.fromMessage(message.getDefinition());

      super.fromMessage(message.getState());

      goalPelvisToWorldTransform.fromMessage(message.getGoalPelvisTransformToWorld());
      goalPelvisFrame.update();
   }

   public CRDTDetachableReferenceFrame getChestFrame()
   {
      return chestFrame;
   }

   public CRDTStatusRigidBodyTransform getGoalPelvisToWorldTransform()
   {
      return goalPelvisToWorldTransform;
   }

   public ReferenceFrame getGoalPelvisFrame()
   {
      return goalPelvisFrame;
   }
}
