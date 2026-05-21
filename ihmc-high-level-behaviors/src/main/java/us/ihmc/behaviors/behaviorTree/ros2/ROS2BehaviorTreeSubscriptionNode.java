package us.ihmc.behaviors.behaviorTree.ros2;

import behavior_msgs.*;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeDefinitionRegistry;

import java.util.ArrayList;
import java.util.List;

public class ROS2BehaviorTreeSubscriptionNode
{
   private long sequenceId;
   private byte packedMessageType;
   private Class<?> nodeDefinitionClass;
   private BehaviorTreeNodeDefinitionMessage behaviorTreeNodeDefinitionMessage;
   private BehaviorTreeNodeStateMessage behaviorTreeNodeStateMessage;
   private BehaviorTreeRootNodeStateMessage behaviorTreeRootNodeStateMessage;
   private ActionSequenceStateMessage actionSequenceStateMessage;
   private FallbackNodeStateMessage fallbackNodeStateMessage;
   private ConditionNodeStateMessage conditionNodeStateMessage;
   private GotoNodeStateMessage gotoNodeStateMessage;
   private CheckpointNodeStateMessage checkpointNodeStateMessage;
   private SceneActionStateMessage sceneActionStateMessage;
   private MimicActionStateMessage mimicActionStateMessage;
   private AI2RNodeStateMessage ai2rNodeStateMessage;
   private DoorTraversalStateMessage doorTraversalStateMessage;
   private BuildingExplorationStateMessage buildingExplorationStateMessage;
   private LeafNodeStateMessage leafNodeStateMessage;
   private ActionNodeStateMessage actionNodeStateMessage;
   private NeckActionStateMessage neckActionStateMessage;
   private SpineActionStateMessage spineActionStateMessage;
   private WalkActionStateMessage walkActionStateMessage;
   private AbilityHandActionStateMessage abilityHandActionStateMessage;
   private EZGripperActionStateMessage ezGripperActionStateMessage;
   private ArmActionStateMessage armActionStateMessage;
   private HandWrenchActionStateMessage handWrenchActionStateMessage;
   private ScrewPrimitiveActionStateMessage screwPrimitiveActionStateMessage;
   private PelvisActionStateMessage pelvisActionStateMessage;
   private WaitActionStateMessage waitActionStateMessage;
   private LegActionStateMessage legActionStateMessage;
   private final List<ROS2BehaviorTreeSubscriptionNode> children = new ArrayList<>();

   public void clear()
   {
      sequenceId = -1;
      packedMessageType = -1;
      nodeDefinitionClass = null;
      behaviorTreeNodeDefinitionMessage = null;
      behaviorTreeNodeStateMessage = null;
      behaviorTreeRootNodeStateMessage = null;
      actionSequenceStateMessage = null;
      fallbackNodeStateMessage = null;
      conditionNodeStateMessage = null;
      gotoNodeStateMessage = null;
      checkpointNodeStateMessage = null;
      sceneActionStateMessage = null;
      mimicActionStateMessage = null;
      ai2rNodeStateMessage = null;
      doorTraversalStateMessage = null;
      buildingExplorationStateMessage = null;
      leafNodeStateMessage = null;
      actionNodeStateMessage = null;
      neckActionStateMessage = null;
      spineActionStateMessage = null;
      walkActionStateMessage = null;
      abilityHandActionStateMessage = null;
      ezGripperActionStateMessage = null;
      armActionStateMessage = null;
      handWrenchActionStateMessage = null;
      screwPrimitiveActionStateMessage = null;
      pelvisActionStateMessage = null;
      waitActionStateMessage = null;
      legActionStateMessage = null;
      children.clear();
   }

   public long getSequenceId()
   {
      return sequenceId;
   }

   public void setSequenceId(long sequenceId)
   {
      this.sequenceId = sequenceId;
   }

   public byte getPackedType()
   {
      return packedMessageType;
   }

   public void setPackedType(byte type)
   {
      packedMessageType = type;
   }

   public Class<?> getDefinitionClass()
   {
      return nodeDefinitionClass;
   }

   public BehaviorTreeNodeDefinitionMessage getBehaviorTreeNodeDefinitionMessage()
   {
      return behaviorTreeNodeDefinitionMessage;
   }

   public void setBehaviorTreeNodeDefinitionMessage(BehaviorTreeNodeDefinitionMessage behaviorTreeNodeDefinitionMessage)
   {
      this.behaviorTreeNodeDefinitionMessage = behaviorTreeNodeDefinitionMessage;
      nodeDefinitionClass = BehaviorTreeDefinitionRegistry.getNodeDefinitionClass(behaviorTreeNodeDefinitionMessage.getType());
   }

   public BehaviorTreeNodeStateMessage getBehaviorTreeNodeStateMessage()
   {
      return behaviorTreeNodeStateMessage;
   }

   public void setBehaviorTreeNodeStateMessage(BehaviorTreeNodeStateMessage behaviorTreeNodeStateMessage)
   {
      this.behaviorTreeNodeStateMessage = behaviorTreeNodeStateMessage;
   }

   public BehaviorTreeRootNodeStateMessage getBehaviorTreeRootNodeStateMessage()
   {
      return behaviorTreeRootNodeStateMessage;
   }

   public void setBehaviorTreeRootNodeStateMessage(BehaviorTreeRootNodeStateMessage behaviorTreeRootNodeStateMessage)
   {
      this.behaviorTreeRootNodeStateMessage = behaviorTreeRootNodeStateMessage;
   }

   public LeafNodeStateMessage getLeafNodeStateMessage()
   {
      return leafNodeStateMessage;
   }

   public void setLeafNodeStateMessage(LeafNodeStateMessage leafNodeStateMessage)
   {
      this.leafNodeStateMessage = leafNodeStateMessage;
   }

   public ActionNodeStateMessage getActionNodeStateMessage()
   {
      return actionNodeStateMessage;
   }

   public void setActionNodeStateMessage(ActionNodeStateMessage actionNodeStateMessage)
   {
      this.actionNodeStateMessage = actionNodeStateMessage;
   }

   public ActionSequenceStateMessage getActionSequenceStateMessage()
   {
      return actionSequenceStateMessage;
   }

   public void setActionSequenceStateMessage(ActionSequenceStateMessage actionSequenceStateMessage)
   {
      this.actionSequenceStateMessage = actionSequenceStateMessage;
   }

   public FallbackNodeStateMessage getFallbackNodeStateMessage()
   {
      return fallbackNodeStateMessage;
   }

   public void setFallbackNodeStateMessage(FallbackNodeStateMessage fallbackNodeStateMessage)
   {
      this.fallbackNodeStateMessage = fallbackNodeStateMessage;
   }

   public ConditionNodeStateMessage getConditionNodeStateMessage()
   {
      return conditionNodeStateMessage;
   }

   public void setConditionNodeStateMessage(ConditionNodeStateMessage conditionNodeStateMessage)
   {
      this.conditionNodeStateMessage = conditionNodeStateMessage;
   }

   public GotoNodeStateMessage getGotoNodeStateMessage()
   {
      return gotoNodeStateMessage;
   }

   public void setGotoNodeStateMessage(GotoNodeStateMessage gotoNodeStateMessage)
   {
      this.gotoNodeStateMessage = gotoNodeStateMessage;
   }

   public CheckpointNodeStateMessage getCheckpointNodeStateMessage()
   {
      return checkpointNodeStateMessage;
   }

   public void setCheckpointNodeStateMessage(CheckpointNodeStateMessage checkpointNodeStateMessage)
   {
      this.checkpointNodeStateMessage = checkpointNodeStateMessage;
   }

   public SceneActionStateMessage getSceneActionStateMessage()
   {
      return sceneActionStateMessage;
   }

   public void setSceneActionStateMessage(SceneActionStateMessage sceneActionStateMessage)
   {
      this.sceneActionStateMessage = sceneActionStateMessage;
   }

   public MimicActionStateMessage getMimicActionStateMessage()
   {
      return mimicActionStateMessage;
   }

   public void setMimicActionStateMessage(MimicActionStateMessage mimicActionStateMessage)
   {
      this.mimicActionStateMessage = mimicActionStateMessage;
   }

   public AI2RNodeStateMessage getAI2RNodeStateMessage()
   {
      return ai2rNodeStateMessage;
   }

   public void setAI2RNodeStateMessage(AI2RNodeStateMessage ai2rNodeStateMessage)
   {
      this.ai2rNodeStateMessage = ai2rNodeStateMessage;
   }

   public DoorTraversalStateMessage getDoorTraversalStateMessage()
   {
      return doorTraversalStateMessage;
   }

   public void setDoorTraversalStateMessage(DoorTraversalStateMessage doorTraversalStateMessage)
   {
      this.doorTraversalStateMessage = doorTraversalStateMessage;
   }

   public BuildingExplorationStateMessage getBuildingExplorationStateMessage()
   {
      return buildingExplorationStateMessage;
   }

   public void setBuildingExplorationStateMessage(BuildingExplorationStateMessage buildingExplorationStateMessage)
   {
      this.buildingExplorationStateMessage = buildingExplorationStateMessage;
   }

   public NeckActionStateMessage getNeckActionStateMessage()
   {
      return neckActionStateMessage;
   }

   public void setNeckActionStateMessage(NeckActionStateMessage neckActionStateMessage)
   {
      this.neckActionStateMessage = neckActionStateMessage;
   }

   public SpineActionStateMessage getSpineActionStateMessage()
   {
      return spineActionStateMessage;
   }

   public void setSpineActionStateMessage(SpineActionStateMessage spineActionStateMessage)
   {
      this.spineActionStateMessage = spineActionStateMessage;
   }

   public WalkActionStateMessage getWalkActionStateMessage()
   {
      return walkActionStateMessage;
   }

   public void setWalkActionStateMessage(WalkActionStateMessage walkActionStateMessage)
   {
      this.walkActionStateMessage = walkActionStateMessage;
   }

   public AbilityHandActionStateMessage getAbilityHandActionStateMessage()
   {
      return abilityHandActionStateMessage;
   }

   public void setAbilityHandActionStateMessage(AbilityHandActionStateMessage abilityHandActionStateMessage)
   {
      this.abilityHandActionStateMessage = abilityHandActionStateMessage;
   }

   public EZGripperActionStateMessage getEZGripperActionStateMessage()
   {
      return ezGripperActionStateMessage;
   }

   public void setEZGripperActionStateMessage(EZGripperActionStateMessage ezGripperActionStateMessage)
   {
      this.ezGripperActionStateMessage = ezGripperActionStateMessage;
   }

   public ArmActionStateMessage getArmActionStateMessage()
   {
      return armActionStateMessage;
   }

   public void setArmActionStateMessage(ArmActionStateMessage armActionStateMessage)
   {
      this.armActionStateMessage = armActionStateMessage;
   }

   public HandWrenchActionStateMessage getHandWrenchActionStateMessage()
   {
      return handWrenchActionStateMessage;
   }

   public void setHandWrenchActionStateMessage(HandWrenchActionStateMessage handWrenchActionStateMessage)
   {
      this.handWrenchActionStateMessage = handWrenchActionStateMessage;
   }

   public ScrewPrimitiveActionStateMessage getScrewPrimitiveActionStateMessage()
   {
      return screwPrimitiveActionStateMessage;
   }

   public void setScrewPrimitiveActionStateMessage(ScrewPrimitiveActionStateMessage screwPrimitiveActionStateMessage)
   {
      this.screwPrimitiveActionStateMessage = screwPrimitiveActionStateMessage;
   }

   public PelvisActionStateMessage getPelvisActionStateMessage()
   {
      return pelvisActionStateMessage;
   }

   public void setPelvisActionStateMessage(PelvisActionStateMessage pelvisActionStateMessage)
   {
      this.pelvisActionStateMessage = pelvisActionStateMessage;
   }

   public WaitActionStateMessage getWaitActionStateMessage()
   {
      return waitActionStateMessage;
   }

   public void setWaitActionStateMessage(WaitActionStateMessage waitActionStateMessage)
   {
      this.waitActionStateMessage = waitActionStateMessage;
   }

   public LegActionStateMessage getLegActionStateMessage()
   {
      return legActionStateMessage;
   }

   public void setLegActionStateMessage(LegActionStateMessage legActionStateMessage)
   {
      this.legActionStateMessage = legActionStateMessage;
   }

   public List<ROS2BehaviorTreeSubscriptionNode> getChildren()
   {
      return children;
   }
}
