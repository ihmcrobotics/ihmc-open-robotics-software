package us.ihmc.behaviors.behaviorTree.ros2;

import behavior_msgs.msg.dds.*;
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
   private CheckPointNodeStateMessage checkPointNodeStateMessage;
   private SceneActionNodeStateMessage sceneActionNodeStateMessage;
   private AI2RNodeStateMessage ai2rNodeStateMessage;
   private DoorTraversalStateMessage doorTraversalStateMessage;
   private BuildingExplorationStateMessage buildingExplorationStateMessage;
   private LeafNodeStateMessage leafNodeStateMessage;
   private ActionNodeStateMessage actionNodeStateMessage;
   private NeckActionStateMessage neckActionStateMessage;
   private SpineActionStateMessage spineActionStateMessage;
   private WalkActionStateMessage walkActionStateMessage;
   private AbilityHandActionStateMessage abilityHandActionStateMessage;
   private SakeHandCommandActionStateMessage sakeHandCommandActionStateMessage;
   private ArmActionStateMessage armActionStateMessage;
   private HandWrenchActionStateMessage handWrenchActionStateMessage;
   private ScrewPrimitiveActionStateMessage screwPrimitiveActionStateMessage;
   private PelvisActionStateMessage pelvisActionStateMessage;
   private WaitDurationActionStateMessage waitDurationActionStateMessage;
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
      checkPointNodeStateMessage = null;
      sceneActionNodeStateMessage = null;
      ai2rNodeStateMessage = null;
      doorTraversalStateMessage = null;
      buildingExplorationStateMessage = null;
      leafNodeStateMessage = null;
      actionNodeStateMessage = null;
      neckActionStateMessage = null;
      spineActionStateMessage = null;
      walkActionStateMessage = null;
      abilityHandActionStateMessage = null;
      sakeHandCommandActionStateMessage = null;
      armActionStateMessage = null;
      handWrenchActionStateMessage = null;
      screwPrimitiveActionStateMessage = null;
      pelvisActionStateMessage = null;
      waitDurationActionStateMessage = null;
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

   public CheckPointNodeStateMessage getCheckPointNodeStateMessage()
   {
      return checkPointNodeStateMessage;
   }

   public void setCheckPointNodeStateMessage(CheckPointNodeStateMessage checkPointNodeStateMessage)
   {
      this.checkPointNodeStateMessage = checkPointNodeStateMessage;
   }

   public SceneActionNodeStateMessage getSceneActionNodeStateMessage()
   {
      return sceneActionNodeStateMessage;
   }

   public void setSceneActionNodeStateMessage(SceneActionNodeStateMessage sceneActionNodeStateMessage)
   {
      this.sceneActionNodeStateMessage = sceneActionNodeStateMessage;
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

   public SakeHandCommandActionStateMessage getSakeHandCommandActionStateMessage()
   {
      return sakeHandCommandActionStateMessage;
   }

   public void setSakeHandCommandActionStateMessage(SakeHandCommandActionStateMessage sakeHandCommandActionStateMessage)
   {
      this.sakeHandCommandActionStateMessage = sakeHandCommandActionStateMessage;
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

   public WaitDurationActionStateMessage getWaitDurationActionStateMessage()
   {
      return waitDurationActionStateMessage;
   }

   public void setWaitDurationActionStateMessage(WaitDurationActionStateMessage waitDurationActionStateMessage)
   {
      this.waitDurationActionStateMessage = waitDurationActionStateMessage;
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
