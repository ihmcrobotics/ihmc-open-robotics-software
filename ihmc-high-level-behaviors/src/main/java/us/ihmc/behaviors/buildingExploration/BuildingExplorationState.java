package us.ihmc.behaviors.buildingExploration;

import behavior_msgs.msg.dds.BuildingExplorationStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeState;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeState;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeTools;
import us.ihmc.behaviors.sequence.ActionNodeState;
import us.ihmc.behaviors.sequence.actions.WaitDurationActionState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

import java.util.ArrayList;
import java.util.List;

public class BuildingExplorationState extends BehaviorTreeNodeState<BuildingExplorationDefinition>
{
   public static final String SET_STATIC_FOR_APPROACH = "Set static for approach";
   public static final String SET_STATIC_FOR_GRASP = "Set static for grasp";

   public static final String START_SCAN = "START SCAN";
   public static final String END_SCAN = "END SCAN";
   public static final String START_PULL_DOOR= "START PULL DOOR";
   public static final String END_PULL_DOOR = "END PULL DOOR";
   public static final String START_PUSH_DOOR= "START PUSH DOOR";
   public static final String END_PUSH_DOOR = "END PUSH DOOR";
   public static final String START_TRASHCAN = "START TRASHCAN";
   public static final String END_TRASHCAN = "END TRASHCAN";
   public static final String START_COUCH = "START COUCH";
   public static final String END_COUCH = "END COUCH";
   public static final String START_TABLE_LEFT = "START TABLE LEFT";
   public static final String END_TABLE_LEFT = "END TABLE LEFT";
   public static final String START_TABLE_RIGHT = "START TABLE RIGHT";
   public static final String END_TABLE_RIGHT= "END TABLE RIGHT";
   public static final String START_SALUTE = "START SALUTE";

   public static final String END_WALK_DOOR_A = "END walk door A";
   public static final String END_WALK_DOOR_B = "END walk door B";
   public static final String END_TURN_DOOR_A = "END turn door A";
   public static final String END_TURN_DOOR_B = "END turn door B";
   public static final String END_WALK_COUCH = "END walk couch";


   private final BuildingExplorationDefinition definition;

   private BehaviorTreeRootNodeState actionSequence;
   private final List<WaitDurationActionState> setStaticForApproachActions = new ArrayList<>();
   private final List<WaitDurationActionState> setStaticForGraspActions = new ArrayList<>();
   private WaitDurationActionState startScanAction;
   private WaitDurationActionState endScanAction;
   private WaitDurationActionState startPullDoorAction;
   private WaitDurationActionState endPullDoorAction;
   private WaitDurationActionState startPushDoorAction;
   private WaitDurationActionState endPushDoorAction;
   private WaitDurationActionState startCouchAction;
   private WaitDurationActionState endCouchAction;
   private WaitDurationActionState startTrashCanAction;
   private WaitDurationActionState endTrashCanAction;
   private WaitDurationActionState startTableRightAction;
   private WaitDurationActionState endTableRightAction;
   private WaitDurationActionState startTableLeftAction;
   private WaitDurationActionState endTableLeftAction;
   private WaitDurationActionState startSaluteAction;
   private WaitDurationActionState endWalkDoorAAction;
   private WaitDurationActionState endWalkDoorBAction;
   private WaitDurationActionState endTurnDoorAAction;
   private WaitDurationActionState endTurnDoorBAction;
   private WaitDurationActionState endWalkCouchAction;

   public BuildingExplorationState(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory)
   {
      super(id, new BuildingExplorationDefinition(crdtInfo, saveFileDirectory), crdtInfo);

      definition = getDefinition();
   }

   @Override
   public void update()
   {
      super.update();

      actionSequence = BehaviorTreeTools.findRootNode(this);

      updateActionSubtree(this);
   }

   public void updateActionSubtree(BehaviorTreeNodeState<?> node)
   {
      setStaticForApproachActions.clear();
      setStaticForGraspActions.clear();

      for (BehaviorTreeNodeState<?> child : node.getChildren())
      {
         if (child instanceof ActionNodeState<?> actionNode)
         {
            if (actionNode instanceof WaitDurationActionState waitDurationAction
                && waitDurationAction.getDefinition().getName().equals(SET_STATIC_FOR_APPROACH))
            {
               setStaticForApproachActions.add(waitDurationAction);
            }
            if (actionNode instanceof WaitDurationActionState waitDurationAction
                && waitDurationAction.getDefinition().getName().equals(SET_STATIC_FOR_GRASP))
            {
               setStaticForGraspActions.add(waitDurationAction);
            }

            if (actionNode instanceof WaitDurationActionState waitDurationAction
                && waitDurationAction.getDefinition().getName().equals(START_SCAN))
            {
               startScanAction = waitDurationAction;
            }
            if (actionNode instanceof WaitDurationActionState waitDurationAction
                && waitDurationAction.getDefinition().getName().equals(END_SCAN))
            {
               endScanAction = waitDurationAction;
            }
            if (actionNode instanceof WaitDurationActionState waitDurationAction
                && waitDurationAction.getDefinition().getName().equals(START_PULL_DOOR))
            {
               startPullDoorAction = waitDurationAction;
            }
            if (actionNode instanceof WaitDurationActionState waitDurationAction
                && waitDurationAction.getDefinition().getName().equals(END_PULL_DOOR))
            {
               endPullDoorAction = waitDurationAction;
            }
            if (actionNode instanceof WaitDurationActionState waitDurationAction
                && waitDurationAction.getDefinition().getName().equals(START_PUSH_DOOR))
            {
               startPushDoorAction = waitDurationAction;
            }
            if (actionNode instanceof WaitDurationActionState waitDurationAction
                && waitDurationAction.getDefinition().getName().equals(END_PUSH_DOOR))
            {
               endPushDoorAction = waitDurationAction;
            }
            if (actionNode instanceof WaitDurationActionState waitDurationAction
                && waitDurationAction.getDefinition().getName().equals(START_COUCH))
            {
               startCouchAction = waitDurationAction;
            }
            if (actionNode instanceof WaitDurationActionState waitDurationAction
                && waitDurationAction.getDefinition().getName().equals(END_COUCH))
            {
               endCouchAction = waitDurationAction;
            }
            if (actionNode instanceof WaitDurationActionState waitDurationAction
                && waitDurationAction.getDefinition().getName().equals(START_TRASHCAN))
            {
               startTrashCanAction = waitDurationAction;
            }
            if (actionNode instanceof WaitDurationActionState waitDurationAction
                && waitDurationAction.getDefinition().getName().equals(END_TRASHCAN))
            {
               endTrashCanAction = waitDurationAction;
            }
            if (actionNode instanceof WaitDurationActionState waitDurationAction
                && waitDurationAction.getDefinition().getName().equals(START_TABLE_LEFT))
            {
               startTableLeftAction = waitDurationAction;
            }
            if (actionNode instanceof WaitDurationActionState waitDurationAction
                && waitDurationAction.getDefinition().getName().equals(END_TABLE_LEFT))
            {
               endTableLeftAction = waitDurationAction;
            }
            if (actionNode instanceof WaitDurationActionState waitDurationAction
                && waitDurationAction.getDefinition().getName().equals(START_TABLE_RIGHT))
            {
               startTableRightAction = waitDurationAction;
            }
            if (actionNode instanceof WaitDurationActionState waitDurationAction
                && waitDurationAction.getDefinition().getName().equals(END_TABLE_RIGHT))
            {
               endTableRightAction = waitDurationAction;
            }
            if (actionNode instanceof WaitDurationActionState waitDurationAction
                && waitDurationAction.getDefinition().getName().equals(START_SALUTE))
            {
               startSaluteAction = waitDurationAction;
            }

            if (actionNode instanceof WaitDurationActionState waitDurationAction
                && waitDurationAction.getDefinition().getName().equals(END_WALK_DOOR_A))
            {
               endWalkDoorAAction = waitDurationAction;
            }
            if (actionNode instanceof WaitDurationActionState waitDurationAction
                && waitDurationAction.getDefinition().getName().equals(END_WALK_DOOR_B))
            {
               endWalkDoorBAction = waitDurationAction;
            }
            if (actionNode instanceof WaitDurationActionState waitDurationAction
                && waitDurationAction.getDefinition().getName().equals(END_TURN_DOOR_A))
            {
               endTurnDoorAAction = waitDurationAction;
            }
            if (actionNode instanceof WaitDurationActionState waitDurationAction
                && waitDurationAction.getDefinition().getName().equals(END_TURN_DOOR_B))
            {
               endTurnDoorBAction = waitDurationAction;
            }
            if (actionNode instanceof WaitDurationActionState waitDurationAction
                && waitDurationAction.getDefinition().getName().equals(END_WALK_COUCH))
            {
               endWalkCouchAction = waitDurationAction;
            }
         }
         else
         {
            updateActionSubtree(child);
         }
      }
   }

   public void toMessage(BuildingExplorationStateMessage message)
   {
      getDefinition().toMessage(message.getDefinition());

      super.toMessage(message.getState());
   }

   public void fromMessage(BuildingExplorationStateMessage message)
   {
      super.fromMessage(message.getState());

      getDefinition().fromMessage(message.getDefinition());
   }

   public BehaviorTreeRootNodeState getActionSequence()
   {
      return actionSequence;
   }

   public List<WaitDurationActionState> getSetStaticForApproachActions()
   {
      return setStaticForApproachActions;
   }

   public List<WaitDurationActionState> getSetStaticForGraspActions()
   {
      return setStaticForGraspActions;
   }

   public WaitDurationActionState getStartScanAction()
   {
      return startScanAction;
   }

   public WaitDurationActionState getEndScanAction()
   {
      return endScanAction;
   }

   public WaitDurationActionState getStartPullDoorAction()
   {
      return startPullDoorAction;
   }

   public WaitDurationActionState getEndPullDoorAction()
   {
      return endPullDoorAction;
   }

   public WaitDurationActionState getStartPushDoorAction()
   {
      return startPushDoorAction;
   }

   public WaitDurationActionState getEndPushDoorAction()
   {
      return endPushDoorAction;
   }

   public WaitDurationActionState getStartCouchAction()
   {
      return startCouchAction;
   }

   public WaitDurationActionState getEndCouchAction()
   {
      return endCouchAction;
   }

   public WaitDurationActionState getStartTrashCanAction()
   {
      return startTrashCanAction;
   }

   public WaitDurationActionState getEndTrashCanAction()
   {
      return endTrashCanAction;
   }

   public WaitDurationActionState getStartTableRightAction()
   {
      return startTableRightAction;
   }

   public WaitDurationActionState getEndTableRightAction()
   {
      return endTableRightAction;
   }

   public WaitDurationActionState getStartTableLeftAction()
   {
      return startTableLeftAction;
   }

   public WaitDurationActionState getEndTableLeftAction()
   {
      return endTableLeftAction;
   }

   public WaitDurationActionState getStartSaluteAction()
   {
      return startSaluteAction;
   }

   public WaitDurationActionState getEndWalkDoorAAction()
   {
      return endWalkDoorAAction;
   }

   public WaitDurationActionState getEndWalkDoorBAction()
   {
      return endWalkDoorBAction;
   }

   public WaitDurationActionState getEndTurnDoorAAction()
   {
      return endTurnDoorAAction;
   }

   public WaitDurationActionState getEndTurnDoorBAction()
   {
      return endTurnDoorBAction;
   }

   public WaitDurationActionState getEndWalkCouchAction()
   {
      return endWalkCouchAction;
   }
}
