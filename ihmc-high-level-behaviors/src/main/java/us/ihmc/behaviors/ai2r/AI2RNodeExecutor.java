package us.ihmc.behaviors.ai2r;

import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExecutor;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

/**
 * For interfacing with external foundation models.
 */
public class AI2RNodeExecutor extends BehaviorTreeNodeExecutor<AI2RNodeState, AI2RNodeDefinition>
{
   private final ROS2SyncedRobotModel syncedRobot;
   private final SceneGraph sceneGraph;

   public AI2RNodeExecutor(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory, ROS2SyncedRobotModel syncedRobot, SceneGraph sceneGraph)
   {
      super(new AI2RNodeState(id, crdtInfo, saveFileDirectory));

      this.syncedRobot = syncedRobot;
      this.sceneGraph = sceneGraph;
   }

   @Override
   public void update()
   {
      super.update();

   }
}
