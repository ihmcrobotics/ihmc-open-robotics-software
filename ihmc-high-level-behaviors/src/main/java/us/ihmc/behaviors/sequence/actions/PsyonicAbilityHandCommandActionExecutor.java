package us.ihmc.behaviors.sequence.actions;

import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sequence.ActionNodeExecutor;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class PsyonicAbilityHandCommandActionExecutor extends ActionNodeExecutor<PsyonicAbilityHandCommandActionState, PsyonicAbilityHandCommandActionDefinition>
{
   private final PsyonicAbilityHandCommandActionState state;
   private final PsyonicAbilityHandCommandActionDefinition definition;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final ROS2SyncedRobotModel syncedRobot;

   public PsyonicAbilityHandCommandActionExecutor(long id,
                                        CRDTInfo crdtInfo,
                                        WorkspaceResourceDirectory saveFileDirectory,
                                        ROS2ControllerHelper ros2ControllerHelper,
                                        ROS2SyncedRobotModel syncedRobot)
   {
      super(new PsyonicAbilityHandCommandActionState(id, crdtInfo, saveFileDirectory));

      state = getState();
      definition = getDefinition();

      this.ros2ControllerHelper = ros2ControllerHelper;
      this.syncedRobot = syncedRobot;
   }

   @Override
   public void update()
   {
      super.update();

      state.setCanExecute(true);
   }

   @Override
   public void triggerActionExecution()
   {
      super.triggerActionExecution();
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      state.setIsExecuting(false);
   }
}
