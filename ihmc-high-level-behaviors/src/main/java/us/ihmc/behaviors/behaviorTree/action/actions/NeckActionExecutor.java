package us.ihmc.behaviors.behaviorTree.action.actions;

import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeExecutor;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeExecutor;
import us.ihmc.commons.Conversions;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.robotics.partNames.NeckJointName;

public class NeckActionExecutor extends ActionNodeExecutor<NeckActionState, NeckActionDefinition>
{
   private double trajectoryStartTime;

   public NeckActionExecutor(long id, BehaviorTreeRootNodeExecutor rootNode)
   {
      super(new NeckActionState(id, rootNode.getState()), rootNode);
   }

   @Override
   public void update()
   {
      super.update();

      state.setCanExecute(true);
   }

   @Override
   public void triggerExecution()
   {
      super.triggerExecution();

      NeckJointName[] neckJointNamesArray = syncedRobot.getRobotModel().getJointMap().getNeckJointNames();
      double[] desiredNeckJointValues = new double[neckJointNamesArray.length];

      for (int i = 0; i < neckJointNamesArray.length; i++)
      {
         switch (neckJointNamesArray[i])
         {
            case DISTAL_NECK_YAW:
               desiredNeckJointValues[i] = definition.getYaw();
               break;
            case DISTAL_NECK_PITCH:
               desiredNeckJointValues[i] = definition.getPitch();
               break;
            case DISTAL_NECK_ROLL:
               desiredNeckJointValues[i] = 0.0;
               break;
            default:
               desiredNeckJointValues[i] = 0.0; // fallback
         }
      }

      ros2ControllerHelper.publishToController(HumanoidMessageTools.createHeadJointspaceTaskspaceTrajectoryMessage(syncedRobot.getReferenceFrames(),
                                                                                                                    neckJointNamesArray,
                                                                                                                    desiredNeckJointValues,
                                                                                                                    definition.getTrajectoryDuration()));

      trajectoryStartTime = Conversions.nanosecondsToSeconds(syncedRobot.getTimestamp());
      state.setNominalExecutionDuration(definition.getTrajectoryDuration());
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      double currentTime = Conversions.nanosecondsToSeconds(syncedRobot.getTimestamp());
      double elapsedTime = currentTime - trajectoryStartTime;

      state.setElapsedExecutionTime(elapsedTime);

      if (elapsedTime >= definition.getTrajectoryDuration() * 2.0)
      {
         state.setIsExecuting(false);
         state.setFailed(true);
         state.getLogger().error("Neck action timed out");
         return;
      }

      boolean timeIsUp = elapsedTime >= definition.getTrajectoryDuration();

      if (timeIsUp)
      {
         state.setIsExecuting(false);
      }
   }
}
