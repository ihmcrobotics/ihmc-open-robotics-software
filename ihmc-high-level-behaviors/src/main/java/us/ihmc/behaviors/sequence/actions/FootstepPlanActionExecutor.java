package us.ihmc.behaviors.sequence.actions;

import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sequence.ActionNodeExecutor;
import us.ihmc.behaviors.tools.walkingController.WalkingFootstepTracker;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.log.LogTools;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class FootstepPlanActionExecutor extends ActionNodeExecutor<FootstepPlanActionState, FootstepPlanActionDefinition>
{
   private final FootstepPlanActionState state;
   private final FootstepPlanActionExecutorBasics footstepPlanExecutorBasics;
   private final ReferenceFrameLibrary referenceFrameLibrary;
   private final FramePose3D solePose = new FramePose3D();
   private final FootstepPlan footstepPlanToExecute = new FootstepPlan();

   public FootstepPlanActionExecutor(long id,
                                     CRDTInfo crdtInfo,
                                     WorkspaceResourceDirectory saveFileDirectory,
                                     ROS2ControllerHelper ros2ControllerHelper,
                                     ROS2SyncedRobotModel syncedRobot,
                                     WalkingFootstepTracker footstepTracker,
                                     ReferenceFrameLibrary referenceFrameLibrary,
                                     SceneGraph sceneGraph,
                                     WalkingControllerParameters walkingControllerParameters)
   {
      super(new FootstepPlanActionState(id, crdtInfo, saveFileDirectory, referenceFrameLibrary));

      state = getState();

      this.referenceFrameLibrary = referenceFrameLibrary;

      footstepPlanExecutorBasics = new FootstepPlanActionExecutorBasics(state.getBasics(),
                                                                        getDefinition().getBasics(),
                                                                        ros2ControllerHelper,
                                                                        syncedRobot,
                                                                        footstepTracker,
                                                                        walkingControllerParameters);
      footstepPlanExecutorBasics.setFootstepPlanToExecute(footstepPlanToExecute);
   }

   @Override
   public void update()
   {
      super.update();

      footstepPlanExecutorBasics.update();

      state.setCanExecute(referenceFrameLibrary.containsFrame(getDefinition().getBasics().getParentFrameName()));
   }

   @Override
   public void triggerActionExecution()
   {
      super.triggerActionExecution();

      if (referenceFrameLibrary.containsFrame(state.getDefinition().getBasics().getParentFrameName()))
      {
         footstepPlanToExecute.clear();
         for (FootstepPlanActionFootstepState footstep : state.getFootsteps())
         {
            solePose.setIncludingFrame(footstep.getSoleFrame().getReferenceFrame().getParent(),
                                       footstep.getDefinition().getSoleToPlanFrameTransform().getValueReadOnly());
            solePose.changeFrame(ReferenceFrame.getWorldFrame());
            footstepPlanToExecute.addFootstep(footstep.getDefinition().getSide(), solePose);
         }

         footstepPlanExecutorBasics.triggerActionExecution();
      }
      else
      {
         LogTools.error("Cannot execute. Frame is not a child of World frame.");
      }
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      footstepPlanExecutorBasics.updateCurrentlyExecuting(this);
   }
}
