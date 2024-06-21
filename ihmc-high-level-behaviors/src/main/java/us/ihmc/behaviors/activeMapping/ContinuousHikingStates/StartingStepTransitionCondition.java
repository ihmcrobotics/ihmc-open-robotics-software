package us.ihmc.behaviors.activeMapping.ContinuousHikingStates;

import controller_msgs.msg.dds.FootstepDataListMessage;
import us.ihmc.behaviors.activeMapping.ContinuousHikingParameters;
import us.ihmc.behaviors.activeMapping.ContinuousHikingState;
import us.ihmc.behaviors.activeMapping.ContinuousPlanner;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;

import static us.ihmc.behaviors.activeMapping.ContinuousPlannerSchedulingTask.statistics;

public class StartingStepTransitionCondition implements StateTransitionCondition
{
   private final ContinuousPlanner continuousPlanner;
   private final StepValidityChecker stepValidityChecker;
   private final ContinuousHikingParameters continuousHikingParameters;

   public StartingStepTransitionCondition(ContinuousPlanner continuousPlanner, StepValidityChecker stepValidityChecker,
                                          ContinuousHikingParameters continuousHikingParameters)
   {
      this.continuousPlanner = continuousPlanner;
      this.stepValidityChecker = stepValidityChecker;
      this.continuousHikingParameters = continuousHikingParameters;
   }

   @Override
   public boolean testCondition(double timeInCurrentState)
   {
      if (!continuousPlanner.isPlanAvailable())
         return false;
      if (!continuousHikingParameters.getStepPublisherEnabled())
         return false;

      return stepValidityChecker.isNextStepValid();
   }
}
