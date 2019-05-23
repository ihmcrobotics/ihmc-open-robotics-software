package us.ihmc.quadrupedRobotics.controlModules.foot;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommand;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.SoleTrajectoryCommand;
import us.ihmc.quadrupedBasics.gait.QuadrupedStep;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.toolbox.QuadrupedStepTransitionCallback;
import us.ihmc.quadrupedRobotics.controller.toolbox.QuadrupedWaypointCallback;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.quadrupedRobotics.util.YoQuadrupedTimedStep;
import us.ihmc.robotics.math.trajectories.trajectorypoints.lists.FrameEuclideanTrajectoryPointList;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.stateMachine.core.StateChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.List;

public class QuadrupedFeetManager
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final QuadrantDependentList<QuadrupedFootControlModule> footControlModules = new QuadrantDependentList<>();
   private final QuadrupedControllerToolbox toolbox;

   public QuadrupedFeetManager(QuadrupedControllerToolbox toolbox, YoGraphicsListRegistry graphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         footControlModules.set(robotQuadrant, new QuadrupedFootControlModule(robotQuadrant, toolbox, graphicsListRegistry, registry));
      }

      this.toolbox = toolbox;
      parentRegistry.addChild(registry);
   }

   public void setControllerCoreMode(WholeBodyControllerCoreMode controllerCoreMode)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         footControlModules.get(robotQuadrant).setControllerCoreMode(controllerCoreMode);
   }

   public void attachStateChangedListener(StateChangedListener<QuadrupedFootStates> stateChangedListener)
   {
      for (RobotQuadrant quadrant : RobotQuadrant.values)
         footControlModules.get(quadrant).attachStateChangedListener(stateChangedListener);
   }

   public void initializeWaypointTrajectory(SoleTrajectoryCommand soleTrajectoryCommand)
   {
      initializeWaypointTrajectory(soleTrajectoryCommand.getRobotQuadrant(), soleTrajectoryCommand.getPositionTrajectory().getTrajectoryPointList());
   }

   public void initializeWaypointTrajectory(RobotQuadrant robotQuadrant, FrameEuclideanTrajectoryPointList trajectoryPointList)
   {
      QuadrupedFootControlModule footControlModule = footControlModules.get(robotQuadrant);
      if (trajectoryPointList.getNumberOfTrajectoryPoints() > 0)
      {
         footControlModule.requestMoveViaWaypoints();
         footControlModule.initializeWaypointTrajectory(trajectoryPointList);
      }
      else
      {
         footControlModule.requestSupport();
      }
   }

   public void triggerStep(QuadrupedTimedStep step)
   {
      footControlModules.get(step.getRobotQuadrant()).triggerStep(step);
   }

   public void triggerSteps(List<? extends QuadrupedTimedStep> steps)
   {
      for (int i = 0; i < steps.size(); i++)
      {
         QuadrupedTimedStep step = steps.get(i);
         footControlModules.get(step.getRobotQuadrant()).triggerStep(step);
      }
   }

   public void adjustSteps(List<QuadrupedStep> activeSteps)
   {
      for (int i = 0; i < activeSteps.size(); i++)
         adjustStep(activeSteps.get(i));
   }

   private final FramePoint3D tempPoint = new FramePoint3D();

   public void adjustStep(QuadrupedStep step)
   {
      tempPoint.setIncludingFrame(step.getReferenceFrame(), step.getGoalPosition());
      tempPoint.changeFrame(ReferenceFrame.getWorldFrame());
      footControlModules.get(step.getRobotQuadrant()).adjustStep(tempPoint);
   }

   /**
    * Request the swing trajectory to speed up using the given speed up factor.
    * It is clamped w.r.t. to {@link QuadrupedSwingState#minSwingTimeForDisturbanceRecovery}.
    * @param speedUpFactor multiplier on the current time
    * @return the current swing time remaining for the swing foot trajectory
    */
   public double requestSwingSpeedUp(RobotQuadrant robotQuadrant, double speedUpTime)
   {
      return footControlModules.get(robotQuadrant).requestSwingSpeedUp(speedUpTime);
   }

   public double computeClampedSwingSpeedUpTime(RobotQuadrant robotQuadrant, double speedUpTime)
   {
      return footControlModules.get(robotQuadrant).computeClampedSwingSpeedUpTime(speedUpTime);
   }

   public void reset()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         footControlModules.get(robotQuadrant).reset();
   }

   public void registerStepTransitionCallback(QuadrupedStepTransitionCallback stepTransitionCallback)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         footControlModules.get(robotQuadrant).registerStepTransitionCallback(stepTransitionCallback);
   }

   public void registerWaypointCallback(QuadrupedWaypointCallback waypointCallback)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         footControlModules.get(robotQuadrant).registerWaypointCallback(waypointCallback);
   }

   public void compute()
   {
      for (RobotQuadrant quadrant : RobotQuadrant.values)
         compute(quadrant);
   }

   public void compute(RobotQuadrant robotQuadrant)
   {
      footControlModules.get(robotQuadrant).compute();
   }

   public ContactState getContactState(RobotQuadrant robotQuadrant)
   {
      return footControlModules.get(robotQuadrant).getContactState();
   }

   public void requestFullContact()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         footControlModules.get(robotQuadrant).requestSupport();
   }

   public FeedbackControlCommandList createFeedbackControlTemplate()
   {
      FeedbackControlCommandList ret = new FeedbackControlCommandList();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         FeedbackControlCommandList template = footControlModules.get(robotQuadrant).createFeedbackControlTemplate();
         ret.addCommandList(template);
      }

      return ret;
   }

   public FeedbackControlCommand<?> getFeedbackControlCommand(RobotQuadrant robotQuadrant)
   {
      return footControlModules.get(robotQuadrant).getFeedbackControlCommand();
   }

   public VirtualModelControlCommand<?> getVirtualModelControlCommand(RobotQuadrant robotQuadrant)
   {
      return footControlModules.get(robotQuadrant).getVirtualModelControlCommand();
   }

   public InverseDynamicsCommand<?> getInverseDynamicsCommand(RobotQuadrant robotQuadrant)
   {
      return footControlModules.get(robotQuadrant).getInverseDynamicsCommand();
   }
}
