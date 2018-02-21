package us.ihmc.quadrupedRobotics.controlModules.foot;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedSolePositionController;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedStepTransitionCallback;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedTaskSpaceEstimates;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedWaypointCallback;
import us.ihmc.quadrupedRobotics.planning.*;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.stateMachines.eventBasedStateMachine.FiniteStateMachineStateChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.List;

public class QuadrupedFeetManager
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final QuadrantDependentList<QuadrupedFootControlModule> footControlModules = new QuadrantDependentList<>();

   public QuadrupedFeetManager(QuadrupedForceControllerToolbox toolbox, YoVariableRegistry parentRegistry)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         footControlModules.set(robotQuadrant, new QuadrupedFootControlModule(robotQuadrant, toolbox, registry));
      }
      parentRegistry.addChild(registry);
   }

   public QuadrupedSolePositionController getSolePositionController(RobotQuadrant robotQuadrant)
   {
      return footControlModules.get(robotQuadrant).getSolePositionController();
   }

   public void attachStateChangedListener(FiniteStateMachineStateChangedListener stateChangedListener)
   {
      for (RobotQuadrant quadrant : RobotQuadrant.values)
         footControlModules.get(quadrant).attachStateChangedListener(stateChangedListener);
   }

   public void initializeWaypointTrajectory(QuadrantDependentList<QuadrupedSoleWaypointList> quadrupedSoleWaypointLists,
                                            QuadrupedTaskSpaceEstimates taskSpaceEstimates, boolean useInitialSoleForceAsFeedforwardTerm)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         QuadrupedSoleWaypointList waypointList = quadrupedSoleWaypointLists.get(robotQuadrant);
         QuadrupedFootControlModule footControlModule = footControlModules.get(robotQuadrant);
         if (waypointList.size() > 0)
         {
            footControlModule.requestMoveViaWaypoints();
            footControlModule.initializeWaypointTrajectory(waypointList, taskSpaceEstimates, useInitialSoleForceAsFeedforwardTerm);
         }
         else
         {
            footControlModule.requestSupport();
         }
      }
   }

   public void triggerStep(QuadrupedTimedStep step)
   {
      footControlModules.get(step.getRobotQuadrant()).triggerStep(step);
   }

   public void triggerSteps(List<YoQuadrupedTimedStep> steps)
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
      step.getGoalPosition(tempPoint);
      tempPoint.changeFrame(ReferenceFrame.getWorldFrame());
      footControlModules.get(step.getRobotQuadrant()).adjustStep(tempPoint);
   }

   public void adjustStep(RobotQuadrant robotQuadrant, FramePoint3DReadOnly adjustedStep)
   {
      footControlModules.get(robotQuadrant).adjustStep(adjustedStep);
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

   public void compute(QuadrantDependentList<FrameVector3D> soleForcesToPack, QuadrupedTaskSpaceEstimates taskSpaceEstimates)
   {
      for (RobotQuadrant quadrant : RobotQuadrant.values)
         compute(soleForcesToPack.get(quadrant), taskSpaceEstimates, quadrant);
   }

   public void compute(FrameVector3D soleForceToPack, QuadrupedTaskSpaceEstimates taskSpaceEstimates, RobotQuadrant robotQuadrant)
   {
      footControlModules.get(robotQuadrant).compute(soleForceToPack, taskSpaceEstimates);
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

   public void requestHoldAll()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         footControlModules.get(robotQuadrant).requestHold();
   }
}
