package us.ihmc.quadrupedRobotics.controlModules.foot;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.quadrupedRobotics.controller.force.QuadrupedForceControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedSolePositionController;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedStepTransitionCallback;
import us.ihmc.quadrupedRobotics.controller.force.toolbox.QuadrupedTaskSpaceEstimates;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedStep;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.stateMachines.eventBasedStateMachine.FiniteStateMachineStateChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class QuadrupedFeetManager
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final QuadrantDependentList<QuadrupedFootControlModule> footControlModules = new QuadrantDependentList<>();

   public QuadrupedFeetManager(QuadrupedForceControllerToolbox toolbox, QuadrantDependentList<QuadrupedSolePositionController> solePositionControllers,
                               YoVariableRegistry parentRegistry)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         footControlModules.set(robotQuadrant, new QuadrupedFootControlModule(robotQuadrant, toolbox, solePositionControllers.get(robotQuadrant), registry));
      }
      parentRegistry.addChild(registry);
   }

   public void attachStateChangedListener(FiniteStateMachineStateChangedListener stateChangedListener)
   {
      for (RobotQuadrant quadrant : RobotQuadrant.values)
         footControlModules.get(quadrant).attachStateChangedListener(stateChangedListener);
   }

   public void triggerStep(RobotQuadrant robotQuadrant, QuadrupedTimedStep stepSequence)
   {
      footControlModules.get(robotQuadrant).triggerStep(stepSequence);
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
}
