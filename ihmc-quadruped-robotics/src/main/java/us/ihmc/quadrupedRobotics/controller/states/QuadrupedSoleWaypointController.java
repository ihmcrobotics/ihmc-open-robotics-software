package us.ihmc.quadrupedRobotics.controller.states;

import us.ihmc.quadrupedRobotics.controlModules.QuadrupedControlManagerFactory;
import us.ihmc.quadrupedRobotics.controlModules.foot.QuadrupedFeetManager;
import us.ihmc.quadrupedRobotics.controller.ControllerEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedController;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerToolbox;
import us.ihmc.quadrupedRobotics.controller.toolbox.QuadrupedWaypointCallback;
import us.ihmc.quadrupedRobotics.messageHandling.QuadrupedStepMessageHandler;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class QuadrupedSoleWaypointController implements QuadrupedController, QuadrupedWaypointCallback
{
   // Yo variables
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final QuadrupedStepMessageHandler stepMessageHandler;

   private final QuadrupedFeetManager feetManager;

   private final QuadrantDependentList<YoBoolean> isDoneMoving = new QuadrantDependentList<>();

   public QuadrupedSoleWaypointController(QuadrupedControllerToolbox controllerToolbox, QuadrupedControlManagerFactory controlManagerFactory,
                                          QuadrupedStepMessageHandler stepMessageHandler, YoVariableRegistry parentRegistry)
   {
      this.stepMessageHandler = stepMessageHandler;

      feetManager = controlManagerFactory.getOrCreateFeetManager();

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         isDoneMoving.put(robotQuadrant, new YoBoolean(robotQuadrant.getShortName() + "SoleWaypointDoneMoving", registry));
      }

      parentRegistry.addChild(registry);
   }

   @Override
   public void isDoneMoving(RobotQuadrant robotQuadrant, boolean doneMoving)
   {
      boolean done = doneMoving && !isDoneMoving.get(robotQuadrant).getBooleanValue();
      isDoneMoving.get(robotQuadrant).set(done);
   }

   @Override
   public void onEntry()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (stepMessageHandler.hasFootTrajectoryForSolePositionControl(robotQuadrant))
         {
            feetManager
                  .initializeWaypointTrajectory(stepMessageHandler.pollFootTrajectoryForSolePositionControl(robotQuadrant));
            isDoneMoving.get(robotQuadrant).set(false);
         }
         else
         {
            isDoneMoving.get(robotQuadrant).set(true);
         }
      }

      feetManager.registerWaypointCallback(this);
   }

   @Override
   public void doAction(double timeInState)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (stepMessageHandler.hasFootTrajectoryForSolePositionControl(robotQuadrant))
         {
            feetManager
                  .initializeWaypointTrajectory(stepMessageHandler.pollFootTrajectoryForSolePositionControl(robotQuadrant));
            isDoneMoving.get(robotQuadrant).set(false);
         }
      }
   }

   @Override
   public ControllerEvent fireEvent(double timeInState)
   {
      boolean allAreDone = false;
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         allAreDone |= isDoneMoving.get(robotQuadrant).getBooleanValue();

      return allAreDone ? ControllerEvent.DONE : null;
   }

   @Override
   public void onExit()
   {
      feetManager.registerWaypointCallback(null);
   }
}
