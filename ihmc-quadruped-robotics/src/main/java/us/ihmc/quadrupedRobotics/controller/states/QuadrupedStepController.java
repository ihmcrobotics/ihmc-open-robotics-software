package us.ihmc.quadrupedRobotics.controller.states;

import us.ihmc.quadrupedRobotics.controlModules.QuadrupedBalanceManager;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedBodyOrientationManager;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedControlManagerFactory;
import us.ihmc.quadrupedRobotics.controlModules.foot.QuadrupedFeetManager;
import us.ihmc.quadrupedRobotics.controller.ControllerEvent;
import us.ihmc.quadrupedRobotics.controller.QuadrupedController;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerToolbox;
import us.ihmc.quadrupedRobotics.messageHandling.QuadrupedStepMessageHandler;
import us.ihmc.quadrupedRobotics.planning.QuadrupedStep;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class QuadrupedStepController implements QuadrupedController
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final QuadrupedStepMessageHandler stepMessageHandler;

   // managers
   private final QuadrupedFeetManager feetManager;
   private final QuadrupedBalanceManager balanceManager;
   private final QuadrupedBodyOrientationManager bodyOrientationManager;

   private final QuadrupedControllerToolbox controllerToolbox;

   public QuadrupedStepController(QuadrupedControllerToolbox controllerToolbox, QuadrupedControlManagerFactory controlManagerFactory,
                                  QuadrupedStepMessageHandler stepMessageHandler, YoVariableRegistry parentRegistry)
   {
      this.controllerToolbox = controllerToolbox;
      this.stepMessageHandler = stepMessageHandler;

      // feedback controllers
      feetManager = controlManagerFactory.getOrCreateFeetManager();
      balanceManager = controlManagerFactory.getOrCreateBalanceManager();
      bodyOrientationManager = controlManagerFactory.getOrCreateBodyOrientationManager();

      parentRegistry.addChild(registry);
   }

   @Override
   public void onEntry()
   {
      // initialize state
      stepMessageHandler.initialize();

      // update task space estimates
      controllerToolbox.update();

      bodyOrientationManager.setDesiredFrameToHoldPosition(controllerToolbox.getReferenceFrames().getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds());
      bodyOrientationManager.initialize();

      feetManager.reset();
      feetManager.requestFullContact();

      stepMessageHandler.process();
      balanceManager.clearStepSequence();
      balanceManager.addStepsToSequence(stepMessageHandler.getStepSequence());

      balanceManager.initializeForStepping();
   }

   @Override
   public void doAction(double timeInState)
   {
      stepMessageHandler.process();

      // trigger step events
      feetManager.triggerSteps(stepMessageHandler.getActiveSteps());

      // update desired contact state and sole forces
      feetManager.compute();

      balanceManager.clearStepSequence();
      balanceManager.addStepsToSequence(stepMessageHandler.getStepSequence());

      // update desired horizontal com forces
      balanceManager.compute();

      // update step adjustment
      RecyclingArrayList<QuadrupedStep> adjustedSteps = balanceManager.computeStepAdjustment(stepMessageHandler.getActiveSteps());
      feetManager.adjustSteps(adjustedSteps);

      // update desired body orientation, angular velocity, and torque
      bodyOrientationManager.compute();
   }

   @Override
   public ControllerEvent fireEvent(double timeInState)
   {
      if (stepMessageHandler.isDoneWithStepSequence())
      {
         return ControllerEvent.DONE;
      }
      return null;
   }

   @Override
   public void onExit()
   {
      stepMessageHandler.reset();
   }

   public void halt()
   {
      stepMessageHandler.halt();
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }
}
