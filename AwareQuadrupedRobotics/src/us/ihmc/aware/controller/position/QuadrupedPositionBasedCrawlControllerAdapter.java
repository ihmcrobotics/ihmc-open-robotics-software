package us.ihmc.aware.controller.position;

import us.ihmc.aware.communication.QuadrupedControllerInputProvider;
import us.ihmc.aware.parameters.QuadrupedRuntimeEnvironment;
import us.ihmc.quadrupedRobotics.controller.QuadrupedPositionBasedCrawlController;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedRobotParameters;

public class QuadrupedPositionBasedCrawlControllerAdapter implements QuadrupedPositionController
{
   private final QuadrupedPositionBasedCrawlController controller;

   public QuadrupedPositionBasedCrawlControllerAdapter(QuadrupedRuntimeEnvironment environment, QuadrupedRobotParameters parameters,
         QuadrupedControllerInputProvider inputProvider)
   {
      // TODO: null foot switches
      this.controller = new QuadrupedPositionBasedCrawlController(environment.getControlDT(), parameters, environment.getFullRobotModel(), inputProvider,
            environment.getFootSwitches(), environment.getLegIkCalculator(), environment.getGlobalDataProducer(), environment.getRobotTimestamp(),
            environment.getParentRegistry(), environment.getGraphicsListRegistry(), environment.getGraphicsListRegistryForDetachedOverhead());
   }

   @Override
   public void onEntry()
   {
      controller.doTransitionIntoAction();
   }

   @Override
   public QuadrupedPositionControllerEvent process()
   {
      controller.doAction();

      // TODO: How do we fire events from the adapted controller?
      return null;
   }

   @Override
   public void onExit()
   {
      controller.doTransitionOutOfAction();
   }
}
