package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.external;

import us.ihmc.humanoidRobotics.communication.controllerAPI.command.CrocoddylSolverTrajectoryCommand;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class WholeBodyConfigurationManager
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoDouble timeAtWholeBodyCommand = new YoDouble("timeAtWholeBodyCommand", registry);

   private final DoubleProvider time;

   public WholeBodyConfigurationManager(DoubleProvider time,
                                        YoRegistry parentRegistry)
   {
      this.time = time;
      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      timeAtWholeBodyCommand.set(Double.NaN);
   }

   public void doControl()
   {}

   public void handleCrocoddylSolverTrajectoryCommand(CrocoddylSolverTrajectoryCommand command)
   {
      timeAtWholeBodyCommand.set(time.getValue());
   }
}
