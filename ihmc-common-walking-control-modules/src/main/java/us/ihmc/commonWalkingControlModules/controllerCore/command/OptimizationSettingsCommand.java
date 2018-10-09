package us.ihmc.commonWalkingControlModules.controllerCore.command;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.ControllerCoreOptimizationSettings;

/**
 * A command that can be used to configure the optimization settings inside the controller core. By default the settings
 * defined inside the {@link ControllerCoreOptimizationSettings} are used inside the controller core. This command allows
 * modifying some of these settings online using the command API.
 *
 * <p>
 * TODO:<br>
 * Extend this command to allow for configuring more setting through the command API (see ticket BEAST-973).
 * </p>
 *
 * @author Georg Wiedebach
 */
public class OptimizationSettingsCommand implements InverseDynamicsCommand<OptimizationSettingsCommand>
{
   /**
    * The lower bound on the rho values in the optimization.
    * @see ControllerCoreOptimizationSettings#getRhoMin()
    */
   private double rhoMin = 0.0;

   public void setRhoMin(double rhoMin)
   {
      this.rhoMin = rhoMin;
   }

   public double getRhoMin()
   {
      return rhoMin;
   }

   @Override
   public void set(OptimizationSettingsCommand other)
   {
      rhoMin = other.rhoMin;
   }

   @Override
   public ControllerCoreCommandType getCommandType()
   {
      return ControllerCoreCommandType.OPTIMIZATION_SETTINGS;
   }

}
