package us.ihmc.commonWalkingControlModules.modelPredictiveController.commands;

/**
 * Specifies an objective value for each of the contact force vectors contained in the contact plane.
 *
 * Can be used to both specify a direct value, as well as upper and lower bounds for those forces.
 */
public class RhoAccelerationObjectiveCommand extends RhoObjectiveCommand
{
   @Override
   public int getDerivativeOrder()
   {
      return 2;
   }
}
