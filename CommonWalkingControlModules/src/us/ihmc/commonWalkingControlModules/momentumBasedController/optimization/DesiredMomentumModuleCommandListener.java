package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredJointAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredPointAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredRateOfChangeOfMomentumCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.DesiredSpatialAccelerationCommand;
import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.MomentumModuleSolution;

public interface DesiredMomentumModuleCommandListener
{
   public abstract void desiredRateOfChangeOfMomentumWasSet(DesiredRateOfChangeOfMomentumCommand desiredRateOfChangeOfMomentumCommand);

   public abstract void desiredJointAccelerationWasSet(DesiredJointAccelerationCommand desiredJointAccelerationCommand);
   public abstract void desiredSpatialAccelerationWasSet(DesiredSpatialAccelerationCommand desiredSpatialAccelerationCommand);
   public abstract void desiredPointAccelerationWasSet(DesiredPointAccelerationCommand desiredPointAccelerationCommand);
   public abstract void reset();
   
   public abstract void momentumModuleSolutionWasComputed(MomentumModuleSolution momentumModuleSolution);
}
