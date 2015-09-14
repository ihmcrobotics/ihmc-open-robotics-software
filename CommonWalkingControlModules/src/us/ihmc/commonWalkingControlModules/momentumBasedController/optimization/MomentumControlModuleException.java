package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.MomentumModuleSolution;
import us.ihmc.tools.exceptions.NoConvergenceException;

public class MomentumControlModuleException extends NoConvergenceException
{
   private static final long serialVersionUID = -6812479891899477883L;

   private final MomentumModuleSolution momentumModuleSolution;
   
   public MomentumControlModuleException(NoConvergenceException noConvergenceException, MomentumModuleSolution momentumModuleSolution)
   {
      this.momentumModuleSolution = momentumModuleSolution;
   }

   public MomentumModuleSolution getMomentumModuleSolution()
   {
      return momentumModuleSolution;
   }

}
