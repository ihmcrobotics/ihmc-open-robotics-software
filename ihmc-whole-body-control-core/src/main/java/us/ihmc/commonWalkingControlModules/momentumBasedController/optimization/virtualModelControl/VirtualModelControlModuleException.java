package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.virtualModelControl;

import us.ihmc.commonWalkingControlModules.virtualModelControl.VirtualModelControlSolution;
import us.ihmc.convexOptimization.exceptions.NoConvergenceException;

public class VirtualModelControlModuleException extends NoConvergenceException
{
   private static final long serialVersionUID = -6812479891899477883L;

   private final VirtualModelControlSolution virtualModelControlSolution;

   public VirtualModelControlModuleException(VirtualModelControlSolution virtualModelControlSolution)
   {
      this.virtualModelControlSolution = virtualModelControlSolution;
   }

   public VirtualModelControlSolution getVirtualModelControlSolution()
   {
      return virtualModelControlSolution;
   }

}
