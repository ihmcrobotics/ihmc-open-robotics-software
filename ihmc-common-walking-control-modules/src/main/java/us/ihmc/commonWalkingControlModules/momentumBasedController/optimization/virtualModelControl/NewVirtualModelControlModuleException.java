package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.virtualModelControl;

import us.ihmc.commonWalkingControlModules.virtualModelControl.NewVirtualModelControlSolution;
import us.ihmc.tools.exceptions.NoConvergenceException;

public class NewVirtualModelControlModuleException extends NoConvergenceException
{
   private static final long serialVersionUID = -6812479891899477883L;

   private final NewVirtualModelControlSolution virtualModelControlSolution;

   public NewVirtualModelControlModuleException(NoConvergenceException noConvergenceException, NewVirtualModelControlSolution virtualModelControlSolution)
   {
      this.virtualModelControlSolution = virtualModelControlSolution;
   }

   public NewVirtualModelControlSolution getVirtualModelControlSolution()
   {
      return virtualModelControlSolution;
   }

}
