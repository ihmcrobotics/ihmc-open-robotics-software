package us.ihmc.quadrupedRobotics.messageHandling;

import us.ihmc.quadrupedRobotics.controlModules.QuadrupedBalanceManager;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedBodyOrientationManager;
import us.ihmc.quadrupedRobotics.controlModules.QuadrupedControlManagerFactory;
import us.ihmc.quadrupedRobotics.controlModules.foot.QuadrupedFeetManager;

public class QuadrupedStepCommandConsumer
{
   private final QuadrupedBalanceManager balanceManager;
   private final QuadrupedFeetManager feetManager;
   private final QuadrupedBodyOrientationManager bodyOrientationManager;



   public QuadrupedStepCommandConsumer(QuadrupedControlManagerFactory managerFactory)
   {
      balanceManager = managerFactory.getOrCreateBalanceManager();
      feetManager = managerFactory.getOrCreateFeetManager();
      bodyOrientationManager = managerFactory.getOrCreateBodyOrientationManager();
   }

   public void update()
   {

   }
}
