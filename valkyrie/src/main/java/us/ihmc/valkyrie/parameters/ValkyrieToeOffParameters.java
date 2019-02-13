package us.ihmc.valkyrie.parameters;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.configurations.ToeOffParameters;

public class ValkyrieToeOffParameters extends ToeOffParameters
{
   private final RobotTarget target;

   public ValkyrieToeOffParameters(RobotTarget target)
   {
      this.target = target;
   }

   @Override
   public boolean doToeOffIfPossible()
   {
      return true;
   }

   @Override
   public boolean doToeOffIfPossibleInSingleSupport()
   {
      return false;
   }

   @Override
   public boolean checkECMPLocationToTriggerToeOff()
   {
      // Used to be: target != RobotTarget.REAL_ROBOT;
      // Trying to see if that's really necessary (Sylvain)
      // It delays the toe-off to some extent which can cause some issues.
      return true;
   }

   @Override
   public double getECMPProximityForToeOff()
   {
      return (target == RobotTarget.REAL_ROBOT) ? 0.02 : 0.04;
   }


   @Override
   public double getMinStepLengthForToeOff()
   {
      return ValkyriePhysicalProperties.footLength;
   }

   /**
    * To enable that feature, doToeOffIfPossible() return true is required.
    */
   @Override
   public boolean doToeOffWhenHittingAnkleLimit()
   {
      return true;
   }

   @Override
   public double getMaximumToeOffAngle()
   {
      return Math.toRadians(30.0);
   }

}
