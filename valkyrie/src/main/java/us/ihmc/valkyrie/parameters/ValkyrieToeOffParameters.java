package us.ihmc.valkyrie.parameters;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.configurations.ToeOffParameters;

public class ValkyrieToeOffParameters extends ToeOffParameters
{
   private final RobotTarget target;
   private final ValkyriePhysicalProperties physicalProperties;

   public ValkyrieToeOffParameters(ValkyriePhysicalProperties physicalProperties, RobotTarget target)
   {
      this.target = target;
      this.physicalProperties = physicalProperties;
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
      return ((target == RobotTarget.REAL_ROBOT) ? 0.02 : 0.04) * physicalProperties.getModelSizeScale();
   }

   @Override
   public double getCoPProximityForToeOff()
   {
      return super.getCoPProximityForToeOff() * physicalProperties.getModelSizeScale();
   }

   @Override
   public double getICPProximityForToeOff()
   {
      return super.getICPProximityForToeOff() * physicalProperties.getModelSizeScale();
   }

   @Override
   public double getMinStepHeightForToeOff()
   {
      return super.getMinStepHeightForToeOff() * physicalProperties.getModelSizeScale();
   }
   
   @Override
   public double getMinStepLengthForToeOff()
   {
      return physicalProperties.getFootLength();
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
   public boolean lookAtTwoStepCapturabilityForToeOff()
   {
      switch (target)
      {
         case SCS:
            return true;
         case GAZEBO:
         case REAL_ROBOT:
         default:
            return false;
      }
   }

   @Override
   public double getMaximumToeOffAngle()
   {
      return Math.toRadians(30.0);
   }

}
