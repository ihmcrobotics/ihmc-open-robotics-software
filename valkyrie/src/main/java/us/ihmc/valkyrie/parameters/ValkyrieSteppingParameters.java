package us.ihmc.valkyrie.parameters;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;

public class ValkyrieSteppingParameters implements SteppingParameters
{
   private final RobotTarget target;

   public ValkyrieSteppingParameters(RobotTarget target)
   {
      this.target = target;
   }

   @Override
   public double getFootForwardOffset()
   {
      return ValkyriePhysicalProperties.footForward;
   }

   @Override
   public double getFootBackwardOffset()
   {
      return ValkyriePhysicalProperties.footBack;
   }

   @Override
   public double getInPlaceWidth()
   {
      if (target == RobotTarget.SCS)
         return 0.25;

      return 0.3;
   }

   @Override
   public double getTurningStepWidth()
   {
      return 0.35;
   }

   @Override
   public double getDesiredStepForward()
   {
      return 0.5; // 0.35;
   }

   @Override
   public double getMaxStepLength()
   {
      if (target == RobotTarget.SCS)
         return 0.6; // 0.5; //0.35;

      return 0.4;
   }

   @Override
   public double getDefaultStepLength()
   {
      return 0.5;
   }

   @Override
   public double getMinStepWidth()
   {
      return (target == RobotTarget.REAL_ROBOT) ? 0.165 : 0.15;
   }

   @Override
   public double getMaxStepWidth()
   {
      return 0.4; // 0.4;
   }

   @Override
   public double getStepPitch()
   {
      return 0.0;
   }

   @Override
   public double getMaxStepUp()
   {
      return 0.3;
   }

   @Override
   public double getMaxStepDown()
   {
      return 0.25;
   }

   @Override
   public double getMaxSwingHeightFromStanceFoot()
   {
      return 0.3;
   }

   @Override public double getDefaultSwingHeightFromStanceFoot()
   {
      if (target == RobotTarget.REAL_ROBOT)
         return 0.05; //changed from 0.025 to 0.05 in 11/2018 to help with foot getting caught on pallet
      else
         return 0.1;
   }

   @Override
   public double getMinSwingHeightFromStanceFoot()
   {
      return 0.025;
   }

   @Override
   public double getMaxAngleTurnOutwards()
   {
      return Math.PI / 3.0;
   }

   @Override
   public double getMaxAngleTurnInwards()
   {
      return -Math.toRadians(30.0);
   }

   @Override
   public double getMinAreaPercentForValidFootstep()
   {
      return 0.5;
   }

   @Override
   public double getDangerAreaPercentForValidFootstep()
   {
      return 0.75;
   }

   @Override
   public double getFootWidth()
   {
      return ValkyriePhysicalProperties.footWidth;
   }

   @Override
   public double getToeWidth()
   {
      return ValkyriePhysicalProperties.footWidth;
   }

   @Override
   public double getFootLength()
   {
      return ValkyriePhysicalProperties.footLength;
   }

   @Override
   public double getActualFootWidth()
   {
      return ValkyriePhysicalProperties.footWidth;
   }

   @Override
   public double getActualFootLength()
   {
      return ValkyriePhysicalProperties.footLength;
   }
}
