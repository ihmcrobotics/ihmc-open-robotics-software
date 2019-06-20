package us.ihmc.atlas.roughTerrainWalking;

import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasPhysicalProperties;
import us.ihmc.atlas.parameters.AtlasSwingTrajectoryParameters;
import us.ihmc.atlas.parameters.AtlasWalkingControllerParameters;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.roughTerrainWalking.AvatarFootstepDataMessageSwingTrajectoryTest;
import us.ihmc.commonWalkingControlModules.configurations.SwingTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasFootstepDataMessageSwingTrajectoryTest extends AvatarFootstepDataMessageSwingTrajectoryTest
{
   @Override
   @Test
   public void testSwingTrajectoryInWorld() throws SimulationExceededMaximumTimeException
   {
      super.testSwingTrajectoryInWorld();
   }

   @Override
   @Test
   public void testSwingTrajectoryTouchdownSpeed() throws SimulationExceededMaximumTimeException
   {
      setPushAndAdjust(false);
      super.testSwingTrajectoryTouchdownSpeed();
   }

   @Override
   @Test
   public void testSwingTrajectoryTouchdownWithAdjustment() throws SimulationExceededMaximumTimeException
   {
      setPushAndAdjust(true);
      super.testSwingTrajectoryTouchdownWithAdjustment();
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      RobotTarget target = RobotTarget.SCS;
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, target, false)
      {
         @Override
         public WalkingControllerParameters getWalkingControllerParameters()
         {
            return new AtlasWalkingControllerParameters(target, getJointMap(), getContactPointParameters())
            {
               @Override
               public boolean allowDisturbanceRecoveryBySpeedingUpSwing()
               {
                  return false;
               }

               @Override
               public SwingTrajectoryParameters getSwingTrajectoryParameters()
               {
                  return new AtlasSwingTrajectoryParameters(target, 1.0)
                  {
                     @Override
                     public double getDesiredTouchdownAcceleration()
                     {
                        return 0.0;
                     }

                     @Override
                     public double getDesiredTouchdownVelocity()
                     {
                        return 0.3;
                     }
                  };
               }
            };
         }
      };
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }

   @Override
   public double getLegLength()
   {
      AtlasPhysicalProperties physicalProperties = new AtlasPhysicalProperties();
      return physicalProperties.getShinLength() + physicalProperties.getThighLength();
   }
}
