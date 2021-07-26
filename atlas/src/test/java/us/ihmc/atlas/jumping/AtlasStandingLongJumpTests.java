package us.ihmc.atlas.jumping;

import org.junit.jupiter.api.Tag;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.initialSetup.AtlasSimInitialSetup;
import us.ihmc.atlas.parameters.AtlasMomentumOptimizationSettings;
import us.ihmc.atlas.parameters.AtlasWalkingControllerParameters;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.avatar.initialSetup.DRCSCSInitialSetup;
import us.ihmc.avatar.jumping.AvatarStandingLongJumpTests;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;

@Tag("humanoid-flat-ground")
public class AtlasStandingLongJumpTests extends AvatarStandingLongJumpTests
{
   @Override
   public DRCRobotModel getRobotModel()
   {
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS)
      {
         @Override
         public WalkingControllerParameters getWalkingControllerParameters()
         {
            return new AtlasWalkingControllerParameters(RobotTarget.SCS, getJointMap(), getContactPointParameters())
            {
               @Override
               public MomentumOptimizationSettings getMomentumOptimizationSettings()
               {
                  return getTestMomentumOptimizationSettings(getJointMap(), getContactPointParameters().getNumberOfContactableBodies());
               }
            };
         }
      };
   }

   @Override
   public DRCRobotInitialSetup getInitialSetup()
   {
      return new AtlasSimInitialSetup();
   }

   @Override
   public String getParameterResourceName()
   {
      return "/us/ihmc/atlas/parameters/jumping_controller.xml";
   }

   private static MomentumOptimizationSettings getTestMomentumOptimizationSettings(HumanoidJointNameMap jointMap, int numberOfContactableBodies)
   {
      double jointAccelerationWeight = 1e-10;
      double jointJerkWeight = 0.0;
      double jointTorqueWeight = 0.0;
      double maximumJointAcceleration = 1000.0;

      return new AtlasMomentumOptimizationSettings(jointMap, numberOfContactableBodies)
      {
         @Override
         public double getJointAccelerationWeight()
         {
            return jointAccelerationWeight;
         }

         @Override
         public double getJointJerkWeight()
         {
            return jointJerkWeight;
         }

         @Override
         public double getJointTorqueWeight()
         {
            return jointTorqueWeight;
         }

         @Override
         public double getMaximumJointAcceleration()
         {
            return maximumJointAcceleration;
         }

         @Override
         public boolean areJointVelocityLimitsConsidered()
         {
            return false;
         }
      };
   }
}
