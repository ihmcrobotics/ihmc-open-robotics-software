package us.ihmc.valkyrie.pushRecovery;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInfo;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.pushRecovery.HumanoidFootFallDisturbanceRecoveryTest;
import us.ihmc.commonWalkingControlModules.configurations.SwingTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;
import us.ihmc.valkyrie.parameters.ValkyrieSwingTrajectoryParameters;
import us.ihmc.valkyrie.parameters.ValkyrieWalkingControllerParameters;

public class ValkyrieFootFallDisturbanceRecoveryTest extends HumanoidFootFallDisturbanceRecoveryTest
{
   @Override
   public DRCRobotModel getRobotModel()
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, ValkyrieRobotVersion.FINGERLESS)
      {
         @Override
         public WalkingControllerParameters getWalkingControllerParameters()
         {
            return new ValkyrieWalkingControllerParameters(getJointMap(), getRobotPhysicalProperties(), getTarget())
            {
               @Override
               public SwingTrajectoryParameters getSwingTrajectoryParameters()
               {
                  return new ValkyrieSwingTrajectoryParameters(getRobotPhysicalProperties(), getTarget())
                  {
                     @Override
                     public Tuple3DReadOnly getTouchdownVelocityWeight()
                     {
                        return new Vector3D(30.0, 30.0, 30.0);
                     }
                  };
               }
            };
         }
      };
      return robotModel;
   }

   @Test
   public void testBlindWalkOverHole(TestInfo testInfo) throws Exception
   {
      super.testBlindWalkOverHole(testInfo, 0.6, 0.25);
   }
}
