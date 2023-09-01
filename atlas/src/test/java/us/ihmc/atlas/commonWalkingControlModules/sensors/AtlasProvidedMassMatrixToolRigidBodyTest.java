package us.ihmc.atlas.commonWalkingControlModules.sensors;

import java.util.Random;

import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.sensors.ProvidedMassMatrixToolRigidBodyTest;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.tools.JointStateType;
import us.ihmc.mecano.tools.MultiBodySystemRandomTools;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;

public class AtlasProvidedMassMatrixToolRigidBodyTest extends ProvidedMassMatrixToolRigidBodyTest
{
   AtlasRobotVersion version = AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ;
   RobotSide side = RobotSide.LEFT;
   AtlasRobotModel atlasRobotModel = new AtlasRobotModel(version, RobotTarget.SCS, false);

   @Override
   public FullHumanoidRobotModel getFullRobotModel()
   {
      FullHumanoidRobotModel fullRobotModel = atlasRobotModel.createFullRobotModel(false);

      Random random = new Random(945298L);
      OneDoFJointBasics[] armJoints = MultiBodySystemTools.createOneDoFJointPath(fullRobotModel.getChest(), fullRobotModel.getHand(side));
      MultiBodySystemRandomTools.nextState(random, JointStateType.CONFIGURATION, armJoints);
      fullRobotModel.updateFrames();

      return fullRobotModel;
   }

   @Override
   @Test
   public void testprovidedMassMatrixToolRigidBody()
   {
      super.testprovidedMassMatrixToolRigidBody();
   }
}
