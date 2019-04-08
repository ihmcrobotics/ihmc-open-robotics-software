package us.ihmc.atlas.commonWalkingControlModules.sensors;

import java.util.Random;
import java.util.stream.DoubleStream;

import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.sensors.ProvidedMassMatrixToolRigidBodyTest;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.LimbName;
import us.ihmc.robotics.robotSide.RobotSide;

public class AtlasProvidedMassMatrixToolRigidBodyTest extends ProvidedMassMatrixToolRigidBodyTest
{
   AtlasRobotVersion version = AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ;
   RobotSide side = RobotSide.LEFT;
   AtlasRobotModel atlasRobotModel = new AtlasRobotModel(version, RobotTarget.SCS, false);

   @Override
   public FullHumanoidRobotModel getFullRobotModel()
   {
      FullHumanoidRobotModel fullRobotModel = atlasRobotModel.createFullRobotModel();

      int numberOfJoints = fullRobotModel.getRobotSpecificJointNames().getArmJointNames().length;
      Random random = new Random(945298L);
      double[] randomAngles = DoubleStream.generate(() -> random.nextDouble()).limit(numberOfJoints).toArray();

      fullRobotModel.setJointAngles(side, LimbName.ARM, randomAngles);
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
