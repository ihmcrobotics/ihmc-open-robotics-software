package us.ihmc.avatar.initialSetup;

import controller_msgs.msg.dds.RobotConfigurationData;
import gnu.trove.list.array.TFloatArrayList;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

public class RobotConfigurationDataInitialSetup implements DRCRobotInitialSetup<HumanoidFloatingRootJointRobot>
{
   private final RobotConfigurationData robotConfigurationData;
   private final OneDoFJointBasics[] allJointsExcludingHands;

   public RobotConfigurationDataInitialSetup(RobotConfigurationData robotConfigurationData, FullHumanoidRobotModel fullHumanoidRobotModel)
   {
      this.robotConfigurationData = robotConfigurationData;
      allJointsExcludingHands = FullRobotModelUtils.getAllJointsExcludingHands(fullHumanoidRobotModel);
   }

   @Override
   public void initializeRobot(HumanoidFloatingRootJointRobot robot, DRCRobotJointMap jointMap)
   {
      robot.getRootJoint().setPosition(robotConfigurationData.getRootTranslation());
      robot.getRootJoint().setQuaternion(robotConfigurationData.getRootOrientation());

      TFloatArrayList jointAngles = robotConfigurationData.getJointAngles();
      TFloatArrayList jointVelocities = robotConfigurationData.getJointVelocities();

      for (int i = 0; i < allJointsExcludingHands.length; i++)
      {
         String jointName = allJointsExcludingHands[i].getName();
         OneDegreeOfFreedomJoint joint = robot.getOneDegreeOfFreedomJoint(jointName);
         joint.setQ(jointAngles.get(i));
         joint.setQd(jointVelocities.get(i));
      }
   }

   @Override
   public void setInitialYaw(double yaw)
   {
   }

   @Override
   public double getInitialYaw()
   {
      return robotConfigurationData.getRootOrientation().getYaw();
   }

   @Override
   public void setInitialGroundHeight(double groundHeight)
   {
   }

   @Override
   public double getInitialGroundHeight()
   {
      return robotConfigurationData.getRootTranslation().getZ();
   }

   @Override
   public void setOffset(Vector3D additionalOffset)
   {
   }

   @Override
   public void getOffset(Vector3D offsetToPack)
   {
      offsetToPack.set(robotConfigurationData.getRootTranslation());
   }
}
