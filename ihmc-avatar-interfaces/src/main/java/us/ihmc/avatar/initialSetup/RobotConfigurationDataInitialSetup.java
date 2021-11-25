package us.ihmc.avatar.initialSetup;

import controller_msgs.msg.dds.RobotConfigurationData;
import gnu.trove.list.array.TFloatArrayList;
import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;

public class RobotConfigurationDataInitialSetup implements RobotInitialSetup<HumanoidFloatingRootJointRobot>
{
   private final RobotConfigurationData robotConfigurationData;
   private final OneDoFJointBasics[] allJointsExcludingHands;

   public RobotConfigurationDataInitialSetup(RobotConfigurationData robotConfigurationData, FullHumanoidRobotModel fullHumanoidRobotModel)
   {
      this.robotConfigurationData = robotConfigurationData;
      allJointsExcludingHands = FullRobotModelUtils.getAllJointsExcludingHands(fullHumanoidRobotModel);
   }

   @Override
   public void initializeRobot(HumanoidFloatingRootJointRobot robot)
   {
      robot.getRootJoint().setPosition(robotConfigurationData.getRootTranslation());
      robot.getRootJoint().setOrientation(robotConfigurationData.getRootOrientation());

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
   public void initializeFullRobotModel(FullHumanoidRobotModel fullRobotModel)
   {
      Pose3DBasics rootJointPose = fullRobotModel.getRootJoint().getJointPose();
      rootJointPose.getPosition().set(robotConfigurationData.getRootTranslation());
      rootJointPose.getOrientation().set(robotConfigurationData.getRootOrientation());

      TFloatArrayList jointAngles = robotConfigurationData.getJointAngles();
      TFloatArrayList jointVelocities = robotConfigurationData.getJointVelocities();

      for (int i = 0; i < allJointsExcludingHands.length; i++)
      {
         String jointName = allJointsExcludingHands[i].getName();
         OneDoFJointBasics joint = fullRobotModel.getOneDoFJointByName(jointName);
         joint.setQ(jointAngles.get(i));
         joint.setQd(jointVelocities.get(i));
      }
   }

   @Override
   public void initializeRobot(RigidBodyBasics rootBody)
   {
      Pose3DBasics rootJointPose = ((FloatingJointBasics) rootBody.getChildrenJoints().get(0)).getJointPose();
      rootJointPose.getPosition().set(robotConfigurationData.getRootTranslation());
      rootJointPose.getOrientation().set(robotConfigurationData.getRootOrientation());

      TFloatArrayList jointAngles = robotConfigurationData.getJointAngles();
      TFloatArrayList jointVelocities = robotConfigurationData.getJointVelocities();

      for (int i = 0; i < allJointsExcludingHands.length; i++)
      {
         String jointName = allJointsExcludingHands[i].getName();
         OneDoFJointBasics joint = (OneDoFJointBasics) MultiBodySystemTools.findJoint(rootBody, jointName);
         joint.setQ(jointAngles.get(i));
         joint.setQd(jointVelocities.get(i));
      }
   }

   @Override
   public void initializeRobotDefinition(RobotDefinition robotDefinition)
   {
      SixDoFJointState initialRootJointState = new SixDoFJointState(robotConfigurationData.getRootOrientation(), robotConfigurationData.getRootTranslation());
      robotDefinition.getRootJointDefinitions().get(0).setInitialJointState(initialRootJointState);

      TFloatArrayList jointAngles = robotConfigurationData.getJointAngles();
      TFloatArrayList jointVelocities = robotConfigurationData.getJointVelocities();

      for (int i = 0; i < allJointsExcludingHands.length; i++)
      {
         String jointName = allJointsExcludingHands[i].getName();
         robotDefinition.getOneDoFJointDefinition(jointName).setInitialJointState(jointAngles.get(i), jointVelocities.get(i));
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
   public void setOffset(Tuple3DReadOnly additionalOffset)
   {
   }

   @Override
   public Vector3D getOffset()
   {
      return robotConfigurationData.getRootTranslation();
   }
}
