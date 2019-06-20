package us.ihmc.commonWalkingControlModules.barrierScheduler.context;

import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullRobotModel;

public class HumanoidRobotContextTools
{
   public static void updateContext(FullRobotModel fullRobotModel, HumanoidRobotContextJointData contextData)
   {
      contextData.clear();
      for (int jointIndex = 0; jointIndex < fullRobotModel.getOneDoFJoints().length; jointIndex++)
      {
         OneDoFJointBasics joint = fullRobotModel.getOneDoFJoints()[jointIndex];
         contextData.addJoint(joint.getQ(), joint.getQd(), joint.getQdd(), joint.getTau());
      }
      FloatingJointBasics rootJoint = fullRobotModel.getRootJoint();
      contextData.setRootJointData(rootJoint.getJointPose(), rootJoint.getJointTwist(), rootJoint.getJointAcceleration());
   }

   public static void updateRobot(FullRobotModel fullRobotModel, HumanoidRobotContextJointData contextData)
   {
      for (int jointIndex = 0; jointIndex < fullRobotModel.getOneDoFJoints().length; jointIndex++)
      {
         OneDoFJointBasics joint = fullRobotModel.getOneDoFJoints()[jointIndex];
         joint.setQ(contextData.getJointQForIndex(jointIndex));
         joint.setQd(contextData.getJointQdForIndex(jointIndex));
         joint.setQdd(contextData.getJointQddForIndex(jointIndex));
         joint.setTau(contextData.getJointTauForIndex(jointIndex));
      }
      FloatingJointBasics rootJoint = fullRobotModel.getRootJoint();
      HumanoidRobotContextRootJointData rootJointData = contextData.getRootJointData();
      rootJoint.setJointOrientation(rootJointData.getRootJointOrientation());
      rootJoint.setJointAngularVelocity(rootJointData.getRootJointAngularVelocity());
      rootJoint.setJointAngularAcceleration(rootJointData.getRootJointAngularAcceleration());
      rootJoint.setJointPosition(rootJointData.getRootJointLocation());
      rootJoint.setJointLinearVelocity(rootJointData.getRootJointLinearVelocity());
      rootJoint.setJointLinearAcceleration(rootJointData.getRootJointLinearAcceleration());
      fullRobotModel.updateFrames();
   }
}
