package us.ihmc.darpaRoboticsChallenge.scriptEngine;

import java.util.EnumMap;

import us.ihmc.communication.packets.manipulation.HandPosePacket;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.humanoidRobotics.model.FullRobotModel;
import us.ihmc.humanoidRobotics.partNames.ArmJointName;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

/**
 * Created by Peter on 12/5/13.
 */
public class HandPoseCalcaulatorFromArmJointAngles
{
   private FullRobotModel fullRobotModel;
   private SideDependentList<EnumMap<ArmJointName, OneDoFJoint>> oneDoFJoints = SideDependentList.createListOfEnumMaps(ArmJointName.class);
   private DRCRobotJointMap jointMap;


   public HandPoseCalcaulatorFromArmJointAngles(DRCRobotModel robotModel)
   {
      jointMap = robotModel.getJointMap();
      fullRobotModel = robotModel.createFullRobotModel();

      for (RobotSide robotSide : RobotSide.values())
      {
    	  for(ArmJointName jointName : jointMap.getArmJointNames())
    	  {
    		  oneDoFJoints.get(robotSide).put(jointName, fullRobotModel.getArmJoint(robotSide, jointName));
    	  }
      }
   }

   public FramePose getHandPoseInChestFrame(HandPosePacket handPosePacket, RigidBodyTransform wristToHandTansform)
   {
      RobotSide robotSide = handPosePacket.getRobotSide();
      int i = -1;
      for(ArmJointName jointName : jointMap.getArmJointNames())
      {
    	  oneDoFJoints.get(robotSide).get(jointName).setQ(handPosePacket.getJointAngles()[++i]);
      }

      ReferenceFrame handPositionControlFrame = fullRobotModel.getHandControlFrame(robotSide);

      FramePose handPose = new FramePose(handPositionControlFrame, wristToHandTansform);
      handPose.changeFrame(fullRobotModel.getChest().getParentJoint().getFrameAfterJoint());

      return handPose;
   }
}
