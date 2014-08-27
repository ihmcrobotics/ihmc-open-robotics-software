package us.ihmc.darpaRoboticsChallenge.scriptEngine;

import java.util.EnumMap;

import javax.media.j3d.Transform3D;

import us.ihmc.communication.packets.manipulation.HandPosePacket;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.humanoidRobot.partNames.ArmJointName;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.OneDoFJoint;

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

   public FramePose getHandPoseInChestFrame(HandPosePacket handPosePacket, Transform3D wristToHandTansform)
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
