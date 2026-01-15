package us.ihmc.avatar.drcRobot;

import gnu.trove.map.hash.TObjectDoubleHashMap;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.robotSide.RobotSide;

/**
 * Computes the equivalent taskspace home pose that corresponds to the jointspace home pose
 */
public class TaskspaceHomePoseCalculator
{
   public TaskspaceHomePoseCalculator(DRCRobotModel robotModel, WalkingControllerParameters walkingControllerParameters)
   {
      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      TObjectDoubleHashMap<String> jointHome = walkingControllerParameters.getOrCreateJointHomeConfiguration();

      // Set robot arm joint angles to jointspace home
      for (RobotSide robotSide : RobotSide.values())
      {
         ArmJointName[] armJointNames = robotModel.getJointMap().getArmJointNames(robotSide);
         for (int i = 0; i < armJointNames.length; i++)
         {
            OneDoFJointBasics armJoint = fullRobotModel.getArmJoint(robotSide, armJointNames[i]);
            double armJointHome = jointHome.get(armJoint.getName());
            armJoint.setQ(armJointHome);
         }
      }

      // Update frames, namely hand control frame
      fullRobotModel.updateFrames();

      // Compute and print taskspace home pose
      for (RobotSide robotSide : RobotSide.values())
      {
         FramePose3D taskspaceHomePose = new FramePose3D(fullRobotModel.getHandControlFrame(robotSide));
         taskspaceHomePose.changeFrame(fullRobotModel.getChest().getBodyFixedFrame());
         LogTools.info(robotSide + " taskspace home pose: " + taskspaceHomePose);
      }
   }
}