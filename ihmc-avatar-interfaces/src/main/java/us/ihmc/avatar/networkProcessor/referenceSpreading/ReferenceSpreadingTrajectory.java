package us.ihmc.avatar.networkProcessor.referenceSpreading;

import controller_msgs.msg.dds.HandHybridJointspaceTaskspaceTrajectoryMessage;
import controller_msgs.msg.dds.JointspaceTrajectoryMessage;
import controller_msgs.msg.dds.OneDoFJointTrajectoryMessage;
import controller_msgs.msg.dds.WrenchTrajectoryMessage;
import controller_msgs.msg.dds.WrenchTrajectoryPointMessage;
import ihmc_common_msgs.msg.dds.QueueableMessage;
import ihmc_common_msgs.msg.dds.SE3TrajectoryMessage;
import ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessage;
import ihmc_common_msgs.msg.dds.TrajectoryPoint1DMessage;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import geometry_msgs.msg.dds.Wrench;
import us.ihmc.log.LogTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

import static us.ihmc.euclid.tools.EuclidCoreTools.zeroVector3D;
import static us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools.createSE3TrajectoryMessage;
import static us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools.createSE3TrajectoryPointMessage;

public class ReferenceSpreadingTrajectory
{
   private static final double INITIAL_TIME_DURATION = 0.5;
   private static final double MAX_POINTS = 200; // se3TrajectoryMessage.getTaskspaceTrajectoryPoints().capacity() does not seem to work. So set manually!
   private static final List<String> JOINT_NAMES = Arrays.asList("SHOULDER_Y", "SHOULDER_X", "SHOULDER_Z", "ELBOW_Y", "WRIST_Z", "WRIST_X", "GRIPPER_Z");

   private final TrajectoryRecordReplay trajectoryPlayer;
   private String filePath;
   private List<String> keyMatrix = new ArrayList<>();
   FullHumanoidRobotModel fullRobotModel;

   ReferenceSpreadingTrajectory(String filePath, FullHumanoidRobotModel fullRobotModel)
   {
      this.fullRobotModel = fullRobotModel;
      this.filePath = filePath;

      trajectoryPlayer = new TrajectoryRecordReplay(filePath, 1, true);
      trajectoryPlayer.importData(true);

      keyMatrix = trajectoryPlayer.getKeyMatrix();
   }

   public HandHybridJointspaceTaskspaceTrajectoryMessage getHandHybridTrajectoryMessage(RobotSide robotSide)
   {
//      trajectoryPlayer.reset();
      SE3TrajectoryMessage se3TrajectoryMessage = new SE3TrajectoryMessage();
      se3TrajectoryMessage.getQueueingProperties().setExecutionMode(QueueableMessage.EXECUTION_MODE_OVERRIDE);
      SE3TrajectoryPointMessage se3TrajectoryPointMessage;

      JointspaceTrajectoryMessage jointspaceTrajectoryMessage = new JointspaceTrajectoryMessage();
      jointspaceTrajectoryMessage.getQueueingProperties().setExecutionMode(QueueableMessage.EXECUTION_MODE_OVERRIDE);
      TrajectoryPoint1DMessage jointTrajectoryPointMessage = new TrajectoryPoint1DMessage();

      for (String jointName : JOINT_NAMES)
      {
         jointspaceTrajectoryMessage.getJointTrajectoryMessages().add().set(new OneDoFJointTrajectoryMessage());
         jointspaceTrajectoryMessage.getJointTrajectoryMessages().get(JOINT_NAMES.indexOf(jointName)).setSequenceId(JOINT_NAMES.indexOf(jointName));
      }

      Point3D desiredPosition = new Point3D();
      YawPitchRoll desiredOrientation = new YawPitchRoll();
      Vector3D desiredLinearVelocity = new Vector3D();
      Vector3D desiredAngularVelocity = new Vector3D();

      WrenchTrajectoryMessage wrenchTrajectoryMessage = new WrenchTrajectoryMessage();
      WrenchTrajectoryPointMessage wrenchTrajectoryPointMessage = new WrenchTrajectoryPointMessage();
      Wrench desiredFeedForwardWrench = new Wrench();

      HashMap<String, Double> currentFrame = new HashMap<>();
//      trajectoryPlayer.reset();
      makeMap(trajectoryPlayer.play(true), currentFrame);
      double startTimeCSV = currentFrame.get("time[sec]") - INITIAL_TIME_DURATION;

      int totalFrames = trajectoryPlayer.getNumberOfLines();
      int frameInterval = Math.max(1, (totalFrames + (int) MAX_POINTS - 2) / ((int) MAX_POINTS - 1));
      LogTools.info("Total frames: " + totalFrames + ", frame interval: " + frameInterval + ", Calculated total frames: " + (totalFrames/frameInterval));
      int frameIndex = 0;
      int jointIndex;

      while (!trajectoryPlayer.hasDoneReplay())
      {
         makeMap(trajectoryPlayer.play(true), currentFrame);
         if (frameIndex % frameInterval == 0)
         {
            double currentTime = currentFrame.get("time[sec]");
            desiredPosition.set(currentFrame.get(robotSide.getUpperCaseName() + "_GRIPPER_YAW_LINKCurrent" + "PositionX"),
                                currentFrame.get(robotSide.getUpperCaseName() + "_GRIPPER_YAW_LINKCurrent" + "PositionY"),
                                currentFrame.get(robotSide.getUpperCaseName() + "_GRIPPER_YAW_LINKCurrent" + "PositionZ"));
            desiredOrientation.setQuaternion(currentFrame.get(robotSide.getUpperCaseName() + "_GRIPPER_YAW_LINKCurrent" + "OrientationQx"),
                                             currentFrame.get(robotSide.getUpperCaseName() + "_GRIPPER_YAW_LINKCurrent" + "OrientationQy"),
                                             currentFrame.get(robotSide.getUpperCaseName() + "_GRIPPER_YAW_LINKCurrent" + "OrientationQz"),
                                             currentFrame.get(robotSide.getUpperCaseName() + "_GRIPPER_YAW_LINKCurrent" + "OrientationQs"));

            desiredLinearVelocity.set(currentFrame.get(robotSide.getUpperCaseName() + "_GRIPPER_YAW_LINKCurrent" + "LinearVelocityX"),
                                      currentFrame.get(robotSide.getUpperCaseName() + "_GRIPPER_YAW_LINKCurrent" + "LinearVelocityY"),
                                      currentFrame.get(robotSide.getUpperCaseName() + "_GRIPPER_YAW_LINKCurrent" + "LinearVelocityZ"));
            desiredAngularVelocity.set(currentFrame.get(robotSide.getUpperCaseName() + "_GRIPPER_YAW_LINKCurrent" + "AngularVelocityX"),
                                       currentFrame.get(robotSide.getUpperCaseName() + "_GRIPPER_YAW_LINKCurrent" + "AngularVelocityY"),
                                       currentFrame.get(robotSide.getUpperCaseName() + "_GRIPPER_YAW_LINKCurrent" + "AngularVelocityZ"));

            for (String jointName : JOINT_NAMES)
            {
               jointIndex = JOINT_NAMES.indexOf(jointName);

               jointTrajectoryPointMessage.setTime(currentTime - startTimeCSV);
               jointTrajectoryPointMessage.setPosition(currentFrame.get("q_" + robotSide.getUpperCaseName() + "_" + jointName));
               jointTrajectoryPointMessage.setVelocity(currentFrame.get("qd_" + robotSide.getUpperCaseName() + "_" + jointName));
               jointTrajectoryPointMessage.setSequenceId(frameIndex);
               jointspaceTrajectoryMessage.getJointTrajectoryMessages().get(jointIndex).getTrajectoryPoints().add().set(jointTrajectoryPointMessage);
            }

            se3TrajectoryPointMessage = createSE3TrajectoryPointMessage(currentTime - startTimeCSV,
                                                                        desiredPosition,
                                                                        desiredOrientation,
                                                                        desiredLinearVelocity,
                                                                        desiredAngularVelocity);
            se3TrajectoryPointMessage.setSequenceId(frameIndex);
            se3TrajectoryMessage.getTaskspaceTrajectoryPoints().add().set(se3TrajectoryPointMessage);

            desiredFeedForwardWrench.getForce().set(currentFrame.get(robotSide.getUpperCaseName() + "_GRIPPER_YAW_LINKpositionFeedbackX"),
                                                    currentFrame.get(robotSide.getUpperCaseName() + "_GRIPPER_YAW_LINKpositionFeedbackY"),
                                                    currentFrame.get(robotSide.getUpperCaseName() + "_GRIPPER_YAW_LINKpositionFeedbackZ"));

            desiredFeedForwardWrench.getTorque().set(currentFrame.get(robotSide.getUpperCaseName() + "_GRIPPER_YAW_LINKorientationFeedbackX"),
                                                     currentFrame.get(robotSide.getUpperCaseName() + "_GRIPPER_YAW_LINKorientationFeedbackY"),
                                                     currentFrame.get(robotSide.getUpperCaseName() + "_GRIPPER_YAW_LINKorientationFeedbackZ"));

            wrenchTrajectoryPointMessage.setTime(currentTime - startTimeCSV);
            wrenchTrajectoryPointMessage.getWrench().set(desiredFeedForwardWrench);
            wrenchTrajectoryPointMessage.setSequenceId(frameIndex);
            wrenchTrajectoryMessage.getWrenchTrajectoryPoints().add().set(wrenchTrajectoryPointMessage);
         }
         frameIndex++;
      }

      HandHybridJointspaceTaskspaceTrajectoryMessage handHybridTrajectoryMessage = new HandHybridJointspaceTaskspaceTrajectoryMessage();
      handHybridTrajectoryMessage.getTaskspaceTrajectoryMessage().set(se3TrajectoryMessage);
      handHybridTrajectoryMessage.setRobotSide(robotSide.toByte());
      handHybridTrajectoryMessage.getTaskspaceTrajectoryMessage().getQueueingProperties().setExecutionMode(QueueableMessage.EXECUTION_MODE_OVERRIDE);

      handHybridTrajectoryMessage.getJointspaceTrajectoryMessage().set(jointspaceTrajectoryMessage);
      handHybridTrajectoryMessage.getFeedforwardTaskspaceTrajectoryMessage().set(wrenchTrajectoryMessage);
      return handHybridTrajectoryMessage;
   }

   private void makeMap(double[] values, HashMap<String, Double> mapToPack)
   {
      for (int i = 0; i < values.length; i++)
      {
         mapToPack.put(trajectoryPlayer.getKeyMatrix().get(i), values[i]);
      }
   }
}
