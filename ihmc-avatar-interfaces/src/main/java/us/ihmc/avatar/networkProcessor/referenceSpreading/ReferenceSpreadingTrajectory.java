package us.ihmc.avatar.networkProcessor.referenceSpreading;

import controller_msgs.msg.dds.HandHybridJointspaceTaskspaceTrajectoryMessage;
import controller_msgs.msg.dds.JointspaceTrajectoryMessage;
import controller_msgs.msg.dds.OneDoFJointTrajectoryMessage;
import controller_msgs.msg.dds.SE3PIDGainsTrajectoryMessage;
import controller_msgs.msg.dds.SE3PIDGainsTrajectoryPointMessage;
import controller_msgs.msg.dds.WrenchTrajectoryMessage;
import controller_msgs.msg.dds.WrenchTrajectoryPointMessage;
import ihmc_common_msgs.msg.dds.QueueableMessage;
import ihmc_common_msgs.msg.dds.SE3TrajectoryMessage;
import ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessage;
import ihmc_common_msgs.msg.dds.TrajectoryPoint1DMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import geometry_msgs.msg.dds.Wrench;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.controllers.pidGains.PID3DGains;
import us.ihmc.robotics.controllers.pidGains.implementations.DefaultPID3DGains;
import us.ihmc.robotics.controllers.pidGains.implementations.PIDGains;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoRegistry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;

import static us.ihmc.euclid.tools.EuclidCoreTools.zeroVector3D;
import static us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools.createSE3TrajectoryMessage;
import static us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools.createSE3TrajectoryPointMessage;

public class ReferenceSpreadingTrajectory
{
   protected final YoRegistry registry;

   private static final double INITIAL_TIME_DURATION = 0;
   private static final double MAX_POINTS = 200; // se3TrajectoryMessage.getTaskspaceTrajectoryPoints().capacity() does not seem to work. So set manually!
   private static final List<String> JOINT_NAMES = Arrays.asList("SHOULDER_Y", "SHOULDER_X", "SHOULDER_Z", "ELBOW_Y", "WRIST_Z", "WRIST_X", "GRIPPER_Z");

   private final TrajectoryRecordReplay trajectoryPlayer;
   private String filePath;
   private List<String> keyMatrix = new ArrayList<>();
   private FullHumanoidRobotModel fullRobotModel;
   DRCRobotModel robotModel;


   private Double startTimeCSV = null;

   ReferenceSpreadingTrajectory(TrajectoryRecordReplay trajectoryPlayer, List<String> keyMatrix, DRCRobotModel robotModel, FullHumanoidRobotModel fullRobotModel, YoRegistry registry)
   {
      this.trajectoryPlayer = trajectoryPlayer;
      this.keyMatrix = keyMatrix;
      this.fullRobotModel = fullRobotModel;
      this.robotModel = robotModel;
      this.registry = registry;
   }

   ReferenceSpreadingTrajectory(String filePath, DRCRobotModel robotModel, FullHumanoidRobotModel fullRobotModel, YoRegistry registry)
   {
      this.fullRobotModel = fullRobotModel;
      this.robotModel = robotModel;
      this.filePath = filePath;
      this.registry = registry;

      trajectoryPlayer = new TrajectoryRecordReplay(filePath, 1, true);
      trajectoryPlayer.importData(true);

      keyMatrix = trajectoryPlayer.getKeyMatrix();
   }

   public HandHybridJointspaceTaskspaceTrajectoryMessage getHandHybridTrajectoryMessage(RobotSide robotSide)
   {
      SE3TrajectoryMessage se3TrajectoryMessage = new SE3TrajectoryMessage();
      se3TrajectoryMessage.getQueueingProperties().setExecutionMode(QueueableMessage.EXECUTION_MODE_OVERRIDE);
//      se3TrajectoryMessage.getLinearWeightMatrix().setXWeight(1.0);
//      se3TrajectoryMessage.getLinearWeightMatrix().setYWeight(1.0);
//      se3TrajectoryMessage.getLinearWeightMatrix().setZWeight(1.0);
//      se3TrajectoryMessage.getAngularWeightMatrix().setXWeight(1.0);
//      se3TrajectoryMessage.getAngularWeightMatrix().setYWeight(1.0);
//      se3TrajectoryMessage.getAngularWeightMatrix().setZWeight(1.0);
      SE3TrajectoryPointMessage se3TrajectoryPointMessage;

      JointspaceTrajectoryMessage jointspaceTrajectoryMessage = new JointspaceTrajectoryMessage();
      jointspaceTrajectoryMessage.getQueueingProperties().setExecutionMode(QueueableMessage.EXECUTION_MODE_OVERRIDE);
      TrajectoryPoint1DMessage jointTrajectoryPointMessage = new TrajectoryPoint1DMessage();
      OneDoFJointBasics joint;
      double jointPosition;
      double jointVelocity;

      for (String jointName : JOINT_NAMES)
      {
         jointspaceTrajectoryMessage.getJointTrajectoryMessages().add().set(new OneDoFJointTrajectoryMessage());
         jointspaceTrajectoryMessage.getJointTrajectoryMessages().get(JOINT_NAMES.indexOf(jointName)).setSequenceId(JOINT_NAMES.indexOf(jointName));
//         jointspaceTrajectoryMessage.getJointTrajectoryMessages().get(JOINT_NAMES.indexOf(jointName)).setWeight(0.01);
      }

      Point3D desiredPosition = new Point3D();
      YawPitchRoll desiredOrientation = new YawPitchRoll();
      Vector3D desiredLinearVelocity = new Vector3D();
      Vector3D desiredAngularVelocity = new Vector3D();

      WrenchTrajectoryMessage wrenchTrajectoryMessage = new WrenchTrajectoryMessage();
      WrenchTrajectoryPointMessage wrenchTrajectoryPointMessage = new WrenchTrajectoryPointMessage();
      Wrench desiredFeedForwardWrench = new Wrench();

      SE3PIDGainsTrajectoryMessage pidGainsTrajectoryMessage = new SE3PIDGainsTrajectoryMessage();
      SE3PIDGainsTrajectoryPointMessage pidGainsTrajectoryPointMessage = new SE3PIDGainsTrajectoryPointMessage();
      pidGainsTrajectoryPointMessage.getLinearGains().set(convertToPID3DGainsMessage(robotModel.getWalkingControllerParameters().getImpedanceHandPositionControlGains()));
      pidGainsTrajectoryPointMessage.getAngularGains().set(convertToPID3DGainsMessage(robotModel.getWalkingControllerParameters().getImpedanceHandOrientationControlGains()));

      LinkedHashMap<String, Double> currentFrame = new LinkedHashMap<>();
      trajectoryPlayer.reset();
      makeMap(trajectoryPlayer.play(false), currentFrame);
      startTimeCSV = currentFrame.get("time[sec]") - INITIAL_TIME_DURATION;
//      LogTools.info("CurrentFrame: " + currentFrame);

      int totalFrames = trajectoryPlayer.getNumberOfLines();
      int frameInterval = Math.max(1, (totalFrames + (int) MAX_POINTS - 2) / ((int) MAX_POINTS - 1));
      int frameIndex = 0;
      int jointIndex;

      while (!trajectoryPlayer.hasDoneReplay())
      {
         makeMap(trajectoryPlayer.play(false), currentFrame);
//         LogTools.info(currentFrame);
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
               joint = fullRobotModel.getOneDoFJointByName(robotSide.getUpperCaseName() + "_" + jointName);
               jointPosition = Math.max(joint.getJointLimitLower(), Math.min(joint.getJointLimitUpper(), currentFrame.get("q_" + robotSide.getUpperCaseName() + "_" + jointName)));
               jointVelocity = Math.max(joint.getVelocityLimitLower(), Math.min(joint.getVelocityLimitUpper(), currentFrame.get("qd_" + robotSide.getUpperCaseName() + "_" + jointName)));
               jointTrajectoryPointMessage.setPosition(jointPosition);
               jointTrajectoryPointMessage.setVelocity(jointVelocity);
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

            if (currentFrame.get("blendingFactor") != null)
            {
               pidGainsTrajectoryPointMessage.getLinearGains().getGainsX().setZeta(currentFrame.get("blendingFactor"));
               pidGainsTrajectoryPointMessage.getAngularGains().getGainsX().setZeta(currentFrame.get("blendingFactor"));
               pidGainsTrajectoryPointMessage.getLinearGains().getGainsY().setZeta(currentFrame.get("blendingFactor"));
               pidGainsTrajectoryPointMessage.getAngularGains().getGainsY().setZeta(currentFrame.get("blendingFactor"));
               pidGainsTrajectoryPointMessage.getLinearGains().getGainsZ().setZeta(currentFrame.get("blendingFactor"));
               pidGainsTrajectoryPointMessage.getAngularGains().getGainsZ().setZeta(currentFrame.get("blendingFactor"));
               pidGainsTrajectoryPointMessage.setTime(currentTime - startTimeCSV);
               pidGainsTrajectoryPointMessage.setSequenceId(frameIndex);
               pidGainsTrajectoryMessage.getPidGainsTrajectoryPoints().add().set(pidGainsTrajectoryPointMessage);
            }
         }
         frameIndex++;
      }

      HandHybridJointspaceTaskspaceTrajectoryMessage handHybridTrajectoryMessage = new HandHybridJointspaceTaskspaceTrajectoryMessage();
      handHybridTrajectoryMessage.getTaskspaceTrajectoryMessage().set(se3TrajectoryMessage);
      handHybridTrajectoryMessage.setRobotSide(robotSide.toByte());
      handHybridTrajectoryMessage.getTaskspaceTrajectoryMessage().getQueueingProperties().setExecutionMode(QueueableMessage.EXECUTION_MODE_OVERRIDE);

      handHybridTrajectoryMessage.getJointspaceTrajectoryMessage().set(jointspaceTrajectoryMessage);
//      handHybridTrajectoryMessage.getFeedforwardTaskspaceTrajectoryMessage().set(wrenchTrajectoryMessage);

      if (!pidGainsTrajectoryMessage.getPidGainsTrajectoryPoints().isEmpty())
         handHybridTrajectoryMessage.getTaskspacePidGainsTrajectoryMessage().set(pidGainsTrajectoryMessage);
      return handHybridTrajectoryMessage;
   }

   public double getStartTimeCSV()
   {
      if (startTimeCSV == null)
      {
         LinkedHashMap<String, Double> currentFrame = new LinkedHashMap<>();
         trajectoryPlayer.reset();
         makeMap(trajectoryPlayer.play(false), currentFrame);
         startTimeCSV = currentFrame.get("time[sec]");
      }
      return startTimeCSV;
   }

   private void makeMap(double[] values, LinkedHashMap<String, Double> mapToPack)
   {
      for (int i = 0; i < values.length; i++)
      {
         mapToPack.put(this.keyMatrix.get(i), values[i]);
      }
   }

   private static controller_msgs.msg.dds.PID3DGains convertToPID3DGainsMessage(us.ihmc.robotics.controllers.pidGains.PID3DGains controllerGains) {
      controller_msgs.msg.dds.PID3DGains msgGains = new controller_msgs.msg.dds.PID3DGains();
      msgGains.getGainsX().setKp(controllerGains.getProportionalGains()[0]);
      msgGains.getGainsX().setKi(controllerGains.getIntegralGains()[0]);
      msgGains.getGainsX().setKd(controllerGains.getDerivativeGains()[0]);
      msgGains.getGainsY().setKp(controllerGains.getProportionalGains()[1]);
      msgGains.getGainsY().setKi(controllerGains.getIntegralGains()[1]);
      msgGains.getGainsY().setKd(controllerGains.getDerivativeGains()[1]);
      msgGains.getGainsZ().setKp(controllerGains.getProportionalGains()[2]);
      msgGains.getGainsZ().setKi(controllerGains.getIntegralGains()[2]);
      msgGains.getGainsZ().setKd(controllerGains.getDerivativeGains()[2]);
      msgGains.setMaximumIntegralError(controllerGains.getMaximumIntegralError());
      msgGains.setMaximumFeedback(controllerGains.getMaximumFeedback());
      msgGains.setMaximumFeedbackRate(controllerGains.getMaximumFeedbackRate());
      msgGains.setMaximumDerivativeError(controllerGains.getMaximumDerivativeError());
      msgGains.setMaximumProportionalError(controllerGains.getMaximumProportionalError());
      return msgGains;
   }
}
