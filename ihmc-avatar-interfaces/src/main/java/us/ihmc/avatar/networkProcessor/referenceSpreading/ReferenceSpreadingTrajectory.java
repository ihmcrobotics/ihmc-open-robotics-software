package us.ihmc.avatar.networkProcessor.referenceSpreading;

import boofcv.gui.d3.Orientation3D;
import controller_msgs.msg.dds.HandTrajectoryMessage;
import ihmc_common_msgs.msg.dds.SE3TrajectoryMessage;
import ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessage;
import us.ihmc.behaviors.tools.TrajectoryRecordReplay;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.log.LogTools;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import static us.ihmc.euclid.tools.EuclidCoreTools.zeroVector3D;
import static us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools.createSE3TrajectoryMessage;
import static us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools.createSE3TrajectoryPointMessage;

public class ReferenceSpreadingTrajectory
{
   private static final double INITIAL_TIME_DURATION = 0.0;
   private static final double MAX_POINTS = 200; // se3TrajectoryMessage.getTaskspaceTrajectoryPoints().capacity() does not seem to work. So set manually!

   private final TrajectoryRecordReplay trajectoryPlayer;
   private String filePath;
   private List<String> keyMatrix = new ArrayList<>();

   ReferenceSpreadingTrajectory(String filePath)
   {
      this.filePath = filePath;

      trajectoryPlayer = new TrajectoryRecordReplay(filePath, 1, true);
      trajectoryPlayer.importData(true);

      keyMatrix = trajectoryPlayer.getKeyMatrix();
   }

   // Warning: Assumption is taken that all data is in the same frame.
   public HandTrajectoryMessage getHandTrajectoryMessage(RobotSide robotSide, Double startTime)
   {
      trajectoryPlayer.reset();
      SE3TrajectoryMessage se3TrajectoryMessage = new SE3TrajectoryMessage();
      SE3TrajectoryPointMessage se3TrajectoryPointMessage;
      long frameId = 0;

      Point3D desiredPosition = new Point3D();
      YawPitchRoll desiredOrientation = new YawPitchRoll();

      HashMap<String, Double> currentFrame = new HashMap<>();
      makeMap(trajectoryPlayer.play(true), currentFrame);
      Double startTimeCSV = currentFrame.get("time[sec]") - INITIAL_TIME_DURATION;

      LogTools.info("Start time: " + startTime);

      String name = robotSide.getUpperCaseName() + "_GRIPPER_YAW_LINKCurrent";
      int totalFrames = trajectoryPlayer.getNumberOfLines();
      int frameInterval = Math.max(1, (totalFrames + (int) MAX_POINTS - 2) / ((int) MAX_POINTS - 1));
      int currentFrameIndex = 0;

      while (!trajectoryPlayer.hasDoneReplay())
      {
         makeMap(trajectoryPlayer.play(true), currentFrame);
         if (currentFrameIndex % frameInterval == 0)
         {
            double currentTime = currentFrame.get("time[sec]");
            desiredPosition.set(currentFrame.get(name + "PositionX"),
                                currentFrame.get(name + "PositionY"),
                                currentFrame.get(name + "PositionZ"));
            desiredOrientation.setQuaternion(currentFrame.get(name + "OrientationQx"),
                                             currentFrame.get(name + "OrientationQy"),
                                             currentFrame.get(name + "OrientationQz"),
                                             currentFrame.get(name + "OrientationQs"));

            se3TrajectoryPointMessage = createSE3TrajectoryPointMessage(currentTime - startTimeCSV + startTime,
                                                                        desiredPosition,
                                                                        desiredOrientation,
                                                                        zeroVector3D,
                                                                        zeroVector3D);
            se3TrajectoryPointMessage.setSequenceId(frameId++);
            se3TrajectoryMessage.getTaskspaceTrajectoryPoints().add().set(se3TrajectoryPointMessage);
         }
         currentFrameIndex++;
      }

      HandTrajectoryMessage handTrajectoryMessage = new HandTrajectoryMessage();
      handTrajectoryMessage.getSe3Trajectory().set(se3TrajectoryMessage);
      handTrajectoryMessage.setRobotSide(robotSide.toByte());
      return handTrajectoryMessage;
   }

   private void makeMap(double[] values, HashMap<String, Double> mapToPack)
   {
      for (int i = 0; i < values.length; i++)
      {
         mapToPack.put(trajectoryPlayer.getKeyMatrix().get(i), values[i]);
      }
   }
}
