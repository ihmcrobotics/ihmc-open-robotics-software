package us.ihmc.perception.lerobot;

import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.perception.RawImage;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensors.ImageSensor;
import us.ihmc.sensors.zed.ZEDImageSensor;

public class LeRobotInferenceUpdateThread extends RepeatingTaskThread
{
   private final LeRobotInferenceManager leRobotInferenceManager;
   private final FullHumanoidRobotModel fullRobotModel;
   private final ImageSensor zedSensor;

   public LeRobotInferenceUpdateThread(String policyName, ROS2Node ros2Node, String robotName, FullHumanoidRobotModel fullRobotModel, ImageSensor zedSensor)
   {
      super(LeRobotInferenceUpdateThread.class.getSimpleName());
      this.fullRobotModel = fullRobotModel;

      this.zedSensor = zedSensor;

      setFrequencyLimit(5);

      // TODO: This thing already has an internal thread. We probably want to just use this thread
      //   I guess we could provide an option.
      //   LeRobotInferenceManager could be improved, broken up a bit
      leRobotInferenceManager = new LeRobotInferenceManager(policyName, robotName, fullRobotModel);
      leRobotInferenceManager.setRunning(true); // TODO: Set from ROS 2 callback on new topic that talks to UI
      leRobotInferenceManager.startPythonServer();
   }

   @Override
   protected void runTask() throws Throwable
   {
      try
      {
         zedSensor.waitForGrab(); // TODO: Ask Tomasz if the threading is correct here

         // TODO: We're reading from the synced robot here, is threading okay? (probably not, need to synchronize)
         MovingReferenceFrame leftFrameAfterJoint = fullRobotModel.getHand(RobotSide.LEFT).getParentJoint().getFrameAfterJoint();
         Pose3D leftPose = new Pose3D(leftFrameAfterJoint.getTransformToWorldFrame());
         MovingReferenceFrame rightFrameAfterJoint = fullRobotModel.getHand(RobotSide.RIGHT).getParentJoint().getFrameAfterJoint();
         Pose3D rightPose = new Pose3D(rightFrameAfterJoint.getTransformToWorldFrame());
         leRobotInferenceManager.publishHandPoses(leftPose, rightPose);

         RawImage leftImage = zedSensor.getImage(ZEDImageSensor.LEFT_COLOR_IMAGE_KEY);
         RawImage rightImage = zedSensor.getImage(ZEDImageSensor.RIGHT_COLOR_IMAGE_KEY);

         // TODO: Check the image format, might need conversion
         leRobotInferenceManager.publishImage(RobotSide.LEFT, leftImage.getCpuImageMat());
         leRobotInferenceManager.publishImage(RobotSide.RIGHT, rightImage.getCpuImageMat());

         leftImage.release();
         rightImage.release();

      }
      catch (InterruptedException ex) { }
   }

   public void destroy()
   {
      blockingKill();
      leRobotInferenceManager.destroy();
   }
}
