package us.ihmc.rdx.ui.graphics;

import toolbox_msgs.msg.dds.KinematicsToolboxOutputStatus;
import toolbox_msgs.msg.dds.WalkingControllerPreviewOutputMessage;
import us.ihmc.communication.ros2.ROS2ControllerPublishSubscribeAPI;
import us.ihmc.commons.MathTools;
import us.ihmc.ros2.ROS2Input;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;

import java.util.concurrent.atomic.AtomicBoolean;

public class RDXWalkingPreviewPlaybackManager
{
   private final ROS2Input<WalkingControllerPreviewOutputMessage> walkingPreviewOutput;
   // frames per call to handle()
   private final int playbackSpeed = 1;

   private int playbackCounter = 0;
   private FullHumanoidRobotModel previewRobotModel = null;
   private OneDoFJointBasics[] previewModelOneDoFJoints = null;

   // whether to animate ghost robot
   private final AtomicBoolean playbackModeActive = new AtomicBoolean(false);

   public RDXWalkingPreviewPlaybackManager(ROS2ControllerPublishSubscribeAPI helper)
   {
      walkingPreviewOutput = helper.subscribe(ROS2Tools.getControllerPreviewOutputTopic(helper.getRobotName()));
   }

   public void create(FullHumanoidRobotModel previewRobotModel)
   {
      this.previewRobotModel = previewRobotModel;
      previewModelOneDoFJoints = FullRobotModelUtils.getAllJointsExcludingHands(previewRobotModel);
   }

   public void setPlaybackMode(boolean active)
   {
      if (active && active != playbackModeActive.get())
         playbackCounter = 0;

      playbackModeActive.set(active);
   }

   public void update()
   {
      if (playbackModeActive.get())
      {
         WalkingControllerPreviewOutputMessage walkingControllerPreviewOutputMessage = walkingPreviewOutput.getLatest();

         if (walkingControllerPreviewOutputMessage == null)
         {
            LogTools.info("No preview in memory.");
            playbackModeActive.set(false);
            return;
         }

         if (playbackCounter >= walkingControllerPreviewOutputMessage.getRobotConfigurations().size())
         {
            playbackCounter = 0;
         }

         setToFrame(playbackCounter);
         playbackCounter += playbackSpeed;
         double alpha = ((double) playbackCounter) / (walkingControllerPreviewOutputMessage.getRobotConfigurations().size() - 1);
//         previewSlider.setValue(MathTools.clamp(alpha, 0.0, 1.0));
      }
   }

   public void requestSpecificPercentageInPreview(double alpha)
   {
      if (playbackModeActive.get())
         return;

      alpha = MathTools.clamp(alpha, 0.0, 1.0);
      WalkingControllerPreviewOutputMessage walkingControllerPreviewOutputMessage = walkingPreviewOutput.getLatest();

      if (walkingControllerPreviewOutputMessage == null)
      {
         LogTools.info("No preview in memory.");
         playbackModeActive.set(false);
         return;
      }

      int frameIndex = (int) (alpha * (walkingControllerPreviewOutputMessage.getRobotConfigurations().size() - 1));
      setToFrame(frameIndex);
   }

   private void setToFrame(int frameIndex)
   {
      WalkingControllerPreviewOutputMessage walkingControllerPreviewOutputMessage = walkingPreviewOutput.getLatest();

      if (walkingControllerPreviewOutputMessage == null)
      {
         LogTools.info("No preview in memory.");
         playbackModeActive.set(false);
         return;
      }

      us.ihmc.idl.IDLSequence.Object<KinematicsToolboxOutputStatus> robotConfigurations = walkingControllerPreviewOutputMessage.getRobotConfigurations();

      if (frameIndex >= robotConfigurations.size())
      {
         LogTools.info("frameIndex out of bound.");
         playbackModeActive.set(false);
         return;
      }

      KinematicsToolboxOutputStatus kinematicsToolboxOutputStatus = robotConfigurations.get(frameIndex);

      us.ihmc.idl.IDLSequence.Float jointAngles = kinematicsToolboxOutputStatus.getDesiredJointAngles();

      if (jointAngles.size() != previewModelOneDoFJoints.length)
      {
         System.err.println("Received " + jointAngles.size() + " from walking controller preview toolbox, expected " + previewModelOneDoFJoints.length);
//         walkingPreviewOutput.set(null);
         return;
      }

      for (int i = 0; i < jointAngles.size(); i++)
      {
         previewModelOneDoFJoints[i].setQ(jointAngles.get(i));
      }

      previewRobotModel.getRootJoint().setJointPosition(kinematicsToolboxOutputStatus.getDesiredRootPosition());
      previewRobotModel.getRootJoint().setJointOrientation(kinematicsToolboxOutputStatus.getDesiredRootOrientation());
   }
}