package us.ihmc.lerobot;

import behavior_msgs.VLAOperationMessage;
import ihmc_common_msgs.YoRegistryMessage;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Point;
import org.bytedeco.opencv.opencv_core.Rect;
import org.bytedeco.opencv.opencv_core.Size;
import toolbox_msgs.KinematicsStreamingToolboxInputMessage;
import toolbox_msgs.KinematicsToolboxOutputStatus;
import toolbox_msgs.KinematicsToolboxRigidBodyMessage;
import toolbox_msgs.ToolboxStateMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.logProcessor.leRobot.LeRobotDataset;
import us.ihmc.avatar.networkProcessor.kinematicsStreamingToolboxModule.KinematicsStreamingToolboxModule;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.communication.HumanoidROS2Topic;
import us.ihmc.communication.ToolboxAPIs;
import us.ihmc.communication.crdt.CRDTBidirectionalBoolean;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.LatestTimestampModifiable;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.packets.ToolboxState;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.communication.ros2.ROS2IOTopicPair;
import us.ihmc.communication.ros2.sync.ROS2PeerClockOffsetEstimator;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2QoSProfile;
import us.ihmc.jros2.ROS2Publisher;
import us.ihmc.jros2.ROS2Topic;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.openpi.OpenpiClient;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.imageMessage.PixelFormat;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensors.ImageSensor;
import us.ihmc.yoVariables.euclid.YoPose3D;

import java.nio.ByteBuffer;
import java.nio.DoubleBuffer;
import java.util.ArrayDeque;
import java.util.Deque;
import java.util.concurrent.CompletableFuture;

/**
 * Autonomy process thread for managing vision-language-action (VLA) inference and supporting remote UI.
 * Manages communication with the Python side, which is running the openpi.
 */
public class VLAUpdateThread extends VLAYoRegistry
{
   public static final ROS2IOTopicPair<VLAOperationMessage> UI = new ROS2IOTopicPair<>(new HumanoidROS2Topic<>().withPrefix("vla_ui")
                                                                                                                .withTypeName(VLAOperationMessage.class)
                                                                                                                .withQoS(ROS2QoSProfile.BEST_EFFORT));
   public static final ROS2Topic<YoRegistryMessage> YO = new HumanoidROS2Topic<>().withPrefix("vla_yo")
                                                                                  .withTypeName(YoRegistryMessage.class)
                                                                                  .withQoS(ROS2QoSProfile.BEST_EFFORT);
   private final RepeatingTaskThread thread = new RepeatingTaskThread(getClass().getSimpleName(), this::runTask).setFrequencyLimit(50.0);
   private final ROS2SyncedRobotModel syncedRobot;
   private final ImageSensor zedSensor;
   private String status = "Not connected to openpi";
   private final FramePose3D framePose = new FramePose3D();
   private final OpenpiClient openpiClient = new OpenpiClient("10.6.192.65", LeRobotDataset.STATE_SIZE);
   private boolean requestValid = false;
   private CompletableFuture<byte[]> openpiRequest;
   private final Notification requested = new Notification();
   private final Deque<DoubleBuffer> actionPlan = new ArrayDeque<>();
   private final SideDependentList<Mat> images = new SideDependentList<>(new Mat(224, 224, opencv_core.CV_8UC3), new Mat(224, 224, opencv_core.CV_8UC3));
   private final YoRegistryMessage yoRegistryMessage = new YoRegistryMessage();
   private final ROS2Publisher<YoRegistryMessage> yoPublisher;

   private final LatestTimestampModifiable latestTimestampModifiable;
   private long sequenceID = 0L;
   private final CRDTBidirectionalBoolean running;
   private final CRDTBidirectionalBoolean controlRobot;
   private final TypedNotification<VLAOperationMessage> uiCommandSubscription;
   private final VLAOperationMessage uiCommandCopy = new VLAOperationMessage();
   private final KinematicsToolboxOutputStatus kstStatusCopy = new KinematicsToolboxOutputStatus();
   private final ROS2Publisher<VLAOperationMessage> uiStatusPublisher;

   private final FullHumanoidRobotModel kstFullRobotModel;
   private final HumanoidReferenceFrames kstReferenceFrames;
   private final OneDoFJointBasics[] kstOneDoFJointsExcludingHands;
   private final TypedNotification<KinematicsToolboxOutputStatus> kstStatusSubscription = new TypedNotification<>();

   private final ROS2Publisher<KinematicsStreamingToolboxInputMessage> kstInputPublisher;
   private final ROS2Publisher<ToolboxStateMessage> kstStatePublisher;

   public VLAUpdateThread(ROS2Node ros2Node,
                          ROS2PeerClockOffsetEstimator clockOffsetEstimator,
                          DRCRobotModel robotModel,
                          ROS2SyncedRobotModel syncedRobot,
                          ImageSensor zedSensor)
   {
      this.syncedRobot = syncedRobot;
      this.zedSensor = zedSensor;

      actionHandPoses.forEach(YoPose3D::setToNaN);
      actionForearmPoses.forEach(YoPose3D::setToNaN);
      yoPublisher = ros2Node.createPublisher(YO);

      latestTimestampModifiable = new LatestTimestampModifiable(new CRDTInfo(ROS2ActorDesignation.ROBOT, clockOffsetEstimator));
      latestTimestampModifiable.modify(); // On startup, we want the initial state to propagate
      running = new CRDTBidirectionalBoolean(latestTimestampModifiable, false);
      controlRobot = new CRDTBidirectionalBoolean(latestTimestampModifiable, false);

      var uiCommandTopic = UI.getTopic(ROS2ActorDesignation.ROBOT.getIncomingQualifier());
      TypedNotification<VLAOperationMessage> typedNotification = new TypedNotification<>();
      ros2Node.createSubscriptionSampler(uiCommandTopic, sample ->
      {
         uiCommandCopy.set(sample);
         typedNotification.set(uiCommandCopy);
      });
      uiCommandSubscription = typedNotification;
      uiStatusPublisher = ros2Node.createPublisher(UI.getTopic(ROS2ActorDesignation.ROBOT.getOutgoingQualifier()));

      kstFullRobotModel = robotModel.createFullRobotModel();
      kstReferenceFrames = new HumanoidReferenceFrames(kstFullRobotModel, robotModel.getSensorInformation());
      kstOneDoFJointsExcludingHands = FullRobotModelUtils.getAllJointsExcludingHands(kstFullRobotModel);
      ros2Node.createSubscriptionSampler(KinematicsStreamingToolboxModule.getOutputStatusTopic(robotModel.getSimpleRobotName()), sample ->
      {
         kstStatusCopy.set(sample);
         kstStatusSubscription.set(kstStatusCopy);
      });

      kstInputPublisher = ros2Node.createPublisher(ToolboxAPIs.getIKStreamingInputTopic(robotModel.getSimpleRobotName()));
      kstStatePublisher = ros2Node.createPublisher(ToolboxAPIs.getIKStreamingStateTopic(robotModel.getSimpleRobotName()));
   }

   public void runTask()
   {
      handUISyncBefore();

      if (running.getValue())
      {
         if (openpiRequest == null)
         {
            tryOpenpiRequest();
         }
         else if (openpiRequest.isDone())
         {
            if (!openpiRequest.isCompletedExceptionally())
            {
               if (openpiClient.unpack(openpiRequest))
               {
                  DoubleBuffer actionChunk = openpiClient.getActionChunk().asDoubleBuffer();
                  int actionSize = LeRobotDataset.STATE_SIZE;
                  for (int i = 0; i < openpiClient.getHorizon(); i++)
                  {
                     DoubleBuffer action = DoubleBuffer.allocate(actionSize);
                     action.put(0, actionChunk, i * actionSize, actionSize);
                     actionPlan.addLast(action);
                  }
                  numberOfActionsReceived.add(actionPlan.size());
               }
            }

            openpiRequest = null;
         }

         DoubleBuffer action = actionPlan.pollFirst();
         if (action != null)
         {
            dispatchActionToKST(action);
         }
      }
      else
      {
         actionPlan.clear();
         status = "Not running";
      }

      MessageTools.toMessage(registry, yoRegistryMessage);
      yoPublisher.publish(yoRegistryMessage);

      handleUISyncAfter();
   }

   private void tryOpenpiRequest()
   {
      if (actionPlan.isEmpty())
      {
         requestValid = true;

         packRequestState();

         packRequestImages();

         if (requestValid)
         {
            openpiRequest = openpiClient.request();
            if (openpiRequest == null)
               status = "Could not connect to server at ws://" + openpiClient.getHost() + ":" + openpiClient.getPort();
            else
            {
               requested.set();
               status = "Requested inference...";
            }
         }
         else
         {
            status = "Waiting for robot data...";
         }
      }
   }

   private void packRequestState()
   {
      ByteBuffer state = openpiClient.getState();
      state.clear();
      synchronized (syncedRobot)
      {
         requestValid &= syncedRobot.getDataReceptionTimerSnapshot().isRunning(0.02);
         requestValid &= kstStatusSubscription.peekHasValue();
         if (requestValid)
            requestValid = kstStatusSubscription.peek().getJointNameHash() != -1;
         if (requestValid)
         {
            kstStatusSubscription.poll();
            KinematicsToolboxOutputStatus kstStatus = kstStatusSubscription.read();
            kstFullRobotModel.getRootJoint().setJointPosition(kstStatus.getDesiredRootPosition().getPoint());
            kstFullRobotModel.getRootJoint().setJointOrientation(kstStatus.getDesiredRootOrientation().getQuaternion());
            for (int i = 0; i < kstOneDoFJointsExcludingHands.length; i++)
               kstOneDoFJointsExcludingHands[i].setQ(kstStatus.getDesiredJointAngles().get(i));
            kstFullRobotModel.getElevator().updateFramesRecursively();
            kstReferenceFrames.updateFrames();

            FullHumanoidRobotModel inputModel = syncedRobot.getFullRobotModel();
            HumanoidReferenceFrames inputReferenceFrames = syncedRobot.getReferenceFrames();

            for (RobotSide side : RobotSide.values)
            {
               if (side == RobotSide.RIGHT || !LeRobotDataset.XYZ_RIGHT_ONLY)
               {
                  YoPose3D stateHandPose = stateHandPoses.get(side);
                  framePose.setToZero(inputModel.getHand(side).getParentJoint().getFrameAfterJoint());
                  framePose.changeFrame(inputReferenceFrames.getExperimentalCameraFrame());
                  stateHandPose.set(framePose);
                  state.putFloat(stateHandPose.getPosition().getX32());
                  state.putFloat(stateHandPose.getPosition().getY32());
                  state.putFloat(stateHandPose.getPosition().getZ32());
                  if (!LeRobotDataset.XYZ_RIGHT_ONLY)
                  {
                     state.putFloat(stateHandPose.getOrientation().getX32());
                     state.putFloat(stateHandPose.getOrientation().getY32());
                     state.putFloat(stateHandPose.getOrientation().getZ32());
                     state.putFloat(stateHandPose.getOrientation().getS32());
                     YoPose3D stateForearmPose = stateForearmPoses.get(side);
                     framePose.setToZero(inputModel.getForearm(side).getParentJoint().getFrameAfterJoint());
                     framePose.changeFrame(inputReferenceFrames.getExperimentalCameraFrame());
                     stateForearmPose.set(framePose);
                     state.putFloat(stateForearmPose.getPosition().getX32());
                     state.putFloat(stateForearmPose.getPosition().getY32());
                     state.putFloat(stateForearmPose.getPosition().getZ32());
                     state.putFloat(stateForearmPose.getOrientation().getX32());
                     state.putFloat(stateForearmPose.getOrientation().getY32());
                     state.putFloat(stateForearmPose.getOrientation().getZ32());
                     state.putFloat(stateForearmPose.getOrientation().getS32());
                  }
               }
            }
         }
      }
   }

   private void packRequestImages()
   {
      for (RobotSide side : RobotSide.values)
      {
         RawImage image = zedSensor.getImage(zedSensor.getImageKeys()[side.ordinal()]);

         requestValid &= image != null;
         if (requestValid)
         {
            Mat rgbColor = new Mat();
            image.getPixelFormat().convertToPixelFormat(image.getCpuImageMat(), rgbColor, PixelFormat.RGB8);
            image.release();

            Size cropSize = new Size(224, 224); // Square frame for siglip
            int scaleWidth = image.getWidth() * cropSize.height() / image.getHeight(); // Account for aspect ratio
            Size scaleDownSize = new Size(scaleWidth, cropSize.height());
            Mat resized = new Mat(scaleDownSize, opencv_core.CV_8UC3);
            opencv_imgproc.resize(rgbColor, resized, scaleDownSize);
            scaleDownSize.close();
            rgbColor.release();

            Point cropOffset = new Point((resized.cols() - cropSize.width()) / 2, 0); // Center crop horizontally
            Rect roi = new Rect(cropOffset, cropSize);
            Mat cropped = new Mat(resized, roi);
            cropped.copyTo(images.get(side));
            resized.close();
            cropSize.close();
            cropOffset.close();
            roi.close();

            Mat continuous = new Mat();
            cropped.copyTo(continuous);
            cropped.close();

            continuous.data().get(openpiClient.getImages().get(side).array());
            continuous.close();
         }
      }
   }

   private void dispatchActionToKST(DoubleBuffer action)
   {
      actionTimestampNanos.set(System.nanoTime()); // TODO: Get this from the policy on the python side?
      numberOfActionsTaken.add(1);
      status = "Taking action %d".formatted(numberOfActionsTaken.getValue());

      synchronized (syncedRobot)
      {
         HumanoidReferenceFrames outputReferenceFrames = syncedRobot.getReferenceFrames();
         int i = 0;
         if (LeRobotDataset.XYZ_RIGHT_ONLY)
         {
            framePose.setToZero(outputReferenceFrames.getExperimentalCameraFrame());
            framePose.getPosition().set(action.get(i++), action.get(i++), action.get(i++));
            boolean circles = true;
            if (circles)
            {
               framePose.getOrientation().setYawPitchRoll(1.0, 0.0, -0.80); // constant from VLAOperation circles
               framePose.changeFrame(ReferenceFrame.getWorldFrame());
               actionHandPoses.get(RobotSide.RIGHT).set(framePose);
            }
            else
            {
               framePose.changeFrame(ReferenceFrame.getWorldFrame());
               // Maintain initial orientation
               framePose.getOrientation().set(outputReferenceFrames.getHandFrame(RobotSide.RIGHT).getTransformToRoot().getRotation());
               actionHandPoses.get(RobotSide.RIGHT).set(framePose);
               framePose.setToZero(outputReferenceFrames.getExperimentalCameraFrame());
               // framePose.getPosition().interpolate(actionHandPoses.get());
               framePose.changeFrame(ReferenceFrame.getWorldFrame());
               actionForearmPoses.get(RobotSide.RIGHT).set(actionHandPoses.get(RobotSide.RIGHT)); // match orientation
            }
         }
         else
         {
            for (RobotSide side : RobotSide.values)
            {
               framePose.setToZero(outputReferenceFrames.getExperimentalCameraFrame());
               framePose.getPosition().set(action.get(i++), action.get(i++), action.get(i++));
               framePose.getOrientation().set(action.get(i++), action.get(i++), action.get(i++), action.get(i++));
               framePose.changeFrame(ReferenceFrame.getWorldFrame());
               actionHandPoses.get(side).set(framePose);
               framePose.setToZero(outputReferenceFrames.getExperimentalCameraFrame());
               framePose.getPosition().set(action.get(i++), action.get(i++), action.get(i++));
               framePose.getOrientation().set(action.get(i++), action.get(i++), action.get(i++), action.get(i++));
               framePose.changeFrame(ReferenceFrame.getWorldFrame());
               actionForearmPoses.get(side).set(framePose);
            }
         }
      }

      KinematicsStreamingToolboxInputMessage ikInputMessage = new KinematicsStreamingToolboxInputMessage();
      ikInputMessage.setStreamToController(controlRobot.getValue());
      ikInputMessage.setTimestamp(actionTimestampNanos.getValue());

      for (RobotSide side : RobotSide.values)
      {
         if (side == RobotSide.RIGHT || !LeRobotDataset.XYZ_RIGHT_ONLY)
         {
            KinematicsToolboxRigidBodyMessage rigidBodyMessage = new KinematicsToolboxRigidBodyMessage();
            rigidBodyMessage.setEndEffectorHashCode(syncedRobot.getFullRobotModel().getHand(side).hashCode());
            rigidBodyMessage.getDesiredPositionInWorld().set(actionHandPoses.get(side).getTranslation());
            rigidBodyMessage.getDesiredOrientationInWorld().set(actionHandPoses.get(side).getRotation());
            rigidBodyMessage.getAngularWeightMatrix().setXWeight(0.02);
            rigidBodyMessage.getAngularWeightMatrix().setYWeight(0.02);
            rigidBodyMessage.getAngularWeightMatrix().setZWeight(0.02);
            ikInputMessage.getInputs().add().set(rigidBodyMessage);

            boolean controlForearm = false;
            if (controlForearm)
            {
               rigidBodyMessage = new KinematicsToolboxRigidBodyMessage();
               rigidBodyMessage.setEndEffectorHashCode(syncedRobot.getFullRobotModel().getForearm(side).hashCode());
               rigidBodyMessage.getDesiredPositionInWorld().set(actionForearmPoses.get(side).getTranslation());
               rigidBodyMessage.getLinearSelectionMatrix().setXSelected(false); // Disable position tracking for forearm
               rigidBodyMessage.getLinearSelectionMatrix().setYSelected(false);
               rigidBodyMessage.getLinearSelectionMatrix().setZSelected(false);
               rigidBodyMessage.getDesiredOrientationInWorld().set(actionForearmPoses.get(side).getRotation());
               rigidBodyMessage.getAngularWeightMatrix().setXWeight(0.01);
               rigidBodyMessage.getAngularWeightMatrix().setYWeight(0.01);
               rigidBodyMessage.getAngularWeightMatrix().setZWeight(0.001);
               ikInputMessage.getInputs().add().set(rigidBodyMessage);
            }
         }
      }
      kstInputPublisher.publish(ikInputMessage);
   }

   private void handUISyncBefore()
   {
      if (uiCommandSubscription.poll())
      {
         VLAOperationMessage uiCommand = uiCommandSubscription.read();
         latestTimestampModifiable.fromMessage(uiCommand.getLatestTimestampModifiable());
         boolean wasRunning = running.getValue();
         running.fromMessage(uiCommand.getRunning());
         if (!wasRunning && running.getValue())
         {
            ToolboxStateMessage toolboxStateMessage = new ToolboxStateMessage();
            toolboxStateMessage.setRequestedToolboxState(ToolboxState.WAKE_UP.toByte());
            kstStatePublisher.publish(toolboxStateMessage);
         }
         controlRobot.fromMessage(uiCommand.getControlRobot());
      }
   }

   private void handleUISyncAfter()
   {
      VLAOperationMessage uiStatus = new VLAOperationMessage();
      latestTimestampModifiable.toMessage(uiStatus.getLatestTimestampModifiable());
      uiStatus.setSequenceId((int) sequenceID++);
      uiStatus.setRunning(running.toMessage());
      uiStatus.setControlRobot(controlRobot.toMessage());
      for (RobotSide side : RobotSide.values)
      {
         uiStatus.getActionHandPoses()[side.ordinal()].set(actionHandPoses.get(side));
         uiStatus.getActionForearmPoses()[side.ordinal()].set(actionForearmPoses.get(side));
      }
      uiStatus.setStatusMessage("%-30s Actions: %d".formatted(status, numberOfActionsReceived.getValue()));
      uiStatusPublisher.publish(uiStatus);
   }

   public void startRepeating()
   {
      thread.startRepeating();
   }

   public void destroy()
   {
      thread.blockingKill();
   }

   public Notification getRequested()
   {
      return requested;
   }

   public SideDependentList<Mat> getImages()
   {
      return images;
   }
}
