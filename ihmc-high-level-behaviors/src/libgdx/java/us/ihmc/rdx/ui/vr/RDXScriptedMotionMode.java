package us.ihmc.rdx.ui.vr;

import controller_msgs.msg.dds.ArmTrajectoryMessage;
import controller_msgs.msg.dds.ChestTrajectoryMessage;
import controller_msgs.msg.dds.JointspaceTrajectoryStatusMessage;
import controller_msgs.msg.dds.TaskspaceTrajectoryStatusMessage;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImDouble;
import org.lwjgl.openvr.InputDigitalActionData;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.rdx.imgui.ImGuiFrequencyPlot;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.teleoperation.RDXScriptedTrajectoryStreamer;
import us.ihmc.rdx.ui.teleoperation.RDXScriptedTrajectoryStreamer.ScriptedTrajectoryType;
import us.ihmc.rdx.vr.RDXVRContext;
import us.ihmc.rdx.vr.RDXVRControllerModel;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.HashMap;
import java.util.Map;

public class RDXScriptedMotionMode
{
   private final ROS2SyncedRobotModel syncedRobot;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final DRCRobotModel robotModel;
   private FullHumanoidRobotModel fullRobotModel;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final FramePose3D tempFramePose = new FramePose3D();
   private final ImGuiFrequencyPlot statusFrequencyPlot = new ImGuiFrequencyPlot();
   private final ImGuiFrequencyPlot outputFrequencyPlot = new ImGuiFrequencyPlot();
   private final Map<String, MutableReferenceFrame> trackedSegmentDesiredFrame = new HashMap<>();
   private final ImBoolean streamToController = new ImBoolean(false);
   private final SceneGraph sceneGraph;
   private RDXScriptedTrajectoryStreamer trajectoryStreamer;
   private final SideDependentList<RigidBodyBasics> hands = new SideDependentList<>();
   private final double defaultScriptedTrajectoryDuration = 5.0;

   private ReferenceFrame chestFrame;
   private final RigidBodyTransform chestTransformToWorld = new RigidBodyTransform();
   private ScriptedTrajectoryType trajectoryType = ScriptedTrajectoryType.STRETCH_OUT_ARMS;
   private boolean isTrajectoryMessageSent = false;
   private boolean isTrajectoryCompleted = false;
   private int reachabilitySweepCounter = 0;
   private boolean isFirstFrame = true;
   ImDouble tempDuration = new ImDouble(4.0);

   private RDXVRControllerModel controllerModel = RDXVRControllerModel.UNKNOWN;

   private final ImDouble homeDuration = new ImDouble();
   private final ImDouble wristRomDuration = new ImDouble();
   private final ImDouble beachBallFlexDuration = new ImDouble();
   private final ImDouble beachBallOverheadDuration = new ImDouble();
   private final ImDouble dabOnThemHatersDuration = new ImDouble();
   private final ImDouble reachabilityArmsBackDuration = new ImDouble();
   private final ImDouble reachabilityArmsForwardDuration = new ImDouble();
   private final ImDouble reachabilityArmsSidewaysDuration = new ImDouble();
   private final ImDouble reachabilityArmsUpDuration = new ImDouble();
   private final ImDouble shoulderPitchRomDuration = new ImDouble();
   private final ImDouble shoulderRollRomDuration = new ImDouble();
   private final ImDouble shoulderYawRomDuration = new ImDouble();
   private final ImDouble elbowRomDuration = new ImDouble();
   private final ImDouble wristYawRomDuration = new ImDouble();
   private final ImDouble wristRollRomDuration = new ImDouble();
   private final ImDouble gripperYawRomDuration = new ImDouble();

   public RDXScriptedMotionMode(ROS2SyncedRobotModel syncedRobot, ROS2ControllerHelper ros2ControllerHelper, SceneGraph sceneGraph)
   {
      this.syncedRobot = syncedRobot;
      this.robotModel = syncedRobot.getRobotModel();
      this.ros2ControllerHelper = ros2ControllerHelper;
      this.sceneGraph = sceneGraph;
   }

   public void create(RDXVRContext vrContext)
   {
      fullRobotModel = syncedRobot.getFullRobotModel();

      // Get all the joints in each arm
      SideDependentList<OneDoFJointBasics[]> armJoints = new SideDependentList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics hand = fullRobotModel.getHand(robotSide);
         RigidBodyBasics chest = fullRobotModel.getChest();
         OneDoFJointBasics[] joints = MultiBodySystemTools.createOneDoFJointPath(chest, hand);
         hands.put(robotSide, hand);
         armJoints.put(robotSide, joints);
      }

      trajectoryStreamer = new RDXScriptedTrajectoryStreamer(armJoints, defaultScriptedTrajectoryDuration);

      //TODO: Add additional logic for joint space trajectories
      // Check for completion of the trajectories.
      ros2ControllerHelper.subscribeViaCallback(HumanoidControllerAPI.getOutputTopic(robotModel.getSimpleRobotName())
                                                                     .withTypeName(TaskspaceTrajectoryStatusMessage.class), message ->
                                                {
                                                   if (message.getEndEffectorName().toString().equals(fullRobotModel.getHand(RobotSide.LEFT).getName()))
                                                   {
                                                      if (message.getTrajectoryExecutionStatus()
                                                          == TaskspaceTrajectoryStatusMessage.TRAJECTORY_EXECUTION_STATUS_COMPLETED)
                                                      {
                                                         // Reset this boolean for the next messages
                                                         isTrajectoryMessageSent = false;
                                                         isTrajectoryCompleted = true;
                                                      }
                                                   }
                                                });

      ros2ControllerHelper.subscribeViaCallback(HumanoidControllerAPI.getOutputTopic(robotModel.getSimpleRobotName())
                                                                     .withTypeName(JointspaceTrajectoryStatusMessage.class), message ->
                                                {
                                                   if (message.getJointNames()
                                                              .getString(0)
                                                              .equals(fullRobotModel.getArmJoint(RobotSide.LEFT, ArmJointName.SHOULDER_PITCH).getName())
                                                       || message.getJointNames()
                                                                 .getString(0)
                                                                 .equals(fullRobotModel.getArmJoint(RobotSide.RIGHT, ArmJointName.SHOULDER_PITCH).getName()))
                                                   {
                                                      if (message.getTrajectoryExecutionStatus()
                                                          == JointspaceTrajectoryStatusMessage.TRAJECTORY_EXECUTION_STATUS_COMPLETED)
                                                      {
                                                         // Reset this boolean for the next messages
                                                         isTrajectoryMessageSent = false;
                                                         isTrajectoryCompleted = true;
                                                      }
                                                   }
                                                });
   }

   public void processVRInput(RDXVRContext vrContext)
   {
      if (controllerModel == RDXVRControllerModel.UNKNOWN)
         controllerModel = vrContext.getControllerModel();
      vrContext.getController(RobotSide.LEFT).runIfConnected(controller ->
                                                             {
                                                                InputDigitalActionData aButton = controller.getAButtonActionData();
                                                                if (aButton.bChanged() && !aButton.bState())
                                                                {
                                                                   streamToController.set(!streamToController.get());
                                                                }

                                                                // NOTE: Implement hand open close for controller trigger button.
                                                                InputDigitalActionData clickTriggerButton = controller.getClickTriggerActionData();
                                                                if (clickTriggerButton.bChanged() && !clickTriggerButton.bState())
                                                                {
                                                                   //TODO: different trajectory here
                                                                }

                                                                // Check if left joystick is pressed in order to trigger recording or replay of motion
                                                                InputDigitalActionData joystickButton = controller.getJoystickPressActionData();
                                                             });

      vrContext.getController(RobotSide.RIGHT).runIfConnected(controller ->
                                                              {
                                                                 InputDigitalActionData aButton = controller.getAButtonActionData();
                                                                 if (aButton.bChanged() && !aButton.bState())
                                                                 {
                                                                    //TODO: different trajectory here
                                                                 }

                                                                 // NOTE: Implement hand open close for controller trigger button.
                                                                 InputDigitalActionData clickTriggerButton = controller.getClickTriggerActionData();
                                                                 if (clickTriggerButton.bChanged() && !clickTriggerButton.bState())
                                                                 { // do not want to close grippers while interacting with the panel
                                                                    //TODO: different trajectory here
                                                                 }
                                                              });
   }

   private void sendScriptedTrajectories()
   {
      // Send messages to the hands or arms based on the scripted trajectories
      for (RobotSide robotSide : RobotSide.values)
      {
         ArmTrajectoryMessage armTrajectoryMessage;
         switch (trajectoryType)
         {
            //TODO: (CD) Don't use this until the taskspace gains are tuned.
            //            case STRETCH_OUT_ARMS:
            //               HandTrajectoryMessage handTrajectoryMessage = new HandTrajectoryMessage();
            //               scriptedTrajectory.packHandTrajectoryMessage(handTrajectoryMessage,
            //                                                            robotSide,
            //                                                            trajectoryType,
            //                                                            scriptedTrajectoryDuration);
            //               ros2ControllerHelper.publishToController(handTrajectoryMessage);
            //               break;
            case HOME_CONFIGURATION:
            case ROM_SHOULDER_PITCH:
            case ROM_SHOULDER_ROLL:
            case ROM_SHOULDER_YAW:
            case ROM_ELBOW:
            case ROM_WRIST_YAW:
            case ROM_WRIST_ROLL:
            case ROM_GRIPPER_YAW:
            case WRIST_RANGE_OF_MOTION:
            case BEACH_BALL_FLEX:
            case BEACH_BALL_OVERHEAD:
            case DAB_ON_THEM_HATERS:
            case REACHABILITY_ARMS_UP:
            case REACHABILITY_ARMS_FORWARD:
            case REACHABILITY_ARMS_SIDEWAYS:
            case REACHABILITY_ARMS_BACK:
               armTrajectoryMessage = trajectoryStreamer.generateArmTrajectoryMessage(trajectoryType, robotSide);
               ros2ControllerHelper.publishToController(armTrajectoryMessage);
               break;
            case REACHABILITY_SWEEP:
               if (reachabilitySweepCounter == 0)
               {
                  armTrajectoryMessage = trajectoryStreamer.generateArmTrajectoryMessage(ScriptedTrajectoryType.REACHABILITY_ARMS_BACK, robotSide);
                  ros2ControllerHelper.publishToController(armTrajectoryMessage);
                  if (robotSide == RobotSide.RIGHT)
                     reachabilitySweepCounter++;
               }
               else if (reachabilitySweepCounter == 1)
               {
                  armTrajectoryMessage = trajectoryStreamer.generateArmTrajectoryMessage(ScriptedTrajectoryType.REACHABILITY_ARMS_SIDEWAYS, robotSide);
                  ros2ControllerHelper.publishToController(armTrajectoryMessage);
                  if (robotSide == RobotSide.RIGHT)
                     reachabilitySweepCounter++;
               }
               else if (reachabilitySweepCounter == 2)
               {
                  armTrajectoryMessage = trajectoryStreamer.generateArmTrajectoryMessage(ScriptedTrajectoryType.REACHABILITY_ARMS_FORWARD, robotSide);
                  ros2ControllerHelper.publishToController(armTrajectoryMessage);
                  if (robotSide == RobotSide.RIGHT)
                     reachabilitySweepCounter++;
               }
               else if (reachabilitySweepCounter == 3)
               {
                  armTrajectoryMessage = trajectoryStreamer.generateArmTrajectoryMessage(ScriptedTrajectoryType.REACHABILITY_ARMS_UP, robotSide);
                  ros2ControllerHelper.publishToController(armTrajectoryMessage);
                  if (robotSide == RobotSide.RIGHT)
                     reachabilitySweepCounter++;
               }
               else if (reachabilitySweepCounter == 4)
               {
                  armTrajectoryMessage = trajectoryStreamer.generateArmTrajectoryMessage(ScriptedTrajectoryType.HOME_CONFIGURATION, robotSide);
                  ros2ControllerHelper.publishToController(armTrajectoryMessage);
                  if (robotSide == RobotSide.RIGHT)
                     reachabilitySweepCounter++;
               }
               else if (reachabilitySweepCounter == 5)
               {
                  armTrajectoryMessage = trajectoryStreamer.generateArmTrajectoryMessage(ScriptedTrajectoryType.WRIST_RANGE_OF_MOTION, robotSide);
                  ros2ControllerHelper.publishToController(armTrajectoryMessage);
                  if (robotSide == RobotSide.RIGHT)
                     reachabilitySweepCounter++;
               }
               else
               {
                  armTrajectoryMessage = trajectoryStreamer.generateArmTrajectoryMessage(ScriptedTrajectoryType.HOME_CONFIGURATION, robotSide);
                  ros2ControllerHelper.publishToController(armTrajectoryMessage);
                  if (robotSide == RobotSide.RIGHT)
                     reachabilitySweepCounter = 0;
               }
               break;

            default:
               throw new RuntimeException("Unhandled trajectory type: " + trajectoryType);
         }
      }

      if (chestFrame == null)
      {
         chestTransformToWorld.set(syncedRobot.getFullRobotModel().getChest().getBodyFixedFrame().getTransformToWorldFrame());
         chestFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(ReferenceFrame.getWorldFrame(), chestTransformToWorld);
      }

      // Send a message to bias the chest to the vertical
      tempFramePose.setToZero(chestFrame);
      tempFramePose.changeFrame(ReferenceFrame.getWorldFrame());
      ChestTrajectoryMessage chestMessage = HumanoidMessageTools.createChestTrajectoryMessage(0.0,
                                                                                              tempFramePose.getOrientation(),
                                                                                              ReferenceFrame.getWorldFrame());
      chestMessage.setSequenceId(0);
      chestMessage.getSo3Trajectory().getWeightMatrix().set(MessageTools.createWeightMatrix3DMessage(10.0));
      ros2ControllerHelper.publishToController(chestMessage);

      outputFrequencyPlot.recordEvent();
   }

   public void update(boolean scriptedMotionEnable)
   {
      // Safety features!
      if (!scriptedMotionEnable)
         streamToController.set(false);
      else
      {
         statusFrequencyPlot.recordEvent();

         if (trajectoryType == ScriptedTrajectoryType.REACHABILITY_SWEEP && !isTrajectoryMessageSent && streamToController.get())
         {
            sendScriptedTrajectories();
            // If the reachability counter makes it back to zero, then the chain of trajectories is completed.
            if (reachabilitySweepCounter == 0)
            {
               //reset
               streamToController.set(false);
            }
            else
            {
               streamToController.set(true);
               isTrajectoryMessageSent = true;
            }
         }
         else if (!isTrajectoryMessageSent && streamToController.get())
         {
            if (isTrajectoryCompleted)
            {
               // reset
               streamToController.set(false);
            }
            else
            {
               sendScriptedTrajectories();
               isTrajectoryMessageSent = true;
            }
         }
         isTrajectoryCompleted = false;
      }
   }

   public void renderImGuiWidgets()
   {
      //TODO: (CD) Don't use this until the taskspace gains are tuned.
      //      ImGui.text("Taskspace Scripted Trajectories:");
      //      if (ImGui.radioButton(labels.get("Stretch out arms"), trajectoryType == ScriptedTrajectoryType.STRETCH_OUT_ARMS))
      //      {
      //         trajectoryType = ScriptedTrajectoryType.STRETCH_OUT_ARMS;
      //      }

      // Ensure the collapsing headers are open on the first frame.
      if (isFirstFrame)
         ImGui.setNextItemOpen(true);

      if (ImGui.collapsingHeader(labels.get("Jointspace Scripted Trajectories:")))
      {
         // Home Configuration
         if (ImGui.radioButton(labels.get("Home Configuration"), trajectoryType == ScriptedTrajectoryType.HOME_CONFIGURATION))
         {
            trajectoryType = ScriptedTrajectoryType.HOME_CONFIGURATION;
         }
         ImGui.sameLine();
         ImGui.pushItemWidth(70.0f);
         homeDuration.set(trajectoryStreamer.getTrajectoryDuration(ScriptedTrajectoryType.HOME_CONFIGURATION));
         if (ImGui.inputDouble("##homeConfigurationDuration", homeDuration))
         {
            trajectoryStreamer.setTrajectoryDuration(ScriptedTrajectoryType.HOME_CONFIGURATION, homeDuration.get());
         }

         // Wrist ROM
         if (ImGui.radioButton(labels.get("Wrist ROM"), trajectoryType == ScriptedTrajectoryType.WRIST_RANGE_OF_MOTION))
         {
            trajectoryType = ScriptedTrajectoryType.WRIST_RANGE_OF_MOTION;
         }
         ImGui.sameLine();
         wristRomDuration.set(trajectoryStreamer.getTrajectoryDuration(ScriptedTrajectoryType.WRIST_RANGE_OF_MOTION));
         if (ImGui.inputDouble("##wristRomDuration", wristRomDuration))
         {
            trajectoryStreamer.setTrajectoryDuration(ScriptedTrajectoryType.WRIST_RANGE_OF_MOTION, wristRomDuration.get());
         }

         // Beach Ball Flex
         if (ImGui.radioButton(labels.get("Beach Ball Flex"), trajectoryType == ScriptedTrajectoryType.BEACH_BALL_FLEX))
         {
            trajectoryType = ScriptedTrajectoryType.BEACH_BALL_FLEX;
         }
         ImGui.sameLine();
         beachBallFlexDuration.set(trajectoryStreamer.getTrajectoryDuration(ScriptedTrajectoryType.BEACH_BALL_FLEX));
         if (ImGui.inputDouble("##beachBallFlexDuration", beachBallFlexDuration))
         {
            trajectoryStreamer.setTrajectoryDuration(ScriptedTrajectoryType.BEACH_BALL_FLEX, beachBallFlexDuration.get());
         }

         // Beach Ball Overhead
         if (ImGui.radioButton(labels.get("Beach Ball Overhead"), trajectoryType == ScriptedTrajectoryType.BEACH_BALL_OVERHEAD))
         {
            trajectoryType = ScriptedTrajectoryType.BEACH_BALL_OVERHEAD;
         }
         ImGui.sameLine();
         beachBallOverheadDuration.set(trajectoryStreamer.getTrajectoryDuration(ScriptedTrajectoryType.BEACH_BALL_OVERHEAD));
         if (ImGui.inputDouble("##beachBallOverheadDuration", beachBallOverheadDuration))
         {
            trajectoryStreamer.setTrajectoryDuration(ScriptedTrajectoryType.BEACH_BALL_OVERHEAD, beachBallOverheadDuration.get());
         }

         // Dab on them haters
         if (ImGui.radioButton(labels.get("Dab on them haters"), trajectoryType == ScriptedTrajectoryType.DAB_ON_THEM_HATERS))
         {
            trajectoryType = ScriptedTrajectoryType.DAB_ON_THEM_HATERS;
         }
         ImGui.sameLine();
         dabOnThemHatersDuration.set(trajectoryStreamer.getTrajectoryDuration(ScriptedTrajectoryType.DAB_ON_THEM_HATERS));
         if (ImGui.inputDouble("##dabOnThemHatersDuration", dabOnThemHatersDuration))
         {
            trajectoryStreamer.setTrajectoryDuration(ScriptedTrajectoryType.DAB_ON_THEM_HATERS, dabOnThemHatersDuration.get());
         }

         ImGui.popItemWidth(); // Reset item width if necessary
      }

      if (isFirstFrame)
         ImGui.setNextItemOpen(true);

      if (ImGui.collapsingHeader(labels.get("Reachability Trajectories:")))
      {
         // Reachability Sweep
         if (ImGui.radioButton(labels.get("Reachability Sweep"), trajectoryType == ScriptedTrajectoryType.REACHABILITY_SWEEP))
         {
            trajectoryType = ScriptedTrajectoryType.REACHABILITY_SWEEP;
         }

         // Reachability Arms Back
         if (ImGui.radioButton(labels.get("Reachability Arms Back"), trajectoryType == ScriptedTrajectoryType.REACHABILITY_ARMS_BACK))
         {
            trajectoryType = ScriptedTrajectoryType.REACHABILITY_ARMS_BACK;
         }
         ImGui.sameLine();
         reachabilityArmsBackDuration.set(trajectoryStreamer.getTrajectoryDuration(ScriptedTrajectoryType.REACHABILITY_ARMS_BACK));
         if (ImGui.inputDouble("##reachabilityArmsBackDuration", reachabilityArmsBackDuration))
         {
            trajectoryStreamer.setTrajectoryDuration(ScriptedTrajectoryType.REACHABILITY_ARMS_BACK, reachabilityArmsBackDuration.get());
         }

         // Reachability Arms Forward
         if (ImGui.radioButton(labels.get("Reachability Arms Forward"), trajectoryType == ScriptedTrajectoryType.REACHABILITY_ARMS_FORWARD))
         {
            trajectoryType = ScriptedTrajectoryType.REACHABILITY_ARMS_FORWARD;
         }
         ImGui.sameLine();
         reachabilityArmsForwardDuration.set(trajectoryStreamer.getTrajectoryDuration(ScriptedTrajectoryType.REACHABILITY_ARMS_FORWARD));
         if (ImGui.inputDouble("##reachabilityArmsForwardDuration", reachabilityArmsForwardDuration))
         {
            trajectoryStreamer.setTrajectoryDuration(ScriptedTrajectoryType.REACHABILITY_ARMS_FORWARD, reachabilityArmsForwardDuration.get());
         }

         // Reachability Arms Sideways
         if (ImGui.radioButton(labels.get("Reachability Arms Sideways"), trajectoryType == ScriptedTrajectoryType.REACHABILITY_ARMS_SIDEWAYS))
         {
            trajectoryType = ScriptedTrajectoryType.REACHABILITY_ARMS_SIDEWAYS;
         }
         ImGui.sameLine();
         reachabilityArmsSidewaysDuration.set(trajectoryStreamer.getTrajectoryDuration(ScriptedTrajectoryType.REACHABILITY_ARMS_SIDEWAYS));
         if (ImGui.inputDouble("##reachabilityArmsSidewaysDuration", reachabilityArmsSidewaysDuration))
         {
            trajectoryStreamer.setTrajectoryDuration(ScriptedTrajectoryType.REACHABILITY_ARMS_SIDEWAYS, reachabilityArmsSidewaysDuration.get());
         }

         // Reachability Arms Up
         if (ImGui.radioButton(labels.get("Reachability Arms Up"), trajectoryType == ScriptedTrajectoryType.REACHABILITY_ARMS_UP))
         {
            trajectoryType = ScriptedTrajectoryType.REACHABILITY_ARMS_UP;
         }
         ImGui.sameLine();
         reachabilityArmsUpDuration.set(trajectoryStreamer.getTrajectoryDuration(ScriptedTrajectoryType.REACHABILITY_ARMS_UP));
         if (ImGui.inputDouble("##reachabilityArmsUpDuration", reachabilityArmsUpDuration))
         {
            trajectoryStreamer.setTrajectoryDuration(ScriptedTrajectoryType.REACHABILITY_ARMS_UP, reachabilityArmsUpDuration.get());
         }
      }

      if (isFirstFrame)
         ImGui.setNextItemOpen(true);

      if (ImGui.collapsingHeader(labels.get("Joint ROM Scripted Trajectories:")))
      {
         // Shoulder Pitch ROM
         if (ImGui.radioButton(labels.get("Shoulder Pitch ROM"), trajectoryType == ScriptedTrajectoryType.ROM_SHOULDER_PITCH))
         {
            trajectoryType = ScriptedTrajectoryType.ROM_SHOULDER_PITCH;
         }
         ImGui.sameLine();
         shoulderPitchRomDuration.set(trajectoryStreamer.getTrajectoryDuration(ScriptedTrajectoryType.ROM_SHOULDER_PITCH));
         if (ImGui.inputDouble("##shoulderPitchRomDuration", shoulderPitchRomDuration))
         {
            trajectoryStreamer.setTrajectoryDuration(ScriptedTrajectoryType.ROM_SHOULDER_PITCH, shoulderPitchRomDuration.get());
         }

         // Shoulder Roll ROM
         if (ImGui.radioButton(labels.get("Shoulder Roll ROM"), trajectoryType == ScriptedTrajectoryType.ROM_SHOULDER_ROLL))
         {
            trajectoryType = ScriptedTrajectoryType.ROM_SHOULDER_ROLL;
         }
         ImGui.sameLine();
         shoulderRollRomDuration.set(trajectoryStreamer.getTrajectoryDuration(ScriptedTrajectoryType.ROM_SHOULDER_ROLL));
         if (ImGui.inputDouble("##shoulderRollRomDuration", shoulderRollRomDuration))
         {
            trajectoryStreamer.setTrajectoryDuration(ScriptedTrajectoryType.ROM_SHOULDER_ROLL, shoulderRollRomDuration.get());
         }

         // Shoulder Yaw ROM
         if (ImGui.radioButton(labels.get("Shoulder Yaw ROM"), trajectoryType == ScriptedTrajectoryType.ROM_SHOULDER_YAW))
         {
            trajectoryType = ScriptedTrajectoryType.ROM_SHOULDER_YAW;
         }
         ImGui.sameLine();
         shoulderYawRomDuration.set(trajectoryStreamer.getTrajectoryDuration(ScriptedTrajectoryType.ROM_SHOULDER_YAW));
         if (ImGui.inputDouble("##shoulderYawRomDuration", shoulderYawRomDuration))
         {
            trajectoryStreamer.setTrajectoryDuration(ScriptedTrajectoryType.ROM_SHOULDER_YAW, shoulderYawRomDuration.get());
         }

         // Elbow ROM
         if (ImGui.radioButton(labels.get("Elbow ROM"), trajectoryType == ScriptedTrajectoryType.ROM_ELBOW))
         {
            trajectoryType = ScriptedTrajectoryType.ROM_ELBOW;
         }
         ImGui.sameLine();
         elbowRomDuration.set(trajectoryStreamer.getTrajectoryDuration(ScriptedTrajectoryType.ROM_ELBOW));
         if (ImGui.inputDouble("##elbowRomDuration", elbowRomDuration))
         {
            trajectoryStreamer.setTrajectoryDuration(ScriptedTrajectoryType.ROM_ELBOW, elbowRomDuration.get());
         }

         // Wrist Yaw ROM
         if (ImGui.radioButton(labels.get("Wrist Yaw ROM"), trajectoryType == ScriptedTrajectoryType.ROM_WRIST_YAW))
         {
            trajectoryType = ScriptedTrajectoryType.ROM_WRIST_YAW;
         }
         ImGui.sameLine();
         wristYawRomDuration.set(trajectoryStreamer.getTrajectoryDuration(ScriptedTrajectoryType.ROM_WRIST_YAW));
         if (ImGui.inputDouble("##wristYawRomDuration", wristYawRomDuration))
         {
            trajectoryStreamer.setTrajectoryDuration(ScriptedTrajectoryType.ROM_WRIST_YAW, wristYawRomDuration.get());
         }

         // Wrist Roll ROM
         if (ImGui.radioButton(labels.get("Wrist Roll ROM"), trajectoryType == ScriptedTrajectoryType.ROM_WRIST_ROLL))
         {
            trajectoryType = ScriptedTrajectoryType.ROM_WRIST_ROLL;
         }
         ImGui.sameLine();
         wristRollRomDuration.set(trajectoryStreamer.getTrajectoryDuration(ScriptedTrajectoryType.ROM_WRIST_ROLL));
         if (ImGui.inputDouble("##wristRollRomDuration", wristRollRomDuration))
         {
            trajectoryStreamer.setTrajectoryDuration(ScriptedTrajectoryType.ROM_WRIST_ROLL, wristRollRomDuration.get());
         }

         // Gripper Yaw ROM
         if (ImGui.radioButton(labels.get("Gripper Yaw ROM"), trajectoryType == ScriptedTrajectoryType.ROM_GRIPPER_YAW))
         {
            trajectoryType = ScriptedTrajectoryType.ROM_GRIPPER_YAW;
         }
         ImGui.sameLine();
         gripperYawRomDuration.set(trajectoryStreamer.getTrajectoryDuration(ScriptedTrajectoryType.ROM_GRIPPER_YAW));
         if (ImGui.inputDouble("##gripperYawRomDuration", gripperYawRomDuration))
         {
            trajectoryStreamer.setTrajectoryDuration(ScriptedTrajectoryType.ROM_GRIPPER_YAW, gripperYawRomDuration.get());
         }

         isFirstFrame = false;
      }

      if (ImGui.button(labels.get("Execute Trajectory")))
      {
         streamToController.set(true);
      }
   }

   public boolean isScriptedMotionExecuting()
   {
      return streamToController.get();
   }
}