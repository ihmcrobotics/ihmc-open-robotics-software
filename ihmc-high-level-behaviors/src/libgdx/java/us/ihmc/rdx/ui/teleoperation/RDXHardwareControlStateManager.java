package us.ihmc.rdx.ui.teleoperation;

import controller_msgs.EStopMasterGainCommandMessage;
import controller_msgs.EStopMasterGainStatusMessage;
import controller_msgs.GoHomeMessage;
import controller_msgs.HighLevelStateChangeStatusMessage;
import controller_msgs.HighLevelStateMessage;
import controller_msgs.RLModelSelectionMessage;
import controller_msgs.RLPolicyState;
import controller_msgs.StopAllTrajectoryMessage;
import imgui.ImGui;
import imgui.flag.ImGuiCol;
import imgui.type.ImBoolean;
import imgui.type.ImDouble;
import imgui.type.ImInt;
import us.ihmc.avatar.arm.PresetArmConfiguration;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.affordances.RDXArmManager;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.Timer;

public class RDXHardwareControlStateManager
{
   protected final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   protected final ROS2ControllerHelper controllerHelper;
   protected final Timer estopMasterGainStatusTimer = new Timer();
   protected final ImBoolean estop = new ImBoolean();
   protected final ImDouble desiredMasterGain = new ImDouble();
   protected final EStopMasterGainCommandMessage hardwareCommandMessage = new EStopMasterGainCommandMessage();
   protected final EStopMasterGainStatusMessage hardwareStatusMessage = new EStopMasterGainStatusMessage();
   protected final ImInt desiredRLModel = new ImInt();
   protected String[] rlModelNames = new String[0];
   protected HighLevelControllerName currentHighLevelState = null;
   /** Last model reported by the controller via {@link RLPolicyState}, not the local combo selection. */
   protected String controllerRLModelName = null;

   public RDXHardwareControlStateManager(ROS2ControllerHelper controllerHelper)
   {
      this.controllerHelper = controllerHelper;

      controllerHelper.subscribeToControllerViaVolatileCallback(EStopMasterGainStatusMessage.class, message ->
      {
         hardwareStatusMessage.set(message);
         consumeHardwareStatusMessage(hardwareStatusMessage);

         estopMasterGainStatusTimer.reset();
      });
      controllerHelper.subscribeToControllerViaVolatileCallback(HighLevelStateChangeStatusMessage.class, message ->
      {  // TODO: Create a HighLevelStateStatusMessage that is periodically published, so we can always know current state
         currentHighLevelState = HighLevelControllerName.fromByte(message.getEndHighLevelControllerName());
      });
      controllerHelper.subscribeToControllerViaVolatileCallback(RLPolicyState.class, this::consumeRLPolicyStateMessage);
      controllerHelper.subscribeViaVolatileCallback(robotName -> HumanoidControllerAPI.getLowFrequencyTopic(RLPolicyState.class, robotName),
                                                    this::consumeRLPolicyStateMessage);
   }

   protected void consumeRLPolicyStateMessage(RLPolicyState rlPolicyState)
   {
      int availableModelCount = Math.min(Byte.toUnsignedInt(rlPolicyState.getNumAvailableModels()), rlPolicyState.getAvailableModels().size());
      if (availableModelCount <= 0)
      {
         rlModelNames = new String[0];
         desiredRLModel.set(0);
         controllerRLModelName = null;
         return;
      }

      String[] modelNames = new String[availableModelCount];
      for (int i = 0; i < availableModelCount; i++)
      {
         String modelName = rlPolicyState.getAvailableModels().get(i).toString();
         modelNames[i] = (modelName == null || modelName.isEmpty()) ? "Model %d".formatted(i) : modelName;
      }
      rlModelNames = modelNames;

      // Always mirror the controller's active model so UI stays correct after controller-side switches
      // (e.g. standup → walking handoff), not only on first receive / catalog changes.
      int currentModel = Byte.toUnsignedInt(rlPolicyState.getCurrentModel());
      if (currentModel >= 0 && currentModel < rlModelNames.length)
      {
         desiredRLModel.set(currentModel);
         controllerRLModelName = rlModelNames[currentModel];
      }
      else if (desiredRLModel.get() >= rlModelNames.length)
         desiredRLModel.set(rlModelNames.length - 1);
   }

   protected void consumeHardwareStatusMessage(EStopMasterGainStatusMessage hardwareStatusMessage)
   {
      estop.set(hardwareStatusMessage.getIsEstopped());
      desiredMasterGain.set(hardwareStatusMessage.getCurrentMasterGain());
   }

   public void renderImGuiWidgets(DRCRobotModel robotModel, HumanoidReferenceFrames referenceFrames, RDXArmManager armManager, double maxPelvisHeight)
   {
      if (estopMasterGainStatusTimer.isRunning(1.0))
      {
         ImGui.pushStyleColor(ImGuiCol.CheckMark, ImGuiTools.DARK_RED);
         ImGui.pushStyleColor(ImGuiCol.Text, ImGuiTools.DARK_RED);
         boolean changed = ImGui.checkbox(labels.get("SOFT-E-STOP"), estop);
         ImGui.sameLine();
         ImGui.setNextItemWidth(ImGui.getColumnWidth());
         changed |= ImGuiTools.sliderDouble(labels.getHidden("Master Gain"), desiredMasterGain, 0.0, 1.0, "Master Gain: %.2f");
         boolean servoRobot = ImGui.button(labels.get("Servo Robot"));
         changed |= servoRobot;
         ImGui.sameLine();
         boolean unservoSlowly = ImGui.button(labels.get("Unservo Slowly"));
         changed |= unservoSlowly;
         ImGui.popStyleColor(2);

         if (changed)
         {
            hardwareCommandMessage.setEstop(estop.get());
            hardwareCommandMessage.setDesiredMasterGain(desiredMasterGain.get());
            hardwareCommandMessage.setServoRobot(servoRobot);
            hardwareCommandMessage.setUnservoQuickly(unservoSlowly);
            controllerHelper.publishToController(hardwareCommandMessage);
         }
      }

      float widgetStartX = 98.0f;
      ImGui.text("Current Controller State: %s".formatted(currentHighLevelState == null ? "Unknown" : currentHighLevelState.name()));

      ImGui.text("Request:");
      ImGui.sameLine();
      ImGui.setCursorPosX(widgetStartX);
      if (ImGui.button(labels.get("Calibration")))
      {
         HighLevelStateMessage highLevelStateMessage = new HighLevelStateMessage();
         highLevelStateMessage.setHighLevelControllerName(HighLevelControllerName.CALIBRATION.toByte());
         controllerHelper.publishToController(highLevelStateMessage);
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get("Freeze")))
      {
         RDXBaseUI.pushNotification("Commanding freeze...");
         sendFreezeRequest();
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get("Stand Prep")))
      {
         RDXBaseUI.pushNotification("Commanding stand prep...");
         sendStandPrepRequest();
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get("Ground Prep")))
      {
         RDXBaseUI.pushNotification("Commanding ground prep...");
         sendGroundPrepRequest();
      }
      ImGui.setCursorPosX(widgetStartX);
      if (ImGui.button(labels.get("Stand Prep Transition")))
      {
         RDXBaseUI.pushNotification("Commanding stand prep transition...");
         sendStandPrepTransitionRequest();
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get("Walking")))
      {
         RDXBaseUI.pushNotification("Commanding Walking Control...");
         sendWalkingRequest();
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get("Exit Walking")))
      {
         RDXBaseUI.pushNotification("Commanding EXIT_WALKING...");
         sendExitWalkingRequest();
      }
      ImGui.setCursorPosX(widgetStartX);
      if (ImGui.button(labels.get("RL Control")))
      {
         RDXBaseUI.pushNotification("Commanding RL Control...");
         sendRLRequest();
      }
      ImGui.text("RL Model:");
      ImGui.sameLine();
      ImGui.setCursorPosX(widgetStartX);
      ImGui.setNextItemWidth(150.0f);
      if (rlModelNames.length > 0)
      {
         if (ImGui.combo(labels.get("RL Model"), desiredRLModel, rlModelNames))
         {
            sendRLModelSelectionRequest();
         }
      }
      else
      {
         ImGui.textDisabled("Waiting for RL models...");
      }

      ImGui.separator();
      ImGui.text("Command:");
      ImGui.sameLine();
      ImGui.setCursorPosX(widgetStartX);
      if (ImGui.button(labels.get("Home Pose")))
      {
         double trajectoryTime = 3.0;

         GoHomeMessage homeLeftArm = new GoHomeMessage();
         homeLeftArm.setHumanoidBodyPart(GoHomeMessage.HUMANOID_BODY_PART_ARM);
         homeLeftArm.setRobotSide(GoHomeMessage.ROBOT_SIDE_LEFT);
         homeLeftArm.setTrajectoryTime(trajectoryTime);
         controllerHelper.publishToController(homeLeftArm);

         GoHomeMessage homeRightArm = new GoHomeMessage();
         homeRightArm.setHumanoidBodyPart(GoHomeMessage.HUMANOID_BODY_PART_ARM);
         homeRightArm.setRobotSide(GoHomeMessage.ROBOT_SIDE_RIGHT);
         homeRightArm.setTrajectoryTime(trajectoryTime);
         controllerHelper.publishToController(homeRightArm);

         GoHomeMessage homePelvis = new GoHomeMessage();
         homePelvis.setHumanoidBodyPart(GoHomeMessage.HUMANOID_BODY_PART_PELVIS);
         homePelvis.setTrajectoryTime(trajectoryTime);
         controllerHelper.publishToController(homePelvis);

         GoHomeMessage homeChest = new GoHomeMessage();
         homeChest.setHumanoidBodyPart(GoHomeMessage.HUMANOID_BODY_PART_CHEST);
         homeChest.setTrajectoryTime(trajectoryTime);

         RDXBaseUI.pushNotification("Commanding home pose...");
         controllerHelper.publishToController(homeChest);
      }
      ImGui.sameLine();
      if (ImGui.button(labels.get("N-Pose")))
      {
         goNPose(robotModel, referenceFrames, armManager, maxPelvisHeight);
      }

      ImGui.sameLine();
      if (ImGui.button(labels.get("Stop All Trajectories")))
      {
         RDXBaseUI.pushNotification("Commanding stop all trajectories...");
         StopAllTrajectoryMessage stopAllTrajectoryMessage = new StopAllTrajectoryMessage();
         controllerHelper.publishToController(stopAllTrajectoryMessage);
      }
   }

   protected void goNPose(DRCRobotModel robotModel, HumanoidReferenceFrames referenceFrames, RDXArmManager armManager, double maxPelvisHeight)
   {
      double trajectoryTime = 3.0;

      for (RobotSide side : RobotSide.values)
         armManager.executeArmAngles(side, PresetArmConfiguration.N_POSE, trajectoryTime);

      if (robotModel.getRobotVersion().hasHead())
      {
         NeckJointName[] neckJointNamesArray = robotModel.getJointMap().getNeckJointNames();
         double[] desiredNeckJointValues = new double[neckJointNamesArray.length];
         for (int i = 0; i < neckJointNamesArray.length; i++)
         {
            desiredNeckJointValues[i] = 0.0; // TODO make 0 robot agnostic
         }
         controllerHelper.publishToController(HumanoidMessageTools.createHeadJointspaceTaskspaceTrajectoryMessage(referenceFrames,
                                                                                                            neckJointNamesArray,
                                                                                                            desiredNeckJointValues,
                                                                                                            trajectoryTime));
      }

      FramePose3D pelvisPose = new FramePose3D(referenceFrames.getMidFeetZUpFrame());
      pelvisPose.getTranslation().addZ(maxPelvisHeight - 0.02);
      pelvisPose.changeFrame(ReferenceFrame.getWorldFrame());
      controllerHelper.publishToController(HumanoidMessageTools.createPelvisTrajectoryMessage(trajectoryTime, pelvisPose));

      GoHomeMessage homeChest = new GoHomeMessage();
      homeChest.setHumanoidBodyPart(GoHomeMessage.HUMANOID_BODY_PART_CHEST);
      homeChest.setTrajectoryTime(trajectoryTime);
      controllerHelper.publishToController(homeChest);
      RDXBaseUI.pushNotification("Commanding N-pose...");
   }

   public void sendRLModelSelectionRequest()
   {
      int selectedModelIndex = desiredRLModel.get();
      if (rlModelNames.length > 0)
         selectedModelIndex = Math.max(0, Math.min(selectedModelIndex, rlModelNames.length - 1));

      RLModelSelectionMessage modelSelectionMessage = new RLModelSelectionMessage();
      modelSelectionMessage.setDesiredModel((byte) selectedModelIndex);
      modelSelectionMessage.setExecuteDesiredModel(true);
      controllerHelper.publishToController(modelSelectionMessage);
   }

   public HighLevelControllerName getCurrentHighLevelState()
   {
      return currentHighLevelState;
   }

   public String getCurrentRLModelName()
   {
      return controllerRLModelName;
   }

   public String getDesiredRLModelName()
   {
      int index = desiredRLModel.get();
      if (rlModelNames.length == 0 || index < 0 || index >= rlModelNames.length)
         return null;
      return rlModelNames[index];
   }

   public boolean selectRLModelByName(String modelName)
   {
      if (modelName == null)
         return false;

      for (int i = 0; i < rlModelNames.length; i++)
      {
         if (modelName.equals(rlModelNames[i]))
         {
            desiredRLModel.set(i);
            sendRLModelSelectionRequest();
            return true;
         }
      }
      LogTools.warn("Requested RL model is not in the controller catalog: {}", modelName);
      return false;
   }

   public void sendRLTransitionRequest()
   {
      HighLevelStateMessage highLevelStateMessage = new HighLevelStateMessage();
      highLevelStateMessage.setHighLevelControllerName(HighLevelControllerName.RL_TRANSITION_STATE.toByte());
      controllerHelper.publishToController(highLevelStateMessage);
   }

   public void sendExitRLRequest()
   {
      HighLevelStateMessage highLevelStateMessage = new HighLevelStateMessage();
      highLevelStateMessage.setHighLevelControllerName(HighLevelControllerName.EXIT_RL.toByte());
      controllerHelper.publishToController(highLevelStateMessage);
   }

   public void sendStandPrepRequest()
   {
      HighLevelStateMessage highLevelStateMessage = new HighLevelStateMessage();
      highLevelStateMessage.setHighLevelControllerName(HighLevelControllerName.STAND_PREP_STATE.toByte());
      controllerHelper.publishToController(highLevelStateMessage);
   }

   public void sendGroundPrepRequest()
   {
      sendGroundPrepRequest(0.0);
   }

   /**
    * Requests ground prep. {@code trajectoryTimeSeconds <= 0} keeps the controller default duration.
    */
   public void sendGroundPrepRequest(double trajectoryTimeSeconds)
   {
      HighLevelStateMessage highLevelStateMessage = new HighLevelStateMessage();
      highLevelStateMessage.setHighLevelControllerName(HighLevelControllerName.GROUND_PREP_STATE.toByte());
      highLevelStateMessage.setTrajectoryTime(trajectoryTimeSeconds);
      controllerHelper.publishToController(highLevelStateMessage);
   }

   /**
    * Sets the next ground-prep spline duration without requesting a state change.
    */
   public void sendGroundPrepTrajectoryDuration(double trajectoryTimeSeconds)
   {
      HighLevelControllerName state = currentHighLevelState != null ? currentHighLevelState : HighLevelControllerName.GROUND_PREP_STATE;
      HighLevelStateMessage highLevelStateMessage = new HighLevelStateMessage();
      highLevelStateMessage.setHighLevelControllerName(state.toByte());
      highLevelStateMessage.setTrajectoryTime(trajectoryTimeSeconds);
      controllerHelper.publishToController(highLevelStateMessage);
   }

   public void sendStandPrepTransitionRequest()
   {
      HighLevelStateMessage highLevelStateMessage = new HighLevelStateMessage();
      highLevelStateMessage.setHighLevelControllerName(HighLevelControllerName.STAND_TRANSITION_STATE.toByte());
      controllerHelper.publishToController(highLevelStateMessage);
   }

   public void sendFreezeRequest()
   {
      HighLevelStateMessage highLevelStateMessage = new HighLevelStateMessage();
      highLevelStateMessage.setHighLevelControllerName(HighLevelControllerName.FREEZE_STATE.toByte());
      controllerHelper.publishToController(highLevelStateMessage);
   }

   public void sendDoNothingRequest()
   {
      HighLevelStateMessage highLevelStateMessage = new HighLevelStateMessage();
      highLevelStateMessage.setHighLevelControllerName(HighLevelControllerName.DO_NOTHING_BEHAVIOR.toByte());
      controllerHelper.publishToController(highLevelStateMessage);
   }

   public void sendExitWalkingRequest()
   {
      HighLevelStateMessage highLevelStateMessage = new HighLevelStateMessage();
      highLevelStateMessage.setHighLevelControllerName(HighLevelControllerName.EXIT_WALKING.toByte());
      controllerHelper.publishToController(highLevelStateMessage);
   }

   public void sendRLRequest()
   {
      HighLevelStateMessage highLevelStateMessage = new HighLevelStateMessage();
      highLevelStateMessage.setHighLevelControllerName(HighLevelControllerName.RL_CONTROL.toByte());
      controllerHelper.publishToController(highLevelStateMessage);
   }

   public void sendWalkingRequest()
   {
      HighLevelStateMessage highLevelStateMessage = new HighLevelStateMessage();
      highLevelStateMessage.setHighLevelControllerName(HighLevelControllerName.WALKING.toByte());
      controllerHelper.publishToController(highLevelStateMessage);
   }

   public void update()
   {
   }

   public void destroy()
   {
   }
}
