package us.ihmc.gdx.ui.teleoperation;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.msg.dds.*;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImInt;
import org.apache.commons.lang3.tuple.MutablePair;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.behaviors.tools.ThrottledRobotStateCallback;
import us.ihmc.behaviors.tools.footstepPlanner.MinimalFootstep;
import us.ihmc.behaviors.tools.yo.YoVariableClientHelper;
import us.ihmc.commons.FormattingTools;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.RobotLowLevelMessenger;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameYawPitchRoll;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.FootstepDataMessageConverter;
import us.ihmc.footstepPlanning.FootstepPlannerOutput;
import us.ihmc.footstepPlanning.FootstepPlannerRequest;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameterKeys;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogger;
import us.ihmc.footstepPlanning.tools.FootstepPlannerRejectionReasonReport;
import us.ihmc.gdx.imgui.ImGuiMovingPlot;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.ImGuiStoredPropertySetTuner;
import us.ihmc.gdx.ui.affordances.GDXRobotWholeBodyInteractable;
import us.ihmc.gdx.ui.affordances.ImGuiGDXManualFootstepPlacement;
import us.ihmc.gdx.ui.affordances.ImGuiGDXPoseGoalAffordance;
import us.ihmc.gdx.ui.graphics.GDXFootstepPlanGraphic;
import us.ihmc.gdx.ui.interactable.GDXChestOrientationSlider;
import us.ihmc.gdx.ui.teleoperation.GDXTeleoperationParameters;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.YawPitchRollAxis;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.tools.string.StringTools;

import java.util.ArrayList;
import java.util.UUID;

/**
 *  Possibly extract simple controller controls to a smaller panel class, like remote safety controls or something.
 */
public class GDXTeleoperationManager extends ImGuiPanel implements RenderableProvider
{
   private static final String WINDOW_NAME = "Teleoperation";
   private static final double MIN_PELVIS_HEIGHT = 0.52;
   private static final double MAX_PELVIS_HEIGHT = 0.90;
   private static final double PELVIS_HEIGHT_RANGE = MAX_PELVIS_HEIGHT - MIN_PELVIS_HEIGHT;
   private static final double MIN_CHEST_PITCH = Math.toRadians(-15.0);
   private static final double MAX_CHEST_PITCH = Math.toRadians(50.0);
   private static final double CHEST_PITCH_RANGE = MAX_CHEST_PITCH - MIN_CHEST_PITCH;
   private static final double MIN_YAW_TORSO = Math.toRadians(-45.0);
   private static final double MAX_YAW_TORSO = Math.toRadians(45.0);
   private static final double YAW_TORSO_RANGE = MAX_YAW_TORSO - MIN_YAW_TORSO;
   private static final double SLIDER_RANGE = 100.0;
   private static final double ROBOT_DATA_EXPIRATION = 1.0;
   private final CommunicationHelper communicationHelper;
   private final ThrottledRobotStateCallback throttledRobotStateCallback;
   private final RobotLowLevelMessenger robotLowLevelMessenger;
   private final ROS2SyncedRobotModel syncedRobot;
   private final GDXFootstepPlanGraphic footstepPlanGraphic;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final float[] stanceHeightSliderValue = new float[1];
   private final float[] pitchTorsoSliderValue = new float[1];
   private final float[] yawTorsoSliderValue = new float[1];
   private final float[] neckPitchSliderValue = new float[1];
   private final ImInt pumpPSI = new ImInt(1);
   private final String[] psiValues = new String[] {"1500", "2300", "2500", "2800"};
//   private final OneDoFJointBasics neckJoint;
//   private double neckJointJointLimitLower;
//   private double neckJointRange;
   private final FootstepPlannerParametersBasics footstepPlannerParameters;
   private final FootstepPlanningModule footstepPlanner;
   private final ImGuiGDXPoseGoalAffordance footstepGoal = new ImGuiGDXPoseGoalAffordance();
   private GDXRobotWholeBodyInteractable interactableRobot;
   private final ImGuiGDXManualFootstepPlacement manualFootstepPlacement = new ImGuiGDXManualFootstepPlacement();
   private final ImGuiStoredPropertySetTuner teleoperationParametersTuner = new ImGuiStoredPropertySetTuner("Teleoperation Parameters");
   private final ImGuiStoredPropertySetTuner footstepPlanningParametersTuner = new ImGuiStoredPropertySetTuner("Footstep Planner Parameters (Teleoperation)");
   private final GDXTeleoperationParameters teleoperationParameters;
   private final DRCRobotModel robotModel;
   private final ROS2NodeInterface ros2Node;
   private FootstepPlannerOutput footstepPlannerOutput;
   private final ROS2SyncedRobotModel syncedRobotForFootstepPlanning;
   private final SideDependentList<FramePose3D> startFootPoses = new SideDependentList<>();
   private final ImGuiMovingPlot statusReceivedPlot = new ImGuiMovingPlot("Hand", 1000, 230, 15);
   private final SideDependentList<ImInt> handConfigurationIndices = new SideDependentList<>(new ImInt(6), new ImInt(6));
   private final String[] handConfigurationNames = new String[HandConfiguration.values.length];
   private final IHMCROS2Input<PlanarRegionsListMessage> lidarREARegions;
   private final ImBoolean showGraphics = new ImBoolean(true);
   private GDXChestOrientationSlider chestPitchSlider;
   private GDXChestOrientationSlider chestYawSlider;

   public GDXTeleoperationManager(String robotRepoName,
                                  String robotSubsequentPathToResourceFolder,
                                  CommunicationHelper communicationHelper)
   {
      this(robotRepoName, robotSubsequentPathToResourceFolder, communicationHelper, null, null, null);
   }

   public GDXTeleoperationManager(String robotRepoName,
                                  String robotSubsequentPathToResourceFolder,
                                  CommunicationHelper communicationHelper,
                                  RobotCollisionModel robotSelfCollisionModel,
                                  RobotCollisionModel robotEnvironmentCollisionModel,
                                  YoVariableClientHelper yoVariableClientHelper)
   {
      super("Teleoperation");

      setRenderMethod(this::renderImGuiWidgets);
      addChild(teleoperationParametersTuner);
      addChild(footstepPlanningParametersTuner);
      this.communicationHelper = communicationHelper;
      String robotName = communicationHelper.getRobotModel().getSimpleRobotName();
      ros2Node = communicationHelper.getROS2Node();
      robotModel = communicationHelper.getRobotModel();

      teleoperationParameters = new GDXTeleoperationParameters(robotRepoName, robotSubsequentPathToResourceFolder, robotModel.getSimpleRobotName());
      teleoperationParameters.load();
      teleoperationParameters.save();

      syncedRobot = communicationHelper.newSyncedRobot();
      robotLowLevelMessenger = communicationHelper.newRobotLowLevelMessenger();

      if (robotLowLevelMessenger == null)
      {
         throw new RuntimeException("Please add implementation of RobotLowLevelMessenger for " + robotName);
      }

//      neckJoint = fullRobotModel.getNeckJoint(NeckJointName.PROXIMAL_NECK_PITCH);
//      if (neckJoint != null)
//      {
//         double neckJointLimitUpper = neckJoint.getJointLimitUpper();
//         neckJointJointLimitLower = neckJoint.getJointLimitLower();
//         neckJointRange = neckJointLimitUpper - neckJointJointLimitLower;
//      }

      throttledRobotStateCallback = new ThrottledRobotStateCallback(ros2Node, robotModel, 5.0, syncedRobot ->
      {
         double pelvisZ = syncedRobot.getFramePoseReadOnly(HumanoidReferenceFrames::getPelvisZUpFrame).getZ();
         double midFeetZ = syncedRobot.getFramePoseReadOnly(HumanoidReferenceFrames::getMidFeetZUpFrame).getZ();
         double midFeetToPelvis = pelvisZ - midFeetZ;
         double heightInRange = midFeetToPelvis - MIN_PELVIS_HEIGHT;
         double newHeightSliderValue = SLIDER_RANGE * heightInRange / PELVIS_HEIGHT_RANGE;
         stanceHeightSliderValue[0] = (float) newHeightSliderValue;
      });

      ROS2ControllerHelper chestSlidersROS2ControllerHelper = new ROS2ControllerHelper(ros2Node, robotModel);
      chestPitchSlider = new GDXChestOrientationSlider(syncedRobot, YawPitchRollAxis.PITCH, chestSlidersROS2ControllerHelper, teleoperationParameters);
      chestYawSlider = new GDXChestOrientationSlider(syncedRobot, YawPitchRollAxis.YAW, chestSlidersROS2ControllerHelper, teleoperationParameters);

      footstepPlanGraphic = new GDXFootstepPlanGraphic(robotModel.getContactPointParameters().getControllerFootGroundContactPoints());
      communicationHelper.subscribeToControllerViaCallback(FootstepDataListMessage.class, footsteps ->
      {
         footstepPlanGraphic.generateMeshesAsync(MinimalFootstep.convertFootstepDataListMessage(footsteps, "Teleoperation Panel Controller Spy"));
      });
      footstepPlannerParameters = communicationHelper.getRobotModel().getFootstepPlannerParameters();
      footstepPlanner = communicationHelper.getOrCreateFootstepPlanner();
      syncedRobotForFootstepPlanning = communicationHelper.newSyncedRobot();
      startFootPoses.put(RobotSide.LEFT, new FramePose3D());
      startFootPoses.put(RobotSide.RIGHT, new FramePose3D());
      lidarREARegions = communicationHelper.subscribe(ROS2Tools.LIDAR_REA_REGIONS);

      HandConfiguration[] values = HandConfiguration.values;
      for (int i = 0; i < values.length; i++)
      {
         handConfigurationNames[i] = values[i].name();
      }

      if (robotSelfCollisionModel != null)
      {
         ROS2ControllerHelper ros2Helper = new ROS2ControllerHelper(ros2Node, robotModel);
         interactableRobot = new GDXRobotWholeBodyInteractable(robotSelfCollisionModel,
                                                               robotEnvironmentCollisionModel,
                                                               robotModel,
                                                               syncedRobot,
                                                               ros2Helper,
                                                               yoVariableClientHelper,
                                                               teleoperationParameters,
                                                               teleoperationParametersTuner);
      }
   }

   public void create(GDXImGuiBasedUI baseUI)
   {
      // TODO: Remove this stuff and use the path control ring for this
      footstepGoal.create(baseUI, goal -> queueFootstepPlanning(), Color.YELLOW);
      baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(footstepGoal::processImGui3DViewInput);
      footstepPlanningParametersTuner.create(footstepPlannerParameters,
                                             FootstepPlannerParameterKeys.keys,
                                             this::queueFootstepPlanning);
      teleoperationParametersTuner.create(teleoperationParameters, GDXTeleoperationParameters.keys);

      manualFootstepPlacement.create(baseUI, communicationHelper, syncedRobot, teleoperationParameters, footstepPlannerParameters);
      baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(manualFootstepPlacement::processImGui3DViewInput);
      baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(manualFootstepPlacement::calculate3DViewPick);

      if (interactableRobot != null)
      {
         interactableRobot.create(baseUI);
         baseUI.getPrimaryScene().addRenderableProvider(interactableRobot, GDXSceneLevel.VIRTUAL);
         baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(interactableRobot::calculate3DViewPick);
         baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(interactableRobot::process3DViewInput);
         interactableRobot.setInteractablesEnabled(true);
         baseUI.getPrimaryScene().addRenderableProvider(interactableRobot);
      }
   }

   public void update()
   {
      syncedRobot.update();
      footstepPlanGraphic.update();
      if (interactableRobot != null)
         interactableRobot.update();
      manualFootstepPlacement.update();
   }

   public void renderImGuiWidgets()
   {
      if (ImGui.button("Home Pose"))
      {
         double trajectoryTime = 3.5;

         GoHomeMessage homeLeftArm = new GoHomeMessage();
         homeLeftArm.setHumanoidBodyPart(GoHomeMessage.HUMANOID_BODY_PART_ARM);
         homeLeftArm.setRobotSide(GoHomeMessage.ROBOT_SIDE_LEFT);
         homeLeftArm.setTrajectoryTime(trajectoryTime);
         communicationHelper.publishToController(homeLeftArm);

         GoHomeMessage homeRightArm = new GoHomeMessage();
         homeRightArm.setHumanoidBodyPart(GoHomeMessage.HUMANOID_BODY_PART_ARM);
         homeRightArm.setRobotSide(GoHomeMessage.ROBOT_SIDE_RIGHT);
         homeRightArm.setTrajectoryTime(trajectoryTime);
         communicationHelper.publishToController(homeRightArm);

         GoHomeMessage homePelvis = new GoHomeMessage();
         homePelvis.setHumanoidBodyPart(GoHomeMessage.HUMANOID_BODY_PART_PELVIS);
         homePelvis.setTrajectoryTime(trajectoryTime);
         communicationHelper.publishToController(homePelvis);

         GoHomeMessage homeChest = new GoHomeMessage();
         homeChest.setHumanoidBodyPart(GoHomeMessage.HUMANOID_BODY_PART_CHEST);
         homeChest.setTrajectoryTime(trajectoryTime);
         communicationHelper.publishToController(homeChest);
      }
      ImGui.sameLine();
      if (ImGui.button("Stand prep"))
      {
         robotLowLevelMessenger.sendStandRequest();
      }
      ImGui.sameLine();
      if (ImGui.button("Abort"))
      {
         robotLowLevelMessenger.sendAbortWalkingRequest();
      }
      ImGui.sameLine();
      if (ImGui.button("Pause"))
      {
         robotLowLevelMessenger.sendPauseWalkingRequest();
      }
      ImGui.sameLine();
      if (ImGui.button("Continue"))
      {
         robotLowLevelMessenger.sendContinueWalkingRequest();
      }
      if (ImGui.button("Freeze"))
      {
         robotLowLevelMessenger.sendFreezeRequest();
      }
      ImGui.sameLine();
      if (ImGui.button("Shutdown"))
      {
         robotLowLevelMessenger.sendShutdownRequest();
      }
      if (ImGui.combo("PSI", pumpPSI, psiValues, psiValues.length))
      {
         sendPSIRequest();
      }
      ImGui.sameLine();
      if (ImGui.button("Resend PSI"))
      {
         sendPSIRequest();
      }

      if (imGuiSlider("Height", stanceHeightSliderValue))
      {
         if (syncedRobot.getDataReceptionTimerSnapshot().isRunning(ROBOT_DATA_EXPIRATION))
         {
            double sliderValue = stanceHeightSliderValue[0];
            double pelvisZ = syncedRobot.getFramePoseReadOnly(HumanoidReferenceFrames::getPelvisZUpFrame).getZ();
            double midFeetZ = syncedRobot.getFramePoseReadOnly(HumanoidReferenceFrames::getMidFeetZUpFrame).getZ();
            double desiredHeight = MIN_PELVIS_HEIGHT + PELVIS_HEIGHT_RANGE * sliderValue / SLIDER_RANGE;
            double desiredHeightInWorld = desiredHeight + midFeetZ;
            LogTools.info(StringTools.format3D("Commanding height trajectory. slider: {} desired: {} (pelvis - midFeetZ): {} in world: {}",
                                               sliderValue,
                                               desiredHeight,
                                               pelvisZ - midFeetZ,
                                               desiredHeightInWorld));
            PelvisHeightTrajectoryMessage message = new PelvisHeightTrajectoryMessage();
            message.getEuclideanTrajectory()
                   .set(HumanoidMessageTools.createEuclideanTrajectoryMessage(2.0,
                                                                              new Point3D(0.0, 0.0, desiredHeightInWorld),
                                                                              ReferenceFrame.getWorldFrame()));
            long frameId = MessageTools.toFrameId(ReferenceFrame.getWorldFrame());
            message.getEuclideanTrajectory().getFrameInformation().setDataReferenceFrameId(frameId);
            message.getEuclideanTrajectory().getSelectionMatrix().setXSelected(false);
            message.getEuclideanTrajectory().getSelectionMatrix().setYSelected(false);
            message.getEuclideanTrajectory().getSelectionMatrix().setZSelected(true);
            communicationHelper.publishToController(message);
         }
      }
      chestPitchSlider.renderImGuiWidgets();
      chestYawSlider.renderImGuiWidgets();
//      if (neckJoint != null && imGuiSlider("Neck Pitch", neckPitchSliderValue))
//      {
//         double percent = neckPitchSliderValue[0] / 100.0;
//         percent = 1.0 - percent;
//         MathTools.checkIntervalContains(percent, 0.0, 1.0);
//         double jointAngle = neckJointJointLimitLower + percent * neckJointRange;
//         LogTools.info("Commanding neck trajectory: slider: {} angle: {}", neckPitchSliderValue[0], jointAngle);
//         communicationHelper.publishToController(HumanoidMessageTools.createNeckTrajectoryMessage(3.0, new double[] {jointAngle}));
//      }
      ImGui.text("Footstep plan:");
      if (footstepPlannerOutput != null)
      {
         ImGui.sameLine();
         if (ImGui.button(labels.get("Walk")))
         {
            walk();
         }
      }
      ImGui.sameLine();
      footstepGoal.renderPlaceGoalButton();
      ImGui.checkbox(labels.get("Show footstep planner parameter tuner"), footstepPlanningParametersTuner.getIsShowing());
      ImGui.checkbox(labels.get("Show teleoperation parameter tuner"), teleoperationParametersTuner.getIsShowing());

      manualFootstepPlacement.renderImGuiWidgets();

      ImGui.sameLine();
      if (ImGui.button(labels.get("Clear")))
      {
         manualFootstepPlacement.clear();
      }

      ImGui.sameLine();
      if (ImGui.button(labels.get("Delete Last")))
      {
         manualFootstepPlacement.removeFootStep();
      }

      for (RobotSide side : RobotSide.values)
      {
         ImGui.text(side.getPascalCaseName() + " hand:");
         ImGui.sameLine();
         if (ImGui.button(labels.get("Calibrate", side.getCamelCaseName())))
         {
            communicationHelper.publish(ROS2Tools::getHandConfigurationTopic,
                                        HumanoidMessageTools.createHandDesiredConfigurationMessage(side, HandConfiguration.CALIBRATE));
         }
         ImGui.sameLine();
         if (ImGui.button(labels.get("Open", side.getCamelCaseName())))
         {
            communicationHelper.publish(ROS2Tools::getHandConfigurationTopic,
                                        HumanoidMessageTools.createHandDesiredConfigurationMessage(side, HandConfiguration.OPEN));
         }
         ImGui.sameLine();
         if (ImGui.button(labels.get("Close", side.getCamelCaseName())))
         {
            communicationHelper.publish(ROS2Tools::getHandConfigurationTopic,
                                        HumanoidMessageTools.createHandDesiredConfigurationMessage(side, HandConfiguration.CLOSE));
         }
         ImGui.sameLine();
         ImGui.pushItemWidth(100.0f);
         ImGui.combo(labels.get("Grip", side.getCamelCaseName()), handConfigurationIndices.get(side), handConfigurationNames);
         ImGui.popItemWidth();
         ImGui.sameLine();
         if (ImGui.button(labels.get("Send", side.getCamelCaseName())))
         {
            HandDesiredConfigurationMessage message
                  = HumanoidMessageTools.createHandDesiredConfigurationMessage(side, HandConfiguration.values[handConfigurationIndices.get(side).get()]);
            communicationHelper.publish(ROS2Tools::getHandConfigurationTopic, message);
         }
      }

      ImGui.text("Lidar REA:");
      ImGui.sameLine();
      if (ImGui.button(labels.get("Clear")))
      {
         REAStateRequestMessage clearMessage = new REAStateRequestMessage();
         clearMessage.setRequestClear(true);
         communicationHelper.publish(ROS2Tools.REA_STATE_REQUEST, clearMessage);
      }
      ImGui.checkbox(labels.get("Show graphics"), showGraphics);
      ImGui.sameLine();
      if (ImGui.button(labels.get("Clear graphics")))
      {
         footstepPlanGraphic.clear();
         footstepGoal.clear();
         manualFootstepPlacement.clear();
      }

      interactableRobot.renderImGuiWidgets();
   }

   private boolean imGuiSlider(String label, float[] value)
   {
      float previousValue = value[0];
      ImGui.sliderFloat(label, value, 0.0f, 100.0f);
      float currentValue = value[0];
      return currentValue != previousValue;
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (showGraphics.get())
      {
         footstepPlanGraphic.getRenderables(renderables, pool);
         footstepGoal.getRenderables(renderables, pool);
         manualFootstepPlacement.getRenderables(renderables, pool);
      }
   }

   private void queueFootstepPlanning()
   {
      Pose3DReadOnly goalPose = footstepGoal.getGoalPose();
      syncedRobotForFootstepPlanning.update();
      for (RobotSide side : RobotSide.values)
      {
         startFootPoses.get(side).set(syncedRobotForFootstepPlanning.getFramePoseReadOnly(referenceFrames -> referenceFrames.getSoleFrame(side)));
      }

      RobotSide stanceSide;
      if (startFootPoses.get(RobotSide.LEFT ).getPosition().distance(goalPose.getPosition())
          <= startFootPoses.get(RobotSide.RIGHT).getPosition().distance(goalPose.getPosition()))
      {
         stanceSide = RobotSide.LEFT;
      }
      else
      {
         stanceSide = RobotSide.RIGHT;
      }

      FootstepPlannerRequest footstepPlannerRequest = new FootstepPlannerRequest();
      footstepPlannerRequest.setPlanBodyPath(false);
      footstepPlannerRequest.setRequestedInitialStanceSide(stanceSide);
      footstepPlannerRequest.setStartFootPoses(startFootPoses.get(RobotSide.LEFT), startFootPoses.get(RobotSide.RIGHT));
      // TODO: Set start footholds!!
      footstepPlannerRequest.setGoalFootPoses(footstepPlannerParameters.getIdealFootstepWidth(), goalPose);
      //      footstepPlannerRequest.setPlanarRegionsList(...);
      footstepPlannerRequest.setAssumeFlatGround(true); // FIXME Assuming flat ground
      //      footstepPlannerRequest.setTimeout(lookAndStepParameters.getFootstepPlannerTimeoutWhileStopped());
      //      footstepPlannerRequest.setSwingPlannerType(swingPlannerType);
      //      footstepPlannerRequest.setSnapGoalSteps(true);

      footstepPlanner.getFootstepPlannerParameters().set(footstepPlannerParameters);
      LogTools.info("Stance side: {}", stanceSide.name());
      LogTools.info("Planning footsteps...");
      FootstepPlannerOutput footstepPlannerOutput = footstepPlanner.handleRequest(footstepPlannerRequest);
      LogTools.info("Footstep planner completed with {}, {} step(s)",
                    footstepPlannerOutput.getFootstepPlanningResult(),
                    footstepPlannerOutput.getFootstepPlan().getNumberOfSteps());

      FootstepPlannerLogger footstepPlannerLogger = new FootstepPlannerLogger(footstepPlanner);
      footstepPlannerLogger.logSession();
      ThreadTools.startAThread(() -> FootstepPlannerLogger.deleteOldLogs(50), "FootstepPlanLogDeletion");

      if (footstepPlannerOutput.getFootstepPlan().getNumberOfSteps() < 1) // failed
      {
         FootstepPlannerRejectionReasonReport rejectionReasonReport = new FootstepPlannerRejectionReasonReport(footstepPlanner);
         rejectionReasonReport.update();
         ArrayList<Pair<Integer, Double>> rejectionReasonsMessage = new ArrayList<>();
         for (BipedalFootstepPlannerNodeRejectionReason reason : rejectionReasonReport.getSortedReasons())
         {
            double rejectionPercentage = rejectionReasonReport.getRejectionReasonPercentage(reason);
            LogTools.info("Rejection {}%: {}", FormattingTools.getFormattedToSignificantFigures(rejectionPercentage, 3), reason);
            rejectionReasonsMessage.add(MutablePair.of(reason.ordinal(), MathTools.roundToSignificantFigures(rejectionPercentage, 3)));
         }
         LogTools.info("Footstep planning failure...");
      }
      else
      {
         footstepPlanGraphic.generateMeshesAsync(MinimalFootstep.reduceFootstepPlanForUIMessager(footstepPlannerOutput.getFootstepPlan(),
                                                                                                 "Teleoperation Panel Planned"));
         this.footstepPlannerOutput = footstepPlannerOutput;
      }
   }

   private void walk()
   {
      FootstepDataListMessage footstepDataListMessage
            = FootstepDataMessageConverter.createFootstepDataListFromPlan(footstepPlannerOutput.getFootstepPlan(),
                                                                          teleoperationParameters.getSwingTime(),
                                                                          teleoperationParameters.getTransferTime());
      footstepDataListMessage.getQueueingProperties().setExecutionMode(ExecutionMode.OVERRIDE.toByte());
      footstepDataListMessage.getQueueingProperties().setMessageId(UUID.randomUUID().getLeastSignificantBits());
      communicationHelper.publishToController(footstepDataListMessage);
      footstepPlannerOutput = null;
   }

   private void sendPSIRequest()
   {
      robotLowLevelMessenger.setHydraulicPumpPSI(Integer.parseInt(psiValues[pumpPSI.get()]));
   }

   public void destroy()
   {
      if (interactableRobot != null)
         interactableRobot.destroy();
      footstepPlanGraphic.destroy();
      throttledRobotStateCallback.destroy();
   }
}
