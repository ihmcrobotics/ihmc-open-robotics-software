package us.ihmc.gdx.ui.teleoperation;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.graphics.glutils.PixmapTextureData;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.GoHomeMessage;
import controller_msgs.msg.dds.HandDesiredConfigurationMessage;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImInt;
import org.apache.commons.lang3.tuple.MutablePair;
import org.apache.commons.lang3.tuple.Pair;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.behaviors.tools.footstepPlanner.MinimalFootstep;
import us.ihmc.behaviors.tools.yo.YoVariableClientHelper;
import us.ihmc.commons.FormattingTools;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Input;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.controllerAPI.RobotLowLevelMessenger;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
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
import us.ihmc.gdx.ui.affordances.ImGuiGDXPlannedFootstepPlacement;
import us.ihmc.gdx.ui.affordances.ImGuiGDXPoseGoalAffordance;
import us.ihmc.gdx.ui.graphics.GDXFootstepPlanGraphic;
import us.ihmc.gdx.ui.interactable.GDXChestOrientationSlider;
import us.ihmc.gdx.ui.interactable.GDXPelvisHeightSlider;
import us.ihmc.gdx.ui.visualizers.ImGuiGDXVisualizer;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.YawPitchRollAxis;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.tools.io.WorkspaceDirectory;
import us.ihmc.tools.io.WorkspaceFile;

import java.util.*;

/**
 *  Possibly extract simple controller controls to a smaller panel class, like remote safety controls or something.
 */
public class GDXTeleoperationManager extends ImGuiPanel implements RenderableProvider
{
   private final CommunicationHelper communicationHelper;
   private final RobotLowLevelMessenger robotLowLevelMessenger;
   private final ROS2SyncedRobotModel syncedRobot;
   private final GDXFootstepPlanGraphic footstepsSentToControllerGraphic;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private GDXPSIAdjustment psiAdjustment;
   private final FootstepPlannerParametersBasics footstepPlannerParameters;
   private final FootstepPlanningModule footstepPlanner;
   private final ImGuiGDXPoseGoalAffordance footstepGoal = new ImGuiGDXPoseGoalAffordance();
   private GDXRobotWholeBodyInteractable interactableRobot;
   private final ImGuiGDXManualFootstepPlacement manualFootstepPlacement = new ImGuiGDXManualFootstepPlacement();
   // TODO: for interactable footings from stepPlan >>
   private final ImGuiGDXPlannedFootstepPlacement plannedFootstepPlacement = new ImGuiGDXPlannedFootstepPlacement();
   // <<
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
   private final GDXPelvisHeightSlider pelvisHeightSlider;
   private final GDXChestOrientationSlider chestPitchSlider;
   private final GDXChestOrientationSlider chestYawSlider;
   private final GDXDesiredRobot desiredRobot;

   // FOR ICONS (NON-BUTTON)
   private final WorkspaceDirectory iconDirectory = new WorkspaceDirectory("ihmc-open-robotics-software",
                                                                           "ihmc-high-level-behaviors/src/libgdx/resources/icons");
   private final String iconFileNames[] = new String[] {"leftHand.png", "rightHand.png", "locationPin.png", "locationFlag.png"};
   private final String fileNameStringKeys[] = new String[] {"leftHand", "rightHand", "locationPin", "locationFlag"};
   private final Map<String, Texture> iconTexturesMap = new HashMap<>();
   private int textureID = 0;

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

      if (teleoperationParameters.getPSIAdjustable())
         psiAdjustment = new GDXPSIAdjustment(robotLowLevelMessenger);

      desiredRobot = new GDXDesiredRobot(robotModel, syncedRobot);

      ROS2ControllerHelper slidersROS2ControllerHelper = new ROS2ControllerHelper(ros2Node, robotModel);
      pelvisHeightSlider = new GDXPelvisHeightSlider(syncedRobot, slidersROS2ControllerHelper, teleoperationParameters);
      // TODO this should update the GDX desired Robot
      chestPitchSlider = new GDXChestOrientationSlider(syncedRobot, YawPitchRollAxis.PITCH, slidersROS2ControllerHelper, teleoperationParameters);
      // TODO this should update the GDX desired robot.
      chestYawSlider = new GDXChestOrientationSlider(syncedRobot, YawPitchRollAxis.YAW, slidersROS2ControllerHelper, teleoperationParameters);

      footstepsSentToControllerGraphic = new GDXFootstepPlanGraphic(robotModel.getContactPointParameters().getControllerFootGroundContactPoints());
      communicationHelper.subscribeToControllerViaCallback(FootstepDataListMessage.class, footsteps ->
      {
         footstepsSentToControllerGraphic.generateMeshesAsync(MinimalFootstep.convertFootstepDataListMessage(footsteps, "Teleoperation Panel Controller Spy"));
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
                                                               desiredRobot,
                                                               ros2Helper,
                                                               yoVariableClientHelper,
                                                               teleoperationParameters,
                                                               teleoperationParametersTuner);
      }
   }

   public void create(GDXImGuiBasedUI baseUI)
   {
      if (iconTexturesMap.size() == 0)
      {
         for (int i = 0; i < iconFileNames.length; ++i)
         {
            WorkspaceFile imageFile = new WorkspaceFile(iconDirectory, iconFileNames[i]);
            Mat readImage = opencv_imgcodecs.imread(imageFile.getFilePath().toString());
            Pixmap pixmap = new Pixmap(readImage.cols(), readImage.rows(), Pixmap.Format.RGBA8888);
            BytePointer rgba8888BytePointer = new BytePointer(pixmap.getPixels());
            Mat rgba8Mat = new Mat(readImage.rows(), readImage.cols(), opencv_core.CV_8UC4, rgba8888BytePointer);
            opencv_imgproc.cvtColor(readImage, rgba8Mat, opencv_imgproc.COLOR_RGB2BGRA);
            iconTexturesMap.put(fileNameStringKeys[i], new Texture(new PixmapTextureData(pixmap, null, false, false)));
         }
      }

      desiredRobot.create();

      // TODO: Remove this stuff and use the path control ring for this
      footstepGoal.create(baseUI, goal -> queueFootstepPlanning(), Color.YELLOW);
      baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(footstepGoal::processImGui3DViewInput);
      footstepPlanningParametersTuner.create(footstepPlannerParameters,
                                             FootstepPlannerParameterKeys.keys,
                                             this::queueFootstepPlanning);
      teleoperationParametersTuner.create(teleoperationParameters, GDXTeleoperationParameters.keys);
      // TODO: create (register) sliders here
      teleoperationParametersTuner.registerSlider("Swing time", 0.3f, 2.5f);
      teleoperationParametersTuner.registerSlider("Transfer time", 0.3f, 2.5f);
      teleoperationParametersTuner.registerSlider("Turn aggressiveness", 0.0f, 10.0f);

      manualFootstepPlacement.create(baseUI, communicationHelper, syncedRobot, teleoperationParameters, footstepPlannerParameters);

      baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(manualFootstepPlacement::processImGui3DViewInput);
      baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(manualFootstepPlacement::calculate3DViewPick);

      // TODO: for interactable footings from stepPlan >>
      plannedFootstepPlacement.create(baseUI, communicationHelper, syncedRobot, teleoperationParameters, footstepPlannerParameters);
      baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(plannedFootstepPlacement::processImGui3DViewInput);
      baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(plannedFootstepPlacement::calculate3DViewPick);
      // <<

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
      desiredRobot.update();
      footstepsSentToControllerGraphic.update();
      if (interactableRobot != null)
         interactableRobot.update(plannedFootstepPlacement);
      manualFootstepPlacement.update();
      plannedFootstepPlacement.update();
      if (manualFootstepPlacement.getFootstepArrayList().size() > 0)
      {
         footstepPlannerOutput = null;
         footstepsSentToControllerGraphic.clear();
      }
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
      if (teleoperationParameters.getPSIAdjustable())
         psiAdjustment.renderImGuiWidgets();

      pelvisHeightSlider.renderImGuiWidgets();
      chestPitchSlider.renderImGuiWidgets();
      chestYawSlider.renderImGuiWidgets();

      // TODO: sliders for footstep parameters here . . .
      // 2nd
      teleoperationParametersTuner.renderDoublePropertySliders();
      teleoperationParametersTuner.renderADoublePropertyTuner(GDXTeleoperationParameters.trajectoryTime, 0.1, 0.5, 0.0, 30.0, true, "s", "%.2f");

      ImGui.checkbox(labels.get("Show footstep planner parameter tuner"), footstepPlanningParametersTuner.getIsShowing());
      ImGui.checkbox(labels.get("Show teleoperation parameter tuner"), teleoperationParametersTuner.getIsShowing());

      ImGui.separator();

      manualFootstepPlacement.renderImGuiWidgets();
      plannedFootstepPlacement.renderImGuiWidgets();

      ImGui.image(iconTexturesMap.get("locationFlag").getTextureObjectHandle(), 22.0f, 22.0f);

      if (footstepPlannerOutput != null)
      {
         ImGui.sameLine();
         if (manualFootstepPlacement.getFootstepArrayList().size() == 0)
         {
            if (ImGui.button(labels.get("Walk")))
            {
               walkFromPlan();
            }
         }
      }
      ImGui.sameLine();
      footstepGoal.renderPlaceGoalButton();

      ImGui.text("Walk path control ring planner:");
      interactableRobot.getWalkPathControlRing().renderImGuiWidgets();

      ImGui.text("Footstep graphics:");
      ImGui.sameLine();
      ImGui.checkbox(labels.get("Show"), showGraphics);
      ImGui.sameLine();
      if (ImGui.button(labels.get("Clear")))
      {
         footstepsSentToControllerGraphic.clear();
         footstepGoal.clear();
         manualFootstepPlacement.clear();
         plannedFootstepPlacement.clear();
      }

      ImGui.separator();

      for (RobotSide side : RobotSide.values)
      {
         if (side == RobotSide.LEFT)
         {
            textureID = iconTexturesMap.get("leftHand").getTextureObjectHandle();
         }
         else if (side == RobotSide.RIGHT)
         {
            textureID = iconTexturesMap.get("rightHand").getTextureObjectHandle();
         }

         ImGui.image(textureID, 22.0f,22.0f);
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

      desiredRobot.renderImGuiWidgets();
      ImGui.sameLine();
      if (ImGui.button(labels.get("Set Desired To Current")))
      {
         interactableRobot.setDesiredToCurrent();
         desiredRobot.setDesiredToCurrent();
      }

      interactableRobot.renderImGuiWidgets();
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      desiredRobot.getRenderables(renderables, pool);

      if (showGraphics.get())
      {
         footstepsSentToControllerGraphic.getRenderables(renderables, pool);
         footstepGoal.getRenderables(renderables, pool);
         manualFootstepPlacement.getRenderables(renderables, pool);
         plannedFootstepPlacement.getRenderables(renderables, pool);
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
      else  // plan generated.
      {
//         footstepsSentToControllerGraphic.generateMeshesAsync(MinimalFootstep.reduceFootstepPlanForUIMessager(footstepPlannerOutput.getFootstepPlan(),
//                                                                                                 "Teleoperation Panel Planned"));
         // TODO: make footsteps from footstepPlan interactable (modifiable)
         plannedFootstepPlacement.updateFromPlan(footstepPlannerOutput.getFootstepPlan());
         footstepPlannerOutput.clear();
         this.footstepPlannerOutput = footstepPlannerOutput;
      }
   }

   private void walkFromPlan()
   {
      plannedFootstepPlacement.walkFromSteps();
   }

   public void destroy()
   {
      desiredRobot.destroy();
      if (interactableRobot != null)
         interactableRobot.destroy();
      footstepsSentToControllerGraphic.destroy();
   }

   public List<ImGuiGDXVisualizer> getVisualizers()
   {
      List<ImGuiGDXVisualizer> visualizers = new ArrayList<>();
      visualizers.add(desiredRobot);
      desiredRobot.setActive(true);

      return visualizers;
   }
}
