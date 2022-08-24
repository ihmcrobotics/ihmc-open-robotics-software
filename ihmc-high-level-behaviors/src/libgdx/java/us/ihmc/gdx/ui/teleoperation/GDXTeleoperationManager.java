package us.ihmc.gdx.ui.teleoperation;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.HandDesiredConfigurationMessage;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImInt;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.behaviors.tools.footstepPlanner.MinimalFootstep;
import us.ihmc.behaviors.tools.yo.YoVariableClientHelper;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameterKeys;
import us.ihmc.gdx.imgui.ImGuiMovingPlot;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.tools.GDXIconTexture;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.ImGuiStoredPropertySetTuner;
import us.ihmc.gdx.ui.affordances.GDXBallAndArrowPosePlacement;
import us.ihmc.gdx.ui.affordances.GDXInteractableFootstepPlan;
import us.ihmc.gdx.ui.affordances.GDXRobotWholeBodyInteractable;
import us.ihmc.gdx.ui.affordances.GDXManualFootstepPlacement;
import us.ihmc.gdx.ui.footstepPlanner.GDXFootstepPlanning;
import us.ihmc.gdx.ui.graphics.GDXFootstepPlanGraphic;
import us.ihmc.gdx.ui.interactable.GDXChestOrientationSlider;
import us.ihmc.gdx.ui.interactable.GDXPelvisHeightSlider;
import us.ihmc.gdx.ui.visualizers.ImGuiGDXVisualizer;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HandConfiguration;
import us.ihmc.robotics.geometry.YawPitchRollAxis;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.tools.io.WorkspaceDirectory;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

/**
 *  Possibly extract simple controller controls to a smaller panel class, like remote safety controls or something.
 */
public class GDXTeleoperationManager extends ImGuiPanel implements RenderableProvider
{
   private final CommunicationHelper communicationHelper;
   private final ROS2SyncedRobotModel syncedRobot;
   private final GDXFootstepPlanGraphic footstepsSentToControllerGraphic;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final GDXRobotLowLevelMessenger robotLowLevelMessenger;
   private final GDXFootstepPlanning footstepPlanning;
   private final GDXBallAndArrowPosePlacement ballAndArrowMidFeetPosePlacement = new GDXBallAndArrowPosePlacement();
   private GDXRobotWholeBodyInteractable interactableRobot;
   private final GDXManualFootstepPlacement manualFootstepPlacement = new GDXManualFootstepPlacement();
   // TODO: for interactable footings from stepPlan >>
   private final GDXInteractableFootstepPlan interactableFootstepPlan = new GDXInteractableFootstepPlan();
   // <<
   private final ImGuiStoredPropertySetTuner teleoperationParametersTuner = new ImGuiStoredPropertySetTuner("Teleoperation Parameters");
   private final ImGuiStoredPropertySetTuner footstepPlanningParametersTuner = new ImGuiStoredPropertySetTuner("Footstep Planner Parameters (Teleoperation)");
   private final GDXTeleoperationParameters teleoperationParameters;
   private final DRCRobotModel robotModel;
   private final ROS2NodeInterface ros2Node;
   private final ImGuiMovingPlot statusReceivedPlot = new ImGuiMovingPlot("Hand", 1000, 230, 15);
   private final SideDependentList<ImInt> handConfigurationIndices = new SideDependentList<>(new ImInt(6), new ImInt(6));
   private final String[] handConfigurationNames = new String[HandConfiguration.values.length];
   private final ImBoolean showGraphics = new ImBoolean(true);
   private final GDXPelvisHeightSlider pelvisHeightSlider;
   private final GDXChestOrientationSlider chestPitchSlider;
   private final GDXChestOrientationSlider chestYawSlider;
   private final GDXDesiredRobot desiredRobot;

   private final WorkspaceDirectory iconDirectory = new WorkspaceDirectory("ihmc-open-robotics-software",
                                                                           "ihmc-high-level-behaviors/src/libgdx/resources/icons");
   private GDXIconTexture locationFlagIcon;
   private final SideDependentList<GDXIconTexture> handIcons = new SideDependentList<>();

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
      ros2Node = communicationHelper.getROS2Node();
      robotModel = communicationHelper.getRobotModel();

      teleoperationParameters = new GDXTeleoperationParameters(robotRepoName, robotSubsequentPathToResourceFolder, robotModel.getSimpleRobotName());
      teleoperationParameters.load();
      teleoperationParameters.save();

      syncedRobot = communicationHelper.newSyncedRobot();

      robotLowLevelMessenger = new GDXRobotLowLevelMessenger(communicationHelper, teleoperationParameters);

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
      footstepPlanning = new GDXFootstepPlanning(robotModel, syncedRobot);

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
                                                               teleoperationParameters);
      }
   }

   public void create(GDXImGuiBasedUI baseUI)
   {
      for (RobotSide side : RobotSide.values)
      {
         handIcons.put(side, new GDXIconTexture(iconDirectory.file(side.getLowerCaseName() + "Hand.png")));
      }
      locationFlagIcon = new GDXIconTexture(iconDirectory.file("locationFlag.png"));

      desiredRobot.create();

      ballAndArrowMidFeetPosePlacement.create(Color.YELLOW);
      baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(ballAndArrowMidFeetPosePlacement::processImGui3DViewInput);
      footstepPlanningParametersTuner.create(footstepPlanning.getFootstepPlannerParameters(),
                                             FootstepPlannerParameterKeys.keys,
                                             footstepPlanning::plan);
      teleoperationParametersTuner.create(teleoperationParameters, GDXTeleoperationParameters.keys);
      teleoperationParametersTuner.registerSlider("Swing time", 0.3f, 2.5f);
      teleoperationParametersTuner.registerSlider("Transfer time", 0.3f, 2.5f);
      teleoperationParametersTuner.registerSlider("Turn aggressiveness", 0.0f, 10.0f);

      interactableFootstepPlan.create(baseUI, communicationHelper, syncedRobot, teleoperationParameters, footstepPlanning.getFootstepPlannerParameters());
      baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(interactableFootstepPlan::processImGui3DViewInput);
      baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(interactableFootstepPlan::calculate3DViewPick);

      manualFootstepPlacement.create(baseUI, interactableFootstepPlan);
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

      // Note: hot button for open / close hand
      
   }

   public void update()
   {
      syncedRobot.update();
      desiredRobot.update();
      footstepsSentToControllerGraphic.update();

      if (ballAndArrowMidFeetPosePlacement.getPlacedNotification().poll())
      {
         footstepPlanning.getMidFeetGoalPose().set(ballAndArrowMidFeetPosePlacement.getGoalPose());
         footstepPlanning.setGoalFootPosesFromMidFeetPose();
         footstepPlanning.setStanceSideToClosestToGoal();
         // TODO: Call planAsync
         footstepPlanning.plan();

         // TODO: make footsteps from footstepPlan interactable (modifiable)
         if (footstepPlanning.isReadyToWalk()) // failed
         {
            interactableFootstepPlan.updateFromPlan(footstepPlanning.getOutput().getFootstepPlan());
         }
      }

      if (interactableRobot != null)
         interactableRobot.update(interactableFootstepPlan);
      manualFootstepPlacement.update();
      interactableFootstepPlan.update();
      if (interactableFootstepPlan.getFootsteps().size() > 0)
      {
         footstepPlanning.setReadyToWalk(false);
         footstepsSentToControllerGraphic.clear();
      }
   }

   public void renderImGuiWidgets()
   {
      robotLowLevelMessenger.renderImGuiWidgets();
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

      ImGui.image(locationFlagIcon.getTexture().getTextureObjectHandle(), 22.0f, 22.0f);
      ImGui.sameLine();
      ballAndArrowMidFeetPosePlacement.renderPlaceGoalButton();

      ImGui.text("Walk path control ring planner:");
      interactableRobot.getWalkPathControlRing().renderImGuiWidgets();

      interactableFootstepPlan.renderImGuiWidgets();
      ImGui.sameLine();
      if (ImGui.button(labels.get("Delete All")))
      {
         footstepsSentToControllerGraphic.clear();
         ballAndArrowMidFeetPosePlacement.clear();
         manualFootstepPlacement.exitPlacement();
         interactableFootstepPlan.clear();
         interactableRobot.getWalkPathControlRing().delete();
      }
      ImGui.checkbox(labels.get("Show footstep related graphics"), showGraphics);

      ImGui.separator();

      for (RobotSide side : RobotSide.values)
      {
         ImGui.image(handIcons.get(side).getTexture().getTextureObjectHandle(), 22.0f,22.0f);
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

   private void publishHandCommand(RobotSide side, String openOrClose)
   {
      if (Objects.equals(openOrClose, "open"))
      {
         communicationHelper.publish(ROS2Tools::getHandConfigurationTopic,
                                     HumanoidMessageTools.createHandDesiredConfigurationMessage(side, HandConfiguration.OPEN));
      }
      else if (Objects.equals(openOrClose, "close"))
      {
         communicationHelper.publish(ROS2Tools::getHandConfigurationTopic,
                                     HumanoidMessageTools.createHandDesiredConfigurationMessage(side, HandConfiguration.CLOSE));
      }

   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      desiredRobot.getRenderables(renderables, pool);

      if (showGraphics.get())
      {
         footstepsSentToControllerGraphic.getRenderables(renderables, pool);
         ballAndArrowMidFeetPosePlacement.getRenderables(renderables, pool);
         manualFootstepPlacement.getRenderables(renderables, pool);
         interactableFootstepPlan.getRenderables(renderables, pool);
      }
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
