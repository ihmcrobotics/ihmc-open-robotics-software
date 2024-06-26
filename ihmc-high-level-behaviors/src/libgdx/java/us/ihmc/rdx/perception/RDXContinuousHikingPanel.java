package us.ihmc.rdx.perception;

import behavior_msgs.msg.dds.ContinuousWalkingCommandMessage;
import com.badlogic.gdx.controllers.Controller;
import com.badlogic.gdx.controllers.Controllers;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.WalkingControllerFailureStatusMessage;
import ihmc_common_msgs.msg.dds.PoseListMessage;
import imgui.ImGui;
import imgui.type.ImBoolean;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.behaviors.activeMapping.ContinuousHikingParameters;
import us.ihmc.behaviors.activeMapping.StancePoseCalculator;
import us.ihmc.commonWalkingControlModules.configurations.SwingTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.trajectories.PositionOptimizedTrajectoryGenerator;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.FootstepDataMessageConverter;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.MonteCarloFootstepPlannerParameters;
import us.ihmc.footstepPlanning.communication.ContinuousWalkingAPI;
import us.ihmc.footstepPlanning.tools.SwingPlannerTools;
import us.ihmc.log.LogTools;
import us.ihmc.perception.HumanoidActivePerceptionModule;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.ui.RDXStoredPropertySetTuner;
import us.ihmc.robotics.math.trajectories.interfaces.PolynomialReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SegmentDependentList;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;

public class RDXContinuousHikingPanel extends RDXPanel implements RenderableProvider
{
   private static final int numberOfKnotPoints = 12;
   private static final int maxIterationsOptimization = 100;
   private final ROS2PublisherBasics<ContinuousWalkingCommandMessage> commandPublisher;
   private final ContinuousWalkingCommandMessage commandMessage = new ContinuousWalkingCommandMessage();
   private final SideDependentList<FramePose3D> startStancePose = new SideDependentList<>(new FramePose3D(), new FramePose3D());
   private final RDXStancePoseSelectionPanel stancePoseSelectionPanel;
   private final PositionOptimizedTrajectoryGenerator positionTrajectoryGenerator = new PositionOptimizedTrajectoryGenerator(numberOfKnotPoints,
                                                                                                                             maxIterationsOptimization);

   private final RDXTerrainPlanningDebugger terrainPlanningDebugger;
   private final HumanoidActivePerceptionModule activePerceptionModule;
   private final SwingTrajectoryParameters swingTrajectoryParameters;
   private final ContinuousHikingParameters continuousHikingParameters;
   private final RDXStoredPropertySetTuner continuousHikingParametersTuner = new RDXStoredPropertySetTuner("Continuous Hiking Parameters (Active Mapping)");

   private final ImBoolean localRenderMode = new ImBoolean(false);
   private final ImBoolean useMonteCarloReference = new ImBoolean(false);
   private final ImBoolean useHybridPlanner = new ImBoolean(false);
   private final ImBoolean useAStarFootstepPlanner = new ImBoolean(true);
   private final ImBoolean useMonteCarloFootstepPlanner = new ImBoolean(false);

   public RDXContinuousHikingPanel(ROS2Helper ros2Helper,
                                   DRCRobotModel robotModel,
                                   HumanoidActivePerceptionModule activePerceptionModule,
                                   ContinuousHikingParameters continuousHikingParameters,
                                   SwingTrajectoryParameters swingTrajectoryParameters,
                                   MonteCarloFootstepPlannerParameters monteCarloPlannerParameters)
   {
      super("Continuous Hiking");
      setRenderMethod(this::renderImGuiWidgets);

      this.activePerceptionModule = activePerceptionModule;
      this.swingTrajectoryParameters = swingTrajectoryParameters;
      this.continuousHikingParameters = continuousHikingParameters;

      ros2Helper.subscribeViaCallback(ContinuousWalkingAPI.START_AND_GOAL_FOOTSTEPS, this::onStartAndGoalPosesReceived);
      ros2Helper.subscribeViaCallback(ContinuousWalkingAPI.PLANNED_FOOTSTEPS, this::onPlannedFootstepsReceived);
      ros2Helper.subscribeViaCallback(ContinuousWalkingAPI.MONTE_CARLO_FOOTSTEP_PLAN, this::onMonteCarloPlanReceived);

      commandPublisher = ros2Helper.getROS2NodeInterface().createPublisher(ContinuousWalkingAPI.CONTINUOUS_WALKING_COMMAND);

      SegmentDependentList<RobotSide, ArrayList<Point2D>> groundContactPoints = robotModel.getContactPointParameters().getControllerFootGroundContactPoints();
      SideDependentList<ConvexPolygon2D> defaultContactPoints = new SideDependentList<>();
      for (RobotSide robotSide : RobotSide.values)
      {
         ConvexPolygon2D defaultFoothold = new ConvexPolygon2D();
         groundContactPoints.get(robotSide).forEach(defaultFoothold::addVertex);
         defaultFoothold.update();
         defaultContactPoints.put(robotSide, defaultFoothold);
      }
      StancePoseCalculator stancePoseCalculator = new StancePoseCalculator(defaultContactPoints);
      stancePoseSelectionPanel = new RDXStancePoseSelectionPanel(ros2Helper, stancePoseCalculator);
      addChild(stancePoseSelectionPanel);

      terrainPlanningDebugger = new RDXTerrainPlanningDebugger(ros2Helper,
                                                               monteCarloPlannerParameters,
                                                               robotModel.getContactPointParameters().getControllerFootGroundContactPoints());

      ros2Helper.subscribeViaCallback(HumanoidControllerAPI.getTopic(WalkingControllerFailureStatusMessage.class, robotModel.getSimpleRobotName()),
                                      message -> terrainPlanningDebugger.reset());

      LogTools.info("Continuous Hiking Parameters Save File " + continuousHikingParameters.findSaveFileDirectory().toString());
      continuousHikingParametersTuner.create(continuousHikingParameters, false);
   }

   public void update(TerrainMapData terrainMapData, HeightMapData heightMapData)
   {
      terrainPlanningDebugger.update(terrainMapData);
      stancePoseSelectionPanel.update(terrainMapData, heightMapData);
   }

   public void renderImGuiWidgets()
   {
      ImGui.text("The ContinuousHikingProcess must be running");
      ImGui.text("And the enabled checkbox must be checked");
      ImGui.text("By holding CTRL the robot will walk forward");
      ImGui.separator();
      continuousHikingParametersTuner.renderImGuiWidgets();

      ImGui.checkbox("Local Render Mode", localRenderMode);
      ImGui.checkbox("Use A* Footstep Planner", useAStarFootstepPlanner);
      ImGui.checkbox("Use Monte-Carlo Footstep Planner", useMonteCarloFootstepPlanner);
      ImGui.checkbox("Use Monte-Carlo Reference", useMonteCarloReference);
      ImGui.separator();
      terrainPlanningDebugger.renderImGuiWidgets();
      publishInputCommandMessage();
   }

   public void processImGui3DViewInput(ImGui3DViewInput input)
   {
      stancePoseSelectionPanel.processImGui3DViewInput(input);
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      stancePoseSelectionPanel.getRenderables(renderables, pool);
      terrainPlanningDebugger.getRenderables(renderables, pool);
   }

   /**
    * We have received the start and goal pose from the process, lets unpack this message and visualize the start and goal on the UI.
    */
   public void onStartAndGoalPosesReceived(PoseListMessage poseListMessage)
   {
      SideDependentList<FramePose3D> goalStancePose = new SideDependentList<>(new FramePose3D(), new FramePose3D());
      List<Pose3D> poses = MessageTools.unpackPoseListMessage(poseListMessage);
      startStancePose.get(RobotSide.LEFT).set(poses.get(0));
      startStancePose.get(RobotSide.RIGHT).set(poses.get(1));
      goalStancePose.get(RobotSide.LEFT).set(poses.get(2));
      goalStancePose.get(RobotSide.RIGHT).set(poses.get(3));

      // Visualize the start and goal poses on the UI
      terrainPlanningDebugger.generateStartAndGoalFootstepGraphics(startStancePose, goalStancePose);
   }

   public void onPlannedFootstepsReceived(FootstepDataListMessage footstepDataListMessage)
   {
      LogTools.debug("Received footstep plan: {}", footstepDataListMessage.getFootstepDataList().size());

      terrainPlanningDebugger.generateFootstepPlanGraphic(footstepDataListMessage);
      if (activePerceptionModule != null && localRenderMode.get())
      {
         terrainPlanningDebugger.generateSwingGraphics(activePerceptionModule.getContinuousPlannerSchedulingTask()
                                                                             .getContinuousPlanner()
                                                                             .getLatestFootstepPlan(),
                                                       activePerceptionModule.getContinuousPlannerSchedulingTask()
                                                                             .getContinuousPlanner()
                                                                             .getLatestSwingTrajectories());
      }
      else
      {
         FootstepPlan plan = FootstepDataMessageConverter.convertToFootstepPlan(footstepDataListMessage);
         List<EnumMap<Axis3D, List<PolynomialReadOnly>>> swingTrajectories = SwingPlannerTools.computeTrajectories(swingTrajectoryParameters,
                                                                                                                   positionTrajectoryGenerator,
                                                                                                                   startStancePose,
                                                                                                                   plan);
         terrainPlanningDebugger.generateSwingGraphics(plan, swingTrajectories);
      }
   }

   public void onMonteCarloPlanReceived(FootstepDataListMessage message)
   {
      LogTools.debug("Received Monte-Carlo Plan: {}", message.getFootstepDataList().size());
      terrainPlanningDebugger.generateMonteCarloPlanGraphic(message);
   }

   private void publishInputCommandMessage()
   {
      Controller currentController = Controllers.getCurrent();
      boolean currentJoystickControllerConnected = currentController != null;

      boolean walkingEnabled = ImGui.getIO().getKeyCtrl();
      double forwardJoystickValue = 0.0;
      double lateralJoystickValue = 0.0;
      double turningJoystickValue = 0.0;

      if (currentJoystickControllerConnected)
      {
         walkingEnabled |= currentController.getButton(currentController.getMapping().buttonR1);
         forwardJoystickValue = -currentController.getAxis(currentController.getMapping().axisLeftY);
         lateralJoystickValue = -currentController.getAxis(currentController.getMapping().axisLeftX);
         turningJoystickValue = -currentController.getAxis(currentController.getMapping().axisRightX);
      }

      // Only allow Continuous Walking if the CTRL key is held and the checkbox is checked
      if (continuousHikingParameters != null && continuousHikingParameters.getEnableContinuousWalking())
      {
         commandMessage.setEnableContinuousWalking(walkingEnabled);
         commandMessage.setPublishToController(ImGui.getIO().getKeyAlt());
         commandMessage.setForwardValue(forwardJoystickValue);
         commandMessage.setLateralValue(lateralJoystickValue);
         commandMessage.setTurningValue(turningJoystickValue);
         commandMessage.setUseMonteCarloFootstepPlanner(useMonteCarloFootstepPlanner.get());
         commandMessage.setUseAstarFootstepPlanner(useAStarFootstepPlanner.get());
         commandMessage.setUseMonteCarloPlanAsReference(useMonteCarloReference.get());
         commandMessage.setUsePreviousPlanAsReference(!useMonteCarloReference.get());
         commandMessage.setUseHybridPlanner(useHybridPlanner.get());

         commandPublisher.publish(commandMessage);
      }
   }

   public void destroy()
   {
      stancePoseSelectionPanel.destroy();
      terrainPlanningDebugger.destroy();
   }

   public RDXStancePoseSelectionPanel getStancePoseSelectionPanel()
   {
      return stancePoseSelectionPanel;
   }
}
