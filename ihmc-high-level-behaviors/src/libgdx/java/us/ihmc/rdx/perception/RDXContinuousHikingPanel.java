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
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.activeMapping.ContinuousHikingParameters;
import us.ihmc.behaviors.activeMapping.ContinuousPlannerSchedulingTask;
import us.ihmc.behaviors.activeMapping.StancePoseCalculator;
import us.ihmc.commonWalkingControlModules.configurations.SwingTrajectoryParameters;
import us.ihmc.commonWalkingControlModules.trajectories.PositionOptimizedTrajectoryGenerator;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.property.ROS2StoredPropertySetGroup;
import us.ihmc.communication.property.StoredPropertySetROS2TopicPair;
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
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.footstepPlanning.tools.SwingPlannerTools;
import us.ihmc.log.LogTools;
import us.ihmc.perception.comms.PerceptionComms;
import us.ihmc.perception.gpuHeightMap.RapidHeightMapExtractor;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.ui.ImGuiRemoteROS2StoredPropertySetGroup;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.RDXStoredPropertySetTuner;
import us.ihmc.robotics.math.trajectories.interfaces.PolynomialReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SegmentDependentList;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.tools.property.StoredPropertySetBasics;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;

public class RDXContinuousHikingPanel extends RDXPanel implements RenderableProvider
{
   private final ROS2Node ros2Node;
   private final DRCRobotModel robotModel;
   private final ROS2SyncedRobotModel syncedRobotModel;

   private static final int numberOfKnotPoints = 12;
   private static final int maxIterationsOptimization = 100;
   private final ROS2PublisherBasics<ContinuousWalkingCommandMessage> commandPublisher;
   private final ContinuousWalkingCommandMessage commandMessage = new ContinuousWalkingCommandMessage();

   private SideDependentList<FramePose3D> startStancePose = new SideDependentList<>(new FramePose3D(), new FramePose3D());
   private final RDXStancePoseSelectionPanel stancePoseSelectionPanel;
   private final PositionOptimizedTrajectoryGenerator positionTrajectoryGenerator = new PositionOptimizedTrajectoryGenerator(numberOfKnotPoints,
                                                                                                                             maxIterationsOptimization);
   private final RDXTerrainPlanningDebugger terrainPlanningDebugger;

   private final ContinuousHikingParameters continuousHikingParameters = new ContinuousHikingParameters();
   private final SwingTrajectoryParameters swingTrajectoryParameters;
   private final RDXStoredPropertySetTuner continuousHikingParametersPanel = new RDXStoredPropertySetTuner("Continuous Hiking Parameters (CH)");
   private final ImGuiRemoteROS2StoredPropertySetGroup remotePropertySets;

   private FootstepPlan latestFootstepPlan;
   private List<EnumMap<Axis3D, List<PolynomialReadOnly>>> swingTrajectories;

   private final ImBoolean localRenderMode = new ImBoolean(false);
   private final ImBoolean useMonteCarloReference = new ImBoolean(false);
   private final ImBoolean useHybridPlanner = new ImBoolean(false);
   private final ImBoolean useAStarFootstepPlanner = new ImBoolean(true);
   private final ImBoolean useMonteCarloFootstepPlanner = new ImBoolean(false);

   // When running in simulation only, these fields allow to run the Continuous Hiking Process locally
   private ContinuousPlannerSchedulingTask continuousPlannerSchedulingTask;
   private ROS2StoredPropertySetGroup ros2PropertySetGroup;
   private boolean runningLocally = false;

   public RDXContinuousHikingPanel(RDXBaseUI baseUI, ROS2Node ros2Node, ROS2Helper ros2Helper, DRCRobotModel robotModel, ROS2SyncedRobotModel syncedRobotModel)
   {
      super("Continuous Hiking");
      setRenderMethod(this::renderImGuiWidgets);

      this.ros2Node = ros2Node;
      this.robotModel = robotModel;
      this.syncedRobotModel = syncedRobotModel;

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
      stancePoseSelectionPanel = new RDXStancePoseSelectionPanel(baseUI, ros2Helper, stancePoseCalculator);
      addChild(stancePoseSelectionPanel);

      MonteCarloFootstepPlannerParameters monteCarloPlannerParameters = new MonteCarloFootstepPlannerParameters();
      DefaultFootstepPlannerParametersBasics footstepPlannerParameters = robotModel.getFootstepPlannerParameters("ForContinuousWalking");
      SwingPlannerParametersBasics swingPlannerParameters = robotModel.getSwingPlannerParameters();
      this.swingTrajectoryParameters = robotModel.getWalkingControllerParameters().getSwingTrajectoryParameters();

      terrainPlanningDebugger = new RDXTerrainPlanningDebugger(ros2Helper,
                                                               monteCarloPlannerParameters,
                                                               robotModel.getContactPointParameters().getControllerFootGroundContactPoints());

      ros2Helper.subscribeViaCallback(HumanoidControllerAPI.getTopic(WalkingControllerFailureStatusMessage.class, robotModel.getSimpleRobotName()),
                                      message -> terrainPlanningDebugger.reset());

      remotePropertySets = new ImGuiRemoteROS2StoredPropertySetGroup(ros2Helper);
      createParametersPanel(continuousHikingParameters, continuousHikingParametersPanel, remotePropertySets, ContinuousWalkingAPI.CONTINUOUS_HIKING_PARAMETERS);
      RDXStoredPropertySetTuner monteCarloPlannerParametersPanel = new RDXStoredPropertySetTuner("Monte Carlo Footstep Planner Parameters (CH)");
      createParametersPanel(monteCarloPlannerParameters,
                            monteCarloPlannerParametersPanel,
                            remotePropertySets,
                            ContinuousWalkingAPI.MONTE_CARLO_PLANNER_PARAMETERS);
      RDXStoredPropertySetTuner footstepPlanningParametersPanel = new RDXStoredPropertySetTuner("Footstep Planner Parameters (CH)");
      createParametersPanel(footstepPlannerParameters, footstepPlanningParametersPanel, remotePropertySets, ContinuousWalkingAPI.FOOTSTEP_PLANNING_PARAMETERS);
      RDXStoredPropertySetTuner swingPlannerParametersPanel = new RDXStoredPropertySetTuner("Swing Planner Parameters (CH)");
      createParametersPanel(swingPlannerParameters, swingPlannerParametersPanel, remotePropertySets, ContinuousWalkingAPI.SWING_PLANNING_PARAMETERS);
      RDXStoredPropertySetTuner heightMapParametersPanel = new RDXStoredPropertySetTuner("Height Map Parameters (CH)");
      createParametersPanel(RapidHeightMapExtractor.getHeightMapParameters(),
                            heightMapParametersPanel,
                            remotePropertySets,
                            PerceptionComms.HEIGHT_MAP_PARAMETERS);
   }

   /**
    * Sets up an ImGui Panel with the given StoredPropertySet
    */
   private void createParametersPanel(StoredPropertySetBasics storedPropertySetParameters,
                                      RDXStoredPropertySetTuner storedPropertySetPanel,
                                      ImGuiRemoteROS2StoredPropertySetGroup remotePropertySets,
                                      StoredPropertySetROS2TopicPair topicName)
   {
      LogTools.info("{%s} Save File", storedPropertySetParameters.findSaveFileDirectory().toString());
      storedPropertySetPanel.create(storedPropertySetParameters, false);
      remotePropertySets.registerRemotePropertySet(storedPropertySetParameters, topicName);
      this.addChild(storedPropertySetPanel);
   }

   /**
    * This allows the {@link ContinuousPlannerSchedulingTask} to be started for when things are running in simulation, during the operation on the robot this
    * method should not be called as it will interfere with the remote process
    */
   public void startContinuousPlannerSchedulingTask()
   {
      runningLocally = true;
      ROS2Helper ros2Helper = new ROS2Helper(ros2Node);
      ros2PropertySetGroup = new ROS2StoredPropertySetGroup(ros2Helper);

      // Add Continuous Hiking Parameters to be between the UI and this process
      ContinuousHikingParameters continuousHikingParameters = new ContinuousHikingParameters();
      ros2PropertySetGroup.registerStoredPropertySet(ContinuousWalkingAPI.CONTINUOUS_HIKING_PARAMETERS, continuousHikingParameters);

      // Add Monte Carlo Footstep Planner Parameters to be between the UI and this process
      MonteCarloFootstepPlannerParameters monteCarloPlannerParameters = new MonteCarloFootstepPlannerParameters();
      ros2PropertySetGroup.registerStoredPropertySet(ContinuousWalkingAPI.MONTE_CARLO_PLANNER_PARAMETERS, monteCarloPlannerParameters);

      // Add A* Footstep Planner Parameters to be between the UI and this process
      DefaultFootstepPlannerParametersBasics footstepPlannerParameters = robotModel.getFootstepPlannerParameters("ForContinuousWalking");
      ros2PropertySetGroup.registerStoredPropertySet(ContinuousWalkingAPI.FOOTSTEP_PLANNING_PARAMETERS, footstepPlannerParameters);

      // Add Swing Planner Parameters to be synced between the UI and this process
      SwingPlannerParametersBasics swingPlannerParameters = robotModel.getSwingPlannerParameters();
      ros2PropertySetGroup.registerStoredPropertySet(ContinuousWalkingAPI.SWING_PLANNING_PARAMETERS, swingPlannerParameters);

      continuousPlannerSchedulingTask = new ContinuousPlannerSchedulingTask(robotModel,
                                                                            ros2Node,
                                                                            syncedRobotModel.getReferenceFrames(),
                                                                            continuousHikingParameters,
                                                                            monteCarloPlannerParameters,
                                                                            footstepPlannerParameters,
                                                                            swingPlannerParameters);
   }

   public void update(TerrainMapData terrainMapData, HeightMapData heightMapData)
   {
      remotePropertySets.setPropertyChanged();

      // When running on the process we don't want to create the parameters locally, this gets done on the remote side
      if (runningLocally)
      {
         ros2PropertySetGroup.update();
      }

      if (latestFootstepPlan != null)
      {
         terrainPlanningDebugger.generateSwingGraphics(latestFootstepPlan, swingTrajectories);
      }
      latestFootstepPlan = null;
      terrainPlanningDebugger.update(terrainMapData);
      stancePoseSelectionPanel.update(terrainMapData, heightMapData);
   }

   public void renderImGuiWidgets()
   {
      ImGui.text("The ContinuousHikingProcess must be running");
      ImGui.text("And the enabled checkbox must be checked");
      ImGui.text("By holding CTRL the robot will walk forward");
      ImGui.separator();
      continuousHikingParametersPanel.renderImGuiWidgets();

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
      SideDependentList<FramePose3D> startStancePose = new SideDependentList<>(new FramePose3D(), new FramePose3D());

      if (!poseListMessage.getPoses().isEmpty())
      {
         List<Pose3D> poses = MessageTools.unpackPoseListMessage(poseListMessage);
         startStancePose.get(RobotSide.LEFT).set(poses.get(0));
         startStancePose.get(RobotSide.RIGHT).set(poses.get(1));
         goalStancePose.get(RobotSide.LEFT).set(poses.get(2));
         goalStancePose.get(RobotSide.RIGHT).set(poses.get(3));
      }

      // Visualize the start and goal poses on the UI
      terrainPlanningDebugger.generateStartAndGoalFootstepGraphics(startStancePose, goalStancePose);

      this.startStancePose = startStancePose;
   }

   public void onPlannedFootstepsReceived(FootstepDataListMessage footstepDataListMessage)
   {
      FootstepPlan footstepPLan = new FootstepPlan();
      List<EnumMap<Axis3D, List<PolynomialReadOnly>>> swingTrajectories = new ArrayList<>();

      if (!footstepDataListMessage.getFootstepDataList().isEmpty())
      {
         LogTools.info("Received footstep plan: {}", footstepDataListMessage.getFootstepDataList().size());

         footstepPLan = FootstepDataMessageConverter.convertToFootstepPlan(footstepDataListMessage);
         swingTrajectories = SwingPlannerTools.computeTrajectories(swingTrajectoryParameters, positionTrajectoryGenerator, startStancePose, footstepPLan);
      }

      terrainPlanningDebugger.generateFootstepPlanGraphic(footstepDataListMessage);

      this.latestFootstepPlan = footstepPLan;
      this.swingTrajectories = swingTrajectories;
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
      if (continuousHikingParameters.getEnableContinuousHiking())
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
      commandPublisher.remove();
      stancePoseSelectionPanel.destroy();
      terrainPlanningDebugger.destroy();
   }

   /**
    * This allows the {@link ContinuousPlannerSchedulingTask} to be started for when things are running in simulation, during the operation on the robot this
    * method should not be called as it will interfere with the remote process
    */
   public ContinuousPlannerSchedulingTask getContinuousPlannerSchedulingTask()
   {
      return continuousPlannerSchedulingTask;
   }

   public RDXStancePoseSelectionPanel getStancePoseSelectionPanel()
   {
      return stancePoseSelectionPanel;
   }
}
