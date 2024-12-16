package us.ihmc.rdx.perception;

import behavior_msgs.msg.dds.ContinuousHikingCommandMessage;
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
import us.ihmc.footstepPlanning.communication.ContinuousHikingAPI;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.footstepPlanning.tools.SwingPlannerTools;
import us.ihmc.log.LogTools;
import us.ihmc.perception.comms.PerceptionComms;
import us.ihmc.perception.gpuHeightMap.RapidHeightMapExtractor;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.rdx.imgui.ImGuiSliderDouble;
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
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.tools.property.StoredPropertySetBasics;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;

public class RDXContinuousHikingPanel extends RDXPanel implements RenderableProvider
{
   private static final int numberOfKnotPoints = 12;
   private static final int maxIterationsOptimization = 100;
   private final ROS2Node ros2Node;
   private final ROS2Helper ros2Helper;
   private final DRCRobotModel robotModel;
   private final ROS2SyncedRobotModel syncedRobotModel;
   private final ROS2Publisher<ContinuousHikingCommandMessage> commandPublisher;
   private final ContinuousHikingCommandMessage commandMessage = new ContinuousHikingCommandMessage();
   private final RDXStancePoseSelectionPanel stancePoseSelectionPanel;
   private final PositionOptimizedTrajectoryGenerator positionTrajectoryGenerator = new PositionOptimizedTrajectoryGenerator(numberOfKnotPoints,
                                                                                                                             maxIterationsOptimization);
   private final RDXTerrainPlanningDebugger terrainPlanningDebugger;
   private final SwingTrajectoryParameters swingTrajectoryParameters;
   private final RDXStoredPropertySetTuner continuousHikingParametersPanel = new RDXStoredPropertySetTuner("Continuous Hiking Parameters (CH)");
   private final ImGuiRemoteROS2StoredPropertySetGroup hostStoredPropertySets;
   private final ImGuiSliderDouble stepsBeforeSafetyStop = new ImGuiSliderDouble("Steps Before Safety Stop", "%.2f");
   private final ImBoolean enableContinuousHiking = new ImBoolean(false);
   private final ImBoolean squareUpToGoal = new ImBoolean(false);
   private final ImBoolean useAStarFootstepPlanner = new ImBoolean(true);
   private final ImBoolean useMonteCarloReference = new ImBoolean(false);
   private final ImBoolean useMonteCarloFootstepPlanner = new ImBoolean(false);
   private SideDependentList<FramePose3D> startStancePose = new SideDependentList<>(new FramePose3D(), new FramePose3D());
   private FootstepPlan latestFootstepPlan;
   private List<EnumMap<Axis3D, List<PolynomialReadOnly>>> swingTrajectories;
   // When running in simulation only, these fields allow running the Continuous Hiking Process locally
   private ContinuousPlannerSchedulingTask continuousPlannerSchedulingTask;
   private ROS2StoredPropertySetGroup clientStoredPropertySets;
   private boolean runSubscriberOnly = false;
   private boolean publishAndSubscribe;

   public RDXContinuousHikingPanel(RDXBaseUI baseUI, ROS2Node ros2Node, ROS2Helper ros2Helper, DRCRobotModel robotModel, ROS2SyncedRobotModel syncedRobotModel)
   {
      super("Continuous Hiking");
      this.ros2Helper = ros2Helper;
      setRenderMethod(this::renderImGuiWidgets);

      this.ros2Node = ros2Node;
      this.robotModel = robotModel;
      this.syncedRobotModel = syncedRobotModel;

      ros2Helper.subscribeViaCallback(ContinuousHikingAPI.START_AND_GOAL_FOOTSTEPS, this::onStartAndGoalPosesReceived);
      ros2Helper.subscribeViaCallback(ContinuousHikingAPI.PLANNED_FOOTSTEPS, this::onPlannedFootstepsReceived);
      ros2Helper.subscribeViaCallback(ContinuousHikingAPI.MONTE_CARLO_FOOTSTEP_PLAN, this::onMonteCarloPlanReceived);

      commandPublisher = ros2Helper.getROS2Node().createPublisher(ContinuousHikingAPI.CONTINUOUS_HIKING_COMMAND);

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

      hostStoredPropertySets = new ImGuiRemoteROS2StoredPropertySetGroup(ros2Helper);
      ContinuousHikingParameters continuousHikingParameters = new ContinuousHikingParameters();
      createParametersPanel(continuousHikingParameters,
                            continuousHikingParametersPanel,
                            hostStoredPropertySets,
                            ContinuousHikingAPI.CONTINUOUS_HIKING_PARAMETERS);
      RDXStoredPropertySetTuner monteCarloPlannerParametersPanel = new RDXStoredPropertySetTuner("Monte Carlo Footstep Planner Parameters (CH)");
      createParametersPanel(monteCarloPlannerParameters,
                            monteCarloPlannerParametersPanel,
                            hostStoredPropertySets,
                            ContinuousHikingAPI.MONTE_CARLO_PLANNER_PARAMETERS);
      RDXStoredPropertySetTuner footstepPlanningParametersPanel = new RDXStoredPropertySetTuner("Footstep Planner Parameters (CH)");
      createParametersPanel(footstepPlannerParameters,
                            footstepPlanningParametersPanel,
                            hostStoredPropertySets,
                            ContinuousHikingAPI.FOOTSTEP_PLANNING_PARAMETERS);
      RDXStoredPropertySetTuner swingPlannerParametersPanel = new RDXStoredPropertySetTuner("Swing Planner Parameters (CH)");
      createParametersPanel(swingPlannerParameters, swingPlannerParametersPanel, hostStoredPropertySets, ContinuousHikingAPI.SWING_PLANNING_PARAMETERS);
      RDXStoredPropertySetTuner heightMapParametersPanel = new RDXStoredPropertySetTuner("Height Map Parameters (CH)");
      createParametersPanel(RapidHeightMapExtractor.getHeightMapParameters(),
                            heightMapParametersPanel,
                            hostStoredPropertySets,
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
   public void startContinuousPlannerSchedulingTask(boolean publishAndSubscribe)
   {
      this.publishAndSubscribe = publishAndSubscribe;
      runSubscriberOnly = true;
      ROS2Helper ros2Helper = new ROS2Helper(ros2Node);
      clientStoredPropertySets = new ROS2StoredPropertySetGroup(ros2Helper);

      // Add Continuous Hiking Parameters to be between the UI and this process
      ContinuousHikingParameters continuousHikingParameters = new ContinuousHikingParameters();
      clientStoredPropertySets.registerStoredPropertySet(ContinuousHikingAPI.CONTINUOUS_HIKING_PARAMETERS, continuousHikingParameters);

      // Add Monte Carlo Footstep Planner Parameters to be between the UI and this process
      MonteCarloFootstepPlannerParameters monteCarloPlannerParameters = new MonteCarloFootstepPlannerParameters();
      clientStoredPropertySets.registerStoredPropertySet(ContinuousHikingAPI.MONTE_CARLO_PLANNER_PARAMETERS, monteCarloPlannerParameters);

      // Add A* Footstep Planner Parameters to be between the UI and this process
      DefaultFootstepPlannerParametersBasics footstepPlannerParameters = robotModel.getFootstepPlannerParameters("ForContinuousWalking");
      clientStoredPropertySets.registerStoredPropertySet(ContinuousHikingAPI.FOOTSTEP_PLANNING_PARAMETERS, footstepPlannerParameters);

      // Add Swing Planner Parameters to be synced between the UI and this process
      SwingPlannerParametersBasics swingPlannerParameters = robotModel.getSwingPlannerParameters();
      clientStoredPropertySets.registerStoredPropertySet(ContinuousHikingAPI.SWING_PLANNING_PARAMETERS, swingPlannerParameters);

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
      updateRos2StoredPropertySets();

      if (latestFootstepPlan != null)
      {
         terrainPlanningDebugger.generateSwingGraphics(latestFootstepPlan, swingTrajectories);
      }
      latestFootstepPlan = null;
      terrainPlanningDebugger.update(terrainMapData);
      stancePoseSelectionPanel.update(terrainMapData, heightMapData);
   }

   /**
    * This method handles updating the stored property sets used in Continuous Hiking.
    * These are all the parameters that are getting synced back and forth between the remote process and the local process.
    * There are three situations that can occur when trying to use Continuous Hiking.
    * <ul>
    *    <li>Case 1: The situation where we are simulating the process running on a remote machine but in reality its running locally.
    *    This is where we only want to update the property sets running on that process. Represented by {@link #clientStoredPropertySets}.
    *    So in this sense we are only subscribing to any updates sent from the user</li>
    *    <li>Case 2: The situation where we are running everything in one simulation.
    *    Here we want to publish, and subscribe in one place as everything is being run on the same machine.
    *    So we update {@link #clientStoredPropertySets} and {@link #hostStoredPropertySets}</li>
    *    <li>Case 3: Then the situation where we only want to publish the property sets to be sent to the remote process.
    *    This is when we don't want to subscribe but we publish and changes to {@link #hostStoredPropertySets} so the remote process can receive these changes</li>
    *
    * </ul>
    */
   private void updateRos2StoredPropertySets()
   {
      if (runSubscriberOnly && !publishAndSubscribe)  // Case 1
      {
         clientStoredPropertySets.update();
      }
      else if (publishAndSubscribe) // Case 2
      {
         clientStoredPropertySets.update();
         hostStoredPropertySets.setPropertyChanged();
      }
      else  // Case 3
      {
         hostStoredPropertySets.setPropertyChanged();
      }
   }

   public void renderImGuiWidgets()
   {
      ImGui.text("The ContinuousHikingProcess must be running");
      ImGui.text("And the enabled checkbox must be checked");
      ImGui.text("By holding CTRL the robot will walk forward");
      ImGui.separator();
      continuousHikingParametersPanel.renderImGuiWidgets();

      ImGui.separator();
      ImGui.text("Options for Continuous Hiking Message");
      ImGui.indent();
      ImGui.checkbox("Enable Continuous Hiking", enableContinuousHiking);
      ImGui.checkbox("Square Up To Goal", squareUpToGoal);
      if (ImGui.button("Clear Planned footsteps"))
      {
         clearPlannedFootsteps();
      }
      ImGui.sameLine();
      stepsBeforeSafetyStop.render(0.0, 50.0);
      ImGui.checkbox("Use A* Footstep Planner", useAStarFootstepPlanner);
      ImGui.checkbox("Use Monte-Carlo Footstep Planner", useMonteCarloFootstepPlanner);
      ImGui.checkbox("Use Monte-Carlo Reference", useMonteCarloReference);
      ImGui.unindent();
      ImGui.separator();
      terrainPlanningDebugger.renderImGuiWidgets();

      // Check to see if a controller is plugged into the computer
      Controller joystickController = Controllers.getCurrent();
      // Here we check against null rather then .isConnected() because if the controller is unplugged, that method won't work
      boolean controllerConnected = joystickController != null;

      if (ImGui.getIO().getKeyCtrl() && ImGui.getIO().getKeyShift())
      {
         publishContinuousHikingCommand();
      }
      else if (controllerConnected)
      {
         if (joystickController.getButton(joystickController.getMapping().buttonA))
         {
            publishJoystickStatus(joystickController);
         }

         if (joystickController.getButton(joystickController.getMapping().buttonX))
         {
            publishStopContinuousHiking();
         }
      }

      if (ImGui.getIO().getKeyAlt())
      {
         publishStopContinuousHiking();
      }
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

   public void clearPlannedFootsteps()
   {
      ros2Helper.publish(ContinuousHikingAPI.CLEAR_GOAL_FOOTSTEPS);
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

   private void publishStopContinuousHiking()
   {
      commandMessage.setEnableContinuousHiking(false);
      commandPublisher.publish(commandMessage);
   }

   private void publishJoystickStatus(Controller joystickController)
   {

      // Setup variables to be published in the message
      boolean walkBackwards = false;
      double forwardJoystickValue = 0.0;
      double lateralJoystickValue = 0.0;
      double turningJoystickValue = 0.0;

      walkBackwards = joystickController.getButton(joystickController.getMapping().buttonB);
      forwardJoystickValue = -joystickController.getAxis(joystickController.getMapping().axisLeftY);
      lateralJoystickValue = -joystickController.getAxis(joystickController.getMapping().axisLeftX);
      turningJoystickValue = -joystickController.getAxis(joystickController.getMapping().axisRightX);

      commandMessage.setEnableContinuousHiking(true);
      commandMessage.setUseJoystickController(true);
      commandMessage.setForwardValue(forwardJoystickValue);
      commandMessage.setWalkBackwards(walkBackwards);
      commandMessage.setLateralValue(lateralJoystickValue);
      commandMessage.setTurningValue(turningJoystickValue);

      commandPublisher.publish(commandMessage);
   }

   /**
    * Here we want to publish and walk forward, that is the point of pressing the keys on the keyboard
    */
   private void publishContinuousHikingCommand()
   {
      commandMessage.setEnableContinuousHiking(enableContinuousHiking.get());
      commandMessage.setStepsBeforeSafetyStop((int) stepsBeforeSafetyStop.getDoubleValue());
      commandMessage.setWalkForwards(true);
      commandMessage.setSquareUpToGoal(squareUpToGoal.get());
      commandMessage.setUseAstarFootstepPlanner(useAStarFootstepPlanner.get());
      commandMessage.setUseMonteCarloFootstepPlanner(useMonteCarloFootstepPlanner.get());
      commandMessage.setUseMonteCarloPlanAsReference(useMonteCarloReference.get());
      commandMessage.setUsePreviousPlanAsReference(!useMonteCarloReference.get());

      commandMessage.setUseJoystickController(false);
      commandMessage.setForwardValue(0.0);
      commandMessage.setWalkBackwards(false);
      commandMessage.setLateralValue(0.0);
      commandMessage.setTurningValue(0.0);

      commandPublisher.publish(commandMessage);
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