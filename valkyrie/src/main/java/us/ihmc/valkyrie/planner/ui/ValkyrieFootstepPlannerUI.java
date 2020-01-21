package us.ihmc.valkyrie.planner.ui;

import controller_msgs.msg.dds.*;
import javafx.application.Application;
import javafx.application.Platform;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.control.CheckBox;
import javafx.scene.input.KeyCode;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.Pane;
import javafx.stage.Stage;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatus;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javaFXVisualizers.JavaFXRobotVisualizer;
import us.ihmc.messager.Messager;
import us.ihmc.pathPlanning.DataSet;
import us.ihmc.pathPlanning.DataSetIOTools;
import us.ihmc.pathPlanning.DataSetName;
import us.ihmc.pathPlanning.PlannerInput;
import us.ihmc.pathPlanning.visibilityGraphs.ui.viewers.PlanarRegionViewer;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.ValkyrieStandPrepParameters;
import us.ihmc.valkyrie.planner.ValkyrieAStarFootstepPlanner;
import us.ihmc.valkyrie.planner.ValkyrieAStarFootstepPlanner.Status;
import us.ihmc.valkyrie.planner.log.ValkyriePlannerLogLoader.ValkyriePlannerLog;
import us.ihmc.valkyrieRosControl.ValkyrieRosControlController;

import java.util.concurrent.atomic.AtomicReference;

public class ValkyrieFootstepPlannerUI extends Application
{
   private final ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, ValkyrieRosControlController.VERSION);
   private final ValkyrieAStarFootstepPlanner planner;
   private final Messager messager = new SharedMemoryJavaFXMessager(ValkyriePlannerMessagerAPI.API);
   private final AtomicReference<ValkyrieFootstepPlanningStatus> planningResult = new AtomicReference<>();

   public static final String MODULE_NAME = "valkyrie_footstep_planner_ui";
   private final Ros2Node rosNode = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, MODULE_NAME);
   private final MessageTopicNameGenerator controllerSubNameGenerator = ControllerAPIDefinition.getSubscriberTopicNameGenerator(robotModel.getSimpleRobotName());
   private final MessageTopicNameGenerator controllerPubNameGenerator = ControllerAPIDefinition.getPublisherTopicNameGenerator(robotModel.getSimpleRobotName());
   private final IHMCROS2Publisher<FootstepDataListMessage> footstepPublisher = ROS2Tools.createPublisher(rosNode, FootstepDataListMessage.class, controllerSubNameGenerator);
   private final IHMCROS2Publisher<PauseWalkingMessage> pausePublisher = ROS2Tools.createPublisher(rosNode, PauseWalkingMessage.class, controllerSubNameGenerator);
   private final AtomicReference<PlanarRegionsList> planarRegionsList = new AtomicReference<>();

   private final PlanarRegionViewer planarRegionViewer = new PlanarRegionViewer();
   private JavaFXRobotVisualizer robotVisualizer = new JavaFXRobotVisualizer(robotModel);
   private ValkyriePlannerGraphicsViewer graphicsViewer;
   private FootstepPoseEditor goalPoseEditor;
   private FootstepPoseEditor waypointPoseEditor;

   @FXML
   private CheckBox showRobot;
   @FXML
   private CheckBox showPath;
   @FXML
   private CheckBox showDebugSteps;

   public ValkyrieFootstepPlannerUI()
   {
      this.planner = new ValkyrieAStarFootstepPlanner(robotModel);
      this.graphicsViewer = new ValkyriePlannerGraphicsViewer(planner.getSnapper(), planner.getParameters(), messager);
      startMessager();
   }

   public ValkyrieFootstepPlannerUI(ValkyrieAStarFootstepPlanner planner)
   {
      this.planner = planner;
      this.graphicsViewer = new ValkyriePlannerGraphicsViewer(planner.getSnapper(), planner.getParameters(), messager);
      planner.addRequestCallback(this::handleRequest);
      startMessager();
   }

   @FXML
   private ValkyriePlannerDashboardController valkyriePlannerDashboardController;
   @FXML
   private ValkyriePlannerParametersController valkyriePlannerParametersController;
   @FXML
   private ValkyriePlannerDebuggerController valkyriePlannerDebuggerController;

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      FXMLLoader loader = new FXMLLoader();
      loader.setController(this);
      loader.setLocation(getClass().getResource(getClass().getSimpleName() + ".fxml"));
      BorderPane mainPane = loader.load();

      View3DFactory view3dFactory = View3DFactory.createSubscene();
      view3dFactory.addCameraController(true);
      view3dFactory.addDefaultLighting();
      Pane subScene = view3dFactory.getSubSceneWrappedInsidePane();
      mainPane.setCenter(subScene);

      view3dFactory.addNodeToView(planarRegionViewer.getRoot());
      view3dFactory.addNodeToView(robotVisualizer.getRootNode());
      view3dFactory.addNodeToView(graphicsViewer.getRoot());

      planarRegionViewer.start();
      robotVisualizer.start();
      graphicsViewer.start();

      mainPane.setCenter(subScene);
      primaryStage.setTitle(getClass().getSimpleName());
      primaryStage.setMaximized(true);
      Scene mainScene = new Scene(mainPane, 600, 400);

      primaryStage.setScene(mainScene);
      primaryStage.show();

      valkyriePlannerDashboardController.setMessager(messager);
      valkyriePlannerDashboardController.setPlanner(planner);
      valkyriePlannerDashboardController.bindParameters();
      graphicsViewer.setGoalPoseProperties(valkyriePlannerDashboardController.getGoalX().getValueFactory().valueProperty(),
                                           valkyriePlannerDashboardController.getGoalY().getValueFactory().valueProperty(),
                                           valkyriePlannerDashboardController.getGoalZ().getValueFactory().valueProperty(),
                                           valkyriePlannerDashboardController.getGoalYaw().getValueFactory().valueProperty());
      graphicsViewer.setWaypointPoseProperties(valkyriePlannerDashboardController.getWaypointX().getValueFactory().valueProperty(),
                                               valkyriePlannerDashboardController.getWaypointY().getValueFactory().valueProperty(),
                                               valkyriePlannerDashboardController.getWaypointZ().getValueFactory().valueProperty(),
                                               valkyriePlannerDashboardController.getWaypointYaw().getValueFactory().valueProperty());

      valkyriePlannerParametersController.setPlanner(planner);
      valkyriePlannerParametersController.bindParameters();
      valkyriePlannerParametersController.setSpinnersFromPlannerParameters();

      setupPubSubs();
      primaryStage.setOnCloseRequest(event -> stop());

      goalPoseEditor = new FootstepPoseEditor(view3dFactory.getSubScene(),
                                              valkyriePlannerDashboardController.getGoalX().getValueFactory().valueProperty(),
                                              valkyriePlannerDashboardController.getGoalY().getValueFactory().valueProperty(),
                                              valkyriePlannerDashboardController.getGoalZ().getValueFactory().valueProperty(),
                                              valkyriePlannerDashboardController.getGoalYaw().getValueFactory().valueProperty());
      goalPoseEditor.start();
      waypointPoseEditor = new FootstepPoseEditor(view3dFactory.getSubScene(),
                                                  valkyriePlannerDashboardController.getWaypointX().getValueFactory().valueProperty(),
                                                  valkyriePlannerDashboardController.getWaypointY().getValueFactory().valueProperty(),
                                                  valkyriePlannerDashboardController.getWaypointZ().getValueFactory().valueProperty(),
                                                  valkyriePlannerDashboardController.getWaypointYaw().getValueFactory().valueProperty());
      waypointPoseEditor.start();

      valkyriePlannerDebuggerController.setup();
      valkyriePlannerDebuggerController.setMessager(messager);
      valkyriePlannerDebuggerController.initialize();

      planner.addRequestCallback(graphicsViewer::initialize);
      planner.addRequestCallback(result -> valkyriePlannerDashboardController.setTimerEnabled(true));
      planner.addIterationCallback(graphicsViewer::processIterationData);
      planner.addStatusCallback(graphicsViewer::processPlanningStatus);
      planner.addStatusCallback(planningResult::set);
      planner.addStatusCallback(valkyriePlannerDashboardController::updatePlanningStatus);
      planner.addStatusCallback(result -> valkyriePlannerDashboardController.setTimerEnabled(result.getPlannerStatus() == Status.PLANNING.toByte()));

      subScene.setOnKeyPressed(keyEvent ->
                               {
                                  if (keyEvent.getCode() == KeyCode.ESCAPE)
                                     pausePublisher.publish(new PauseWalkingMessage());
                               });


      messager.registerTopicListener(ValkyriePlannerMessagerAPI.doPlanning, request -> new Thread(this::submitPlanningRequest).start());
      messager.registerTopicListener(ValkyriePlannerMessagerAPI.haltPlanning, halt -> planner.halt());
      messager.registerTopicListener(ValkyriePlannerMessagerAPI.placeGoal, placeGoal -> goalPoseEditor.enable());
      messager.registerTopicListener(ValkyriePlannerMessagerAPI.addWaypoint, addWaypoint -> waypointPoseEditor.enable());
      messager.registerTopicListener(ValkyriePlannerMessagerAPI.dataSetSelected, this::setupWithDataSet);
      messager.registerTopicListener(ValkyriePlannerMessagerAPI.logToLoad, this::setupWithLog);

      robotVisualizer.getRootNode().setMouseTransparent(true);
      showRobot.setOnAction(event -> robotVisualizer.getRootNode().setVisible(showRobot.isSelected()));
      showPath.setOnAction(event -> graphicsViewer.showPath(showPath.isSelected()));
      showDebugSteps.setOnAction(event -> graphicsViewer.showDebugSteps(showDebugSteps.isSelected()));
   }

   private void setupWithDataSet(DataSetName dataSetName)
   {
      if(dataSetName == null)
         return;

      DataSet dataSet = DataSetIOTools.loadDataSet(dataSetName);
      PlanarRegionsList planarRegionsList = dataSet.getPlanarRegionsList();
      setRegions(planarRegionsList);

      if (!dataSet.hasPlannerInput() || Double.isNaN(dataSet.getPlannerInput().getGoalYaw()))
         return;

      PlannerInput plannerInput = dataSet.getPlannerInput();
      Pose3D startPose = new Pose3D(plannerInput.getStartPosition(), new Quaternion(plannerInput.getStartYaw(), 0.0, 0.0));
      Pose3D goalPose = new Pose3D(plannerInput.getGoalPosition(), new Quaternion(plannerInput.getGoalYaw(), 0.0, 0.0));
      setStartAndGoal(startPose, goalPose);
   }

   private void setupWithLog(ValkyriePlannerLog log)
   {
      PlanarRegionsList planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsList(log.getRequestPacket().getPlanarRegionsListMessage());

      // set regions
      setRegions(planarRegionsList);

      // set start/goal
      Pose3D startPose = new Pose3D();
      Pose3D goalPose = new Pose3D();
      startPose.interpolate(log.getRequestPacket().getStartLeftFootPose(), log.getRequestPacket().getStartRightFootPose(), 0.5);
      goalPose.interpolate(log.getRequestPacket().getGoalLeftFootPose(), log.getRequestPacket().getGoalRightFootPose(), 0.5);
      setStartAndGoal(startPose, goalPose);

      // set parameters
      planner.getParameters().setFromPacket(log.getRequestPacket().getParameters());
      valkyriePlannerParametersController.setSpinnersFromPlannerParameters();

      // set status/solution
      valkyriePlannerDashboardController.updatePlanningStatus(log.getStatusPacket());
      graphicsViewer.processPlanningStatus(log.getStatusPacket());

      // debugger
      valkyriePlannerDebuggerController.reset(log.getIterationData(), log.getEdgeDataMap(), planarRegionsList);
   }

   private void handleRequest(ValkyrieFootstepPlanningRequestPacket request)
   {
      setRegions(PlanarRegionMessageConverter.convertToPlanarRegionsList(request.getPlanarRegionsListMessage()));
      Pose3D startPose = new Pose3D();
      Pose3D goalPose = new Pose3D();
      startPose.interpolate(request.getStartLeftFootPose(), request.getStartRightFootPose(), 0.5);
      goalPose.interpolate(request.getGoalLeftFootPose(), request.getGoalRightFootPose(), 0.5);
      setStartAndGoal(startPose, goalPose);
   }

   private void setRegions(PlanarRegionsList planarRegionsList)
   {
      planarRegionViewer.buildMeshAndMaterialOnThread(planarRegionsList);
      goalPoseEditor.setPlanarRegions(planarRegionsList);
      waypointPoseEditor.setPlanarRegions(planarRegionsList);
      this.planarRegionsList.set(planarRegionsList);
   }

   private void setStartAndGoal(Pose3D startPose, Pose3D goalPose)
   {
      double xOffset = -0.0357;
      double height = 0.9902;
      startPose.appendTranslation(xOffset, 0.0, height);
      ValkyrieStandPrepParameters standPrepParameters = new ValkyrieStandPrepParameters(robotModel.getJointMap());
      robotVisualizer.submitNewConfiguration(startPose.getOrientation(), startPose.getPosition(), standPrepParameters::getSetpoint);
      valkyriePlannerDashboardController.setGoalPose(goalPose.getPosition(), goalPose.getYaw());
      graphicsViewer.clearWaypoints();
   }

   private void setupPubSubs()
   {
      ROS2Tools.createCallbackSubscription(rosNode, RobotConfigurationData.class, controllerPubNameGenerator, s -> robotVisualizer.submitNewConfiguration(s.takeNextData()));
      ROS2Tools.createCallbackSubscription(rosNode,
                                           PlanarRegionsListMessage.class,
                                           REACommunicationProperties.publisherTopicNameGenerator,
                                           s ->
                                           {
                                              PlanarRegionsList planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsList(s.takeNextData());
                                              this.planarRegionsList.set(planarRegionsList);
                                              planarRegionViewer.buildMeshAndMaterialOnThread(planarRegionsList);
                                              goalPoseEditor.setPlanarRegions(planarRegionsList);
                                              waypointPoseEditor.setPlanarRegions(planarRegionsList);
                                           });
      ROS2Tools.createCallbackSubscription(rosNode, WalkingStatusMessage.class, controllerPubNameGenerator, s ->
      {
         WalkingStatus walkingStatus = WalkingStatus.fromByte(s.takeNextData().getWalkingStatus());
         if(walkingStatus == WalkingStatus.COMPLETED || walkingStatus == WalkingStatus.PAUSED)
            graphicsViewer.reset();
      });

      messager.registerTopicListener(ValkyriePlannerMessagerAPI.sendPlanningResult, send -> footstepPublisher.publish(planningResult.get().getFootstepDataList()));
      messager.registerTopicListener(ValkyriePlannerMessagerAPI.stopWalking, send -> pausePublisher.publish(new PauseWalkingMessage()));
   }

   public void stop()
   {
      planarRegionViewer.stop();
      robotVisualizer.stop();
      graphicsViewer.stop();
      planner.closeAndDispose();
   }

   private void submitPlanningRequest()
   {
      ValkyrieFootstepPlanningRequestPacket requestPacket = new ValkyrieFootstepPlanningRequestPacket();

      requestPacket.setTimeout(valkyriePlannerDashboardController.getTimeout());
      requestPacket.setAssumeFlatGround(false);
      planner.getParameters().setPacket(requestPacket.getParameters());
      if(planarRegionsList.get() != null)
         requestPacket.getPlanarRegionsListMessage().set(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(planarRegionsList.get()));

      robotVisualizer.getFullRobotModel().updateFrames();
      requestPacket.getStartLeftFootPose().set(robotVisualizer.getFullRobotModel().getSoleFrame(RobotSide.LEFT).getTransformToWorldFrame());
      requestPacket.getStartRightFootPose().set(robotVisualizer.getFullRobotModel().getSoleFrame(RobotSide.RIGHT).getTransformToWorldFrame());

      requestPacket.getGoalLeftFootPose().set(valkyriePlannerDashboardController.getGoalPose());
      requestPacket.getGoalLeftFootPose().appendTranslation(0.0, 0.5 * planner.getParameters().getIdealFootstepWidth(), 0.0);
      requestPacket.getGoalRightFootPose().set(valkyriePlannerDashboardController.getGoalPose());
      requestPacket.getGoalRightFootPose().appendTranslation(0.0, - 0.5 * planner.getParameters().getIdealFootstepWidth(), 0.0);

      graphicsViewer.packWaypoints(requestPacket);

      planner.handleRequestPacket(requestPacket);
      Platform.runLater(() -> valkyriePlannerDebuggerController.reset(planner.getIterationData(), planner.getEdgeDataMap(), planarRegionsList.get()));
   }

   private void startMessager()
   {
      try
      {
         messager.startMessager();
      }
      catch (Exception e)
      {
         e.printStackTrace();
         throw new RuntimeException("Unable to start message");
      }
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}
