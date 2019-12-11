package us.ihmc.valkyrie.planner.ui;

import controller_msgs.msg.dds.*;
import javafx.application.Application;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.Pane;
import javafx.stage.Stage;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerAPIDefinition;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ROS2Tools.MessageTopicNameGenerator;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatus;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javaFXVisualizers.JavaFXRobotVisualizer;
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
import us.ihmc.valkyrieRosControl.ValkyrieRosControlController;

import java.util.concurrent.atomic.AtomicReference;

public class ValkyrieFootstepPlannerUI extends Application
{
   private final ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, ValkyrieRosControlController.VERSION);
   private final ValkyrieAStarFootstepPlanner planner = new ValkyrieAStarFootstepPlanner(robotModel);
   private final AtomicReference<ValkyrieFootstepPlanningResult> planningResult = new AtomicReference<>();

   public static final String MODULE_NAME = "valkyrie_footstep_planner";
   private final Ros2Node rosNode = ROS2Tools.createRos2Node(PubSubImplementation.FAST_RTPS, MODULE_NAME);
   private final MessageTopicNameGenerator controllerSubNameGenerator = ControllerAPIDefinition.getSubscriberTopicNameGenerator(robotModel.getSimpleRobotName());
   private final MessageTopicNameGenerator controllerPubNameGenerator = ControllerAPIDefinition.getPublisherTopicNameGenerator(robotModel.getSimpleRobotName());
   private final IHMCROS2Publisher<FootstepDataListMessage> footstepPublisher = ROS2Tools.createPublisher(rosNode, FootstepDataListMessage.class, controllerSubNameGenerator);
   private final IHMCROS2Publisher<AbortWalkingMessage> abortPublisher = ROS2Tools.createPublisher(rosNode, AbortWalkingMessage.class, controllerSubNameGenerator);
   private final AtomicReference<PlanarRegionsList> planarRegionsList = new AtomicReference<>();

   private final PlanarRegionViewer planarRegionViewer = new PlanarRegionViewer();
   private JavaFXRobotVisualizer robotVisualizer = new JavaFXRobotVisualizer(robotModel);
   private ValkyriePlannerGraphicsViewer graphicsViewer = planner.createGraphicsViewer();
   private GoalPoseEditor goalPoseEditor;

   @FXML
   private ValkyriePlannerDashboardController valkyriePlannerDashboardController;

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

      valkyriePlannerDashboardController.setDoPlanningCallback(this::submitPlanningRequest);
      valkyriePlannerDashboardController.setHaltPlanningCallback(planner::halt);
      valkyriePlannerDashboardController.setDataSetSelectionCallback(name ->
                                                                     {
                                                                        setupWithDataSet(name);
                                                                        graphicsViewer.reset();
                                                                     });
      valkyriePlannerDashboardController.setParameters(planner.getParameters());

      graphicsViewer.setGoalPoseProperties(valkyriePlannerDashboardController.getGoalX().getValueFactory().valueProperty(),
                                           valkyriePlannerDashboardController.getGoalY().getValueFactory().valueProperty(),
                                           valkyriePlannerDashboardController.getGoalZ().getValueFactory().valueProperty(),
                                           valkyriePlannerDashboardController.getGoalYaw().getValueFactory().valueProperty());

      setupPubSubs();
      primaryStage.setOnCloseRequest(event -> stop());

      goalPoseEditor = new GoalPoseEditor(view3dFactory.getSubScene(),
                                          valkyriePlannerDashboardController.getGoalX().getValueFactory().valueProperty(),
                                          valkyriePlannerDashboardController.getGoalY().getValueFactory().valueProperty(),
                                          valkyriePlannerDashboardController.getGoalZ().getValueFactory().valueProperty(),
                                          valkyriePlannerDashboardController.getGoalYaw().getValueFactory().valueProperty());
      goalPoseEditor.start();
      valkyriePlannerDashboardController.setPlaceGoalCallback(goalPoseEditor::enable);
   }

   private void setupWithDataSet(DataSetName dataSetName)
   {
      if(dataSetName == null)
         return;

      DataSet dataSet = DataSetIOTools.loadDataSet(dataSetName);
      PlanarRegionsList planarRegionsList = dataSet.getPlanarRegionsList();
      planarRegionViewer.buildMeshAndMaterialOnThread(planarRegionsList);
      goalPoseEditor.setPlanarRegions(planarRegionsList);
      this.planarRegionsList.set(planarRegionsList);

      if (!dataSet.hasPlannerInput())
         return;

      double height = 0.995;
      PlannerInput plannerInput = dataSet.getPlannerInput();
      Vector3D rootJointPosition = new Vector3D(plannerInput.getStartPosition());
      rootJointPosition.addZ(height);
      Quaternion rootJointOrientation = new Quaternion(plannerInput.getStartYaw(), 0.0, 0.0);
      ValkyrieStandPrepParameters standPrepParameters = new ValkyrieStandPrepParameters(robotModel.getJointMap());
      robotVisualizer.submitNewConfiguration(rootJointOrientation, rootJointPosition, standPrepParameters::getSetpoint);

      valkyriePlannerDashboardController.setGoalPose(plannerInput.getGoalPosition().getX(),
                                                     plannerInput.getGoalPosition().getY(),
                                                     plannerInput.getGoalPosition().getZ(),
                                                     plannerInput.getGoalYaw());
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
                                           });
      ROS2Tools.createCallbackSubscription(rosNode, WalkingStatusMessage.class, controllerPubNameGenerator, s ->
      {
         WalkingStatus walkingStatus = WalkingStatus.fromByte(s.takeNextData().getWalkingStatus());
         if(walkingStatus == WalkingStatus.COMPLETED || walkingStatus == WalkingStatus.ABORT_REQUESTED || walkingStatus == WalkingStatus.PAUSED)
            graphicsViewer.reset();
      });

      valkyriePlannerDashboardController.setSendPlanningResultCallback(() -> footstepPublisher.publish(planningResult.get().getFootstepDataList()));
      valkyriePlannerDashboardController.setStopWalkingCallback(() -> abortPublisher.publish(new AbortWalkingMessage()));
   }

   public void stop()
   {
      planarRegionViewer.stop();
      robotVisualizer.stop();
      graphicsViewer.stop();
   }

   private void submitPlanningRequest()
   {
      ValkyrieFootstepPlanningRequestPacket requestPacket = new ValkyrieFootstepPlanningRequestPacket();
      requestPacket.getPlanarRegionsListMessage().set(PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(planarRegionsList.get()));
      requestPacket.setTimeout(valkyriePlannerDashboardController.getTimeout());
      requestPacket.setAssumeFlatGround(false);
      planner.getParameters().setPacket(requestPacket.getParameters());

      robotVisualizer.getFullRobotModel().updateFrames();
      requestPacket.getStartLeftFootPose().set(robotVisualizer.getFullRobotModel().getSoleFrame(RobotSide.LEFT).getTransformToWorldFrame());
      requestPacket.getStartRightFootPose().set(robotVisualizer.getFullRobotModel().getSoleFrame(RobotSide.RIGHT).getTransformToWorldFrame());

      requestPacket.getGoalLeftFootPose().set(valkyriePlannerDashboardController.getGoalPose());
      requestPacket.getGoalLeftFootPose().appendTranslation(0.0, 0.5 * planner.getParameters().getIdealFootstepWidth(), 0.0);
      requestPacket.getGoalRightFootPose().set(valkyriePlannerDashboardController.getGoalPose());
      requestPacket.getGoalRightFootPose().appendTranslation(0.0, - 0.5 * planner.getParameters().getIdealFootstepWidth(), 0.0);

      planningResult.set(planner.handleRequestPacket(requestPacket));
      valkyriePlannerDashboardController.onPlannerCompleted();
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}
