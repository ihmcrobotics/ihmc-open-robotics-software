package us.ihmc.humanoidBehaviors.ui.behaviors;

import static us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehavior.LookAndStepBehaviorAPI.BodyPathPlanForUI;
import static us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehavior.LookAndStepBehaviorAPI.CurrentState;
import static us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehavior.LookAndStepBehaviorAPI.FootstepPlanForUI;
import static us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehavior.LookAndStepBehaviorAPI.FootstepPlannerParameters;
import static us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehavior.LookAndStepBehaviorAPI.LookAndStepParameters;
import static us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehavior.LookAndStepBehaviorAPI.MapRegionsForUI;
import static us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehavior.LookAndStepBehaviorAPI.OperatorReviewEnabled;
import static us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehavior.LookAndStepBehaviorAPI.RePlan;
import static us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehavior.LookAndStepBehaviorAPI.SubGoalForUI;
import static us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehavior.LookAndStepBehaviorAPI.TakeStep;
import static us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehavior.LookAndStepBehaviorAPI.BodyPathPlanInput;

import java.util.ArrayList;
import java.util.List;

import javafx.application.Platform;
import javafx.fxml.FXML;
import javafx.scene.SubScene;
import javafx.scene.control.Button;
import javafx.scene.control.CheckBox;
import javafx.scene.control.TableView;
import javafx.scene.control.TextField;
import javafx.scene.paint.Color;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.communication.RemoteREAInterface;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.graphSearch.VisibilityGraphPathPlanner;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameterKeys;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehavior;
import us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehaviorParameters;
import us.ihmc.humanoidBehaviors.tools.HumanoidRobotState;
import us.ihmc.humanoidBehaviors.tools.RemoteHumanoidRobotInterface;
import us.ihmc.humanoidBehaviors.ui.BehaviorUIDefinition;
import us.ihmc.humanoidBehaviors.ui.BehaviorUIInterface;
import us.ihmc.humanoidBehaviors.ui.editors.OrientationYawEditor;
import us.ihmc.humanoidBehaviors.ui.editors.SnappedPositionEditor;
import us.ihmc.humanoidBehaviors.ui.graphics.BodyPathPlanGraphic;
import us.ihmc.humanoidBehaviors.ui.graphics.FootstepPlanGraphic;
import us.ihmc.humanoidBehaviors.ui.graphics.PositionGraphic;
import us.ihmc.humanoidBehaviors.ui.graphics.live.LivePlanarRegionsGraphic;
import us.ihmc.humanoidBehaviors.ui.model.FXUIActionMap;
import us.ihmc.humanoidBehaviors.ui.model.FXUITrigger;
import us.ihmc.javafx.parameter.JavaFXStoredPropertyTable;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersBasics;
import us.ihmc.pathPlanning.visibilityGraphs.postProcessing.BodyPathPostProcessor;
import us.ihmc.pathPlanning.visibilityGraphs.postProcessing.ObstacleAvoidanceProcessor;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.Ros2NodeInterface;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class LookAndStepBehaviorUI extends BehaviorUIInterface
{
   public static final BehaviorUIDefinition DEFINITION = new BehaviorUIDefinition(LookAndStepBehavior.DEFINITION, LookAndStepBehaviorUI::new);

   private final LookAndStepBehaviorParameters lookAndStepParameters = new LookAndStepBehaviorParameters();
   private FootstepPlannerParametersBasics footstepPlannerParameters;

   private Messager behaviorMessager;
   private FootstepPlanGraphic footstepPlanGraphic;
   private LivePlanarRegionsGraphic livePlanarRegionsGraphic;
   private PositionGraphic subGoalGraphic;
   private BodyPathPlanGraphic bodyPathPlanGraphic;
   private PoseGraphic goalGraphic;
   
   private VisibilityGraphPathPlanner bodyPathPlanner;

   private SnappedPositionEditor snappedPositionEditor;
   private OrientationYawEditor orientationYawEditor;

   private FXUIActionMap placeGoalActionMap;

   @FXML private Button placeGoalButton;
   @FXML private CheckBox operatorReviewCheckBox;
   @FXML private TextField behaviorState;
   @FXML private TableView lookAndStepParameterTable;
   @FXML private TableView footstepPlannerParameterTable;

   private RemoteREAInterface rea;
   private RemoteHumanoidRobotInterface robot;
   private FramePose3D leftFootPoseTemp = new FramePose3D();
   private FramePose3D rightFootPoseTemp = new FramePose3D();

   @Override
   public void init(SubScene sceneNode, Ros2NodeInterface ros2Node, Messager behaviorMessager, DRCRobotModel robotModel)
   {
      this.behaviorMessager = behaviorMessager;

      footstepPlanGraphic = new FootstepPlanGraphic(robotModel);
      behaviorMessager.registerTopicListener(FootstepPlanForUI, footstepPlanGraphic::generateMeshesAsynchronously);

      livePlanarRegionsGraphic = new LivePlanarRegionsGraphic(false);
      behaviorMessager.registerTopicListener(MapRegionsForUI, livePlanarRegionsGraphic::acceptPlanarRegions);

      goalGraphic = new PoseGraphic("Goal", Color.CADETBLUE, 0.03);
      
      subGoalGraphic = new PositionGraphic(Color.GREEN, 0.02);
      behaviorMessager.registerTopicListener(SubGoalForUI, position -> Platform.runLater(() -> subGoalGraphic.setPosition(position)));

      bodyPathPlanGraphic = new BodyPathPlanGraphic();
      behaviorMessager.registerTopicListener(BodyPathPlanForUI,
                                             bodyPathPlan -> Platform.runLater(() -> bodyPathPlanGraphic.generateMeshesAsynchronously(bodyPathPlan)));

      JavaFXStoredPropertyTable lookAndStepJavaFXStoredPropertyTable = new JavaFXStoredPropertyTable(lookAndStepParameterTable);
      lookAndStepJavaFXStoredPropertyTable.setup(lookAndStepParameters, LookAndStepBehaviorParameters.keys, this::publishLookAndStepParameters);

      footstepPlannerParameters = robotModel.getFootstepPlannerParameters();
      JavaFXStoredPropertyTable footstepPlannerJavaFXStoredPropertyTable = new JavaFXStoredPropertyTable(footstepPlannerParameterTable);
      footstepPlannerJavaFXStoredPropertyTable.setup(footstepPlannerParameters, FootstepPlannerParameterKeys.keys, this::footstepPlanningParameters);

      VisibilityGraphsParametersBasics visibilityGraphParameters = robotModel.getVisibilityGraphsParameters();
      BodyPathPostProcessor pathPostProcessor = new ObstacleAvoidanceProcessor(visibilityGraphParameters);
      bodyPathPlanner = new VisibilityGraphPathPlanner(visibilityGraphParameters, pathPostProcessor,
                                                            new YoVariableRegistry(getClass().getSimpleName()));
      
      rea = new RemoteREAInterface(ros2Node); // maybe instead subscribe to all regions (Visible regions)
      robot = new RemoteHumanoidRobotInterface(ros2Node, robotModel);

      behaviorMessager.registerTopicListener(CurrentState, state -> Platform.runLater(() -> behaviorState.setText(state)));

      snappedPositionEditor = new SnappedPositionEditor(sceneNode);
      orientationYawEditor = new OrientationYawEditor(sceneNode);

      placeGoalActionMap = new FXUIActionMap(startAction ->
      {
         placeGoalButton.setDisable(true);
         snappedPositionEditor.edit(SnappedPositionEditor.EditMode.XY_PLANE, goalGraphic, exitType ->
         {
            placeGoalActionMap.triggerAction(exitType);
         });
      });
      placeGoalActionMap.mapAction(FXUITrigger.POSITION_LEFT_CLICK, trigger ->
      {
         orientationYawEditor.edit(goalGraphic, exitType -> placeGoalActionMap.triggerAction(exitType));
      });
      placeGoalActionMap.mapAction(FXUITrigger.ORIENTATION_LEFT_CLICK, trigger ->
      {
         // calculate and send body path plan
         bodyPathPlanner.setGoal(goalGraphic.getPose());
         bodyPathPlanner.setPlanarRegionsList(rea.getLatestPlanarRegionsList());
         HumanoidRobotState humanoidRobotState = robot.pollHumanoidRobotState();
         leftFootPoseTemp.setToZero(humanoidRobotState.getSoleFrame(RobotSide.LEFT));
         rightFootPoseTemp.setToZero(humanoidRobotState.getSoleFrame(RobotSide.RIGHT));
         leftFootPoseTemp.changeFrame(ReferenceFrame.getWorldFrame());
         rightFootPoseTemp.changeFrame(ReferenceFrame.getWorldFrame());
         bodyPathPlanner.setStanceFootPoses(leftFootPoseTemp, rightFootPoseTemp);
         bodyPathPlanner.planWaypoints();
         ArrayList<Point3D> waypointsAsPoints = new ArrayList<>();
         for (Pose3DReadOnly poseWaypoint : bodyPathPlanner.getWaypoints())
         {
            waypointsAsPoints.add(new Point3D(poseWaypoint.getPosition()));
         }

         behaviorMessager.submitMessage(BodyPathPlanInput, (List<Point3D>) waypointsAsPoints);

         placeGoalButton.setDisable(false);
      });

//      sceneNode.addEventHandler(MouseEvent.MOUSE_CLICKED, this::mouseClicked);

      // TODO Add joystick support
   }

//   private final void mouseClicked(MouseEvent event)
//   {
//      if (!event.isConsumed() && event.isStillSincePress() && BehaviorUI.ACTIVE_EDITOR == null)
//      {
//         PickResult pickResult = event.getPickResult();
//         Node intersectedNode = pickResult.getIntersectedNode();
//      }
//   }

   @Override
   public void setEnabled(boolean enabled)
   {
      if (!enabled)
      {
         livePlanarRegionsGraphic.clear();
         footstepPlanGraphic.clear();
         Platform.runLater(() -> getChildren().remove(subGoalGraphic.getNode()));
         Platform.runLater(() -> getChildren().remove(goalGraphic));
         Platform.runLater(() -> getChildren().remove(bodyPathPlanGraphic));
         Platform.runLater(() -> getChildren().remove(livePlanarRegionsGraphic));
         Platform.runLater(() -> getChildren().remove(footstepPlanGraphic));
      }
      else
      {
         Platform.runLater(() -> getChildren().add(subGoalGraphic.getNode()));
         Platform.runLater(() -> getChildren().add(goalGraphic));
         Platform.runLater(() -> getChildren().add(bodyPathPlanGraphic));
         Platform.runLater(() -> getChildren().add(livePlanarRegionsGraphic));
         Platform.runLater(() -> getChildren().add(footstepPlanGraphic));
      }
   }

   private void publishLookAndStepParameters()
   {
      behaviorMessager.submitMessage(LookAndStepParameters, lookAndStepParameters.getAllAsStrings());
   }

   private void footstepPlanningParameters()
   {
      behaviorMessager.submitMessage(FootstepPlannerParameters, footstepPlannerParameters.getAllAsStrings());
   }

   @FXML public void placeGoalButton()
   {
      placeGoalActionMap.start();
   }

   @FXML public void takeStep()
   {
      behaviorMessager.submitMessage(TakeStep, new Object());
   }

   @FXML public void rePlan()
   {
      behaviorMessager.submitMessage(RePlan, new Object());
   }

   @FXML public void saveLookAndStepParameters()
   {
      lookAndStepParameters.save();
   }

   @FXML public void saveFootstepPlanningParameters()
   {
      footstepPlannerParameters.save();
   }

   @FXML public void operatorReviewCheckBox()
   {
      behaviorMessager.submitMessage(OperatorReviewEnabled, operatorReviewCheckBox.isSelected());
   }
}
