package us.ihmc.humanoidBehaviors.ui.behaviors;

import javafx.application.Platform;
import javafx.fxml.FXML;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.SubScene;
import javafx.scene.control.Button;
import javafx.scene.control.CheckBox;
import javafx.scene.control.Spinner;
import javafx.scene.control.TextField;
import javafx.scene.input.MouseButton;
import javafx.scene.input.MouseEvent;
import javafx.scene.input.PickResult;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.humanoidBehaviors.patrol.PatrolBehavior;
import us.ihmc.humanoidBehaviors.patrol.PatrolBehavior.API;
import us.ihmc.humanoidBehaviors.patrol.PatrolBehavior.OperatorPlanReviewResult;
import us.ihmc.humanoidBehaviors.patrol.PatrolBehavior.PatrolBehaviorState;
import us.ihmc.humanoidBehaviors.ui.BehaviorUI;
import us.ihmc.humanoidBehaviors.ui.editors.OrientationYawEditor;
import us.ihmc.humanoidBehaviors.ui.editors.SnappedPositionEditor;
import us.ihmc.humanoidBehaviors.ui.graphics.FootstepPlanGraphic;
import us.ihmc.humanoidBehaviors.ui.model.FXUIStateMachine;
import us.ihmc.humanoidBehaviors.ui.model.FXUIStateTransitionTrigger;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;

import java.util.ArrayList;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

public class PatrolBehaviorUIController extends Group
{
   @FXML private Button placeWaypoints;
   @FXML private Button goToWaypoint;
   @FXML private Spinner<Integer> waypointIndex;
   @FXML private Button pauseWalking;
   @FXML private TextField remoteCurrentWaypointIndex;
   @FXML private TextField remoteCurrentState;
   @FXML private CheckBox loopThroughWaypoints;
   @FXML private CheckBox swingOverPlanarRegions;
   @FXML private CheckBox operatorPlanReview;
   @FXML private CheckBox upDownExploration;
   @FXML private Button replan;
   @FXML private Button sendPlan;

   private JavaFXMessager uiMessager;
   private SubScene sceneNode;
   private Messager behaviorMessager;

   private ArrayList<PatrolWaypointGraphic> waypoints = new ArrayList<>();
   private final ExecutorService executorService = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));
   private FootstepPlanGraphic footstepPlanGraphic;

   private SnappedPositionEditor snappedPositionEditor;
   private OrientationYawEditor orientationYawEditor;

   private FXUIStateMachine waypointPlacementStateMachine;
   private FXUIStateMachine waypointInsertStateMachine;
   private int currentInsertIndex;

   public void init(SubScene sceneNode, Messager behaviorMessager, DRCRobotModel robotModel)
   {
      this.sceneNode = sceneNode;
      this.behaviorMessager = behaviorMessager;

      footstepPlanGraphic = new FootstepPlanGraphic(robotModel);
      getChildren().add(footstepPlanGraphic);

      snappedPositionEditor = new SnappedPositionEditor(sceneNode);
      orientationYawEditor = new OrientationYawEditor(sceneNode);

      behaviorMessager.registerTopicListener(PatrolBehavior.API.CurrentFootstepPlan, plan -> {
         executorService.submit(() -> {
            LogTools.debug("Received footstep plan containing {} steps", plan.size());
            footstepPlanGraphic.generateMeshes(plan);
         });
      });

      Platform.runLater(() ->
      {
         replan.setDisable(true);
         sendPlan.setDisable(true);
      });
      behaviorMessager.registerTopicListener(PatrolBehavior.API.CurrentState, state -> Platform.runLater(() ->
      {
         remoteCurrentState.setText(state.name());
         if (state == PatrolBehaviorState.REVIEW && operatorPlanReview.isSelected())
         {
            replan.setDisable(false);
            sendPlan.setDisable(false);
         }
         else
         {
            replan.setDisable(true);
            sendPlan.setDisable(true);
         }
      }));
      behaviorMessager.registerTopicListener(PatrolBehavior.API.CurrentWaypointIndexStatus,
                                             index -> Platform.runLater(() -> remoteCurrentWaypointIndex.setText(index.toString())));

      waypointPlacementStateMachine = new FXUIStateMachine(uiMessager, FXUIStateTransitionTrigger.RIGHT_CLICK, trigger ->
      {
         placeWaypoints.setDisable(true);
         removeAllWaypointGraphics();
         goToNextWaypointPositionEdit();
      });
      waypointPlacementStateMachine.mapTransition(FXUIStateTransitionTrigger.POSITION_LEFT_CLICK, trigger ->
      {
         PatrolWaypointGraphic latestWaypoint = waypoints.get(waypoints.size() - 1);
         latestWaypoint.getOrientationGraphic().getNode().setVisible(true);
         orientationYawEditor.edit(latestWaypoint, exitType -> waypointPlacementStateMachine.transition(exitType));
      });
      waypointPlacementStateMachine.mapTransition(FXUIStateTransitionTrigger.ORIENTATION_LEFT_CLICK, trigger ->
      {
         goToNextWaypointPositionEdit();
      });
      waypointPlacementStateMachine.mapTransition(FXUIStateTransitionTrigger.RIGHT_CLICK, trigger ->
      {
         LogTools.debug("Completed waypoint placement.");
         removeWaypoint(waypoints.get(waypoints.size() - 1));
         teleopUpdateWaypoints();
         placeWaypoints.setDisable(false);
      });

      waypointInsertStateMachine = new FXUIStateMachine(uiMessager, FXUIStateTransitionTrigger.RIGHT_CLICK, trigger ->
      {
         placeWaypoints.setDisable(true);
         goToNextInsertedWaypointPositionEdit();
      });
      waypointInsertStateMachine.mapTransition(FXUIStateTransitionTrigger.POSITION_LEFT_CLICK, trigger ->
      {
         PatrolWaypointGraphic latestWaypoint = waypoints.get(currentInsertIndex);
         latestWaypoint.getOrientationGraphic().getNode().setVisible(true);
         orientationYawEditor.edit(latestWaypoint, exitType -> waypointInsertStateMachine.transition(exitType));
      });
      waypointInsertStateMachine.mapTransition(FXUIStateTransitionTrigger.ORIENTATION_LEFT_CLICK, trigger ->
      {
         goToNextInsertedWaypointPositionEdit();
      });
      waypointInsertStateMachine.mapTransition(FXUIStateTransitionTrigger.RIGHT_CLICK, trigger ->
      {
         LogTools.debug("Completed waypoint insertion.");
         removeWaypointGraphic(currentInsertIndex);
         teleopUpdateWaypoints();
         placeWaypoints.setDisable(false);
      });

      sceneNode.addEventHandler(MouseEvent.MOUSE_CLICKED, this::mouseClicked);
   }

   private void goToNextWaypointPositionEdit()
   {
      PatrolWaypointGraphic waypointGraphic = createWaypointGraphic();
      LogTools.debug("Placing waypoint {}", waypoints.size());
      waypoints.forEach(waypoint -> waypoint.setMouseTransparent(true));
      snappedPositionEditor.edit(waypointGraphic, exitType ->
      {
         waypointPlacementStateMachine.transition(exitType);
         waypoints.forEach(waypoint -> waypoint.setMouseTransparent(false));
      });
   }

   private void goToNextInsertedWaypointPositionEdit()
   {
      currentInsertIndex++;
      waypoints.forEach(waypoint -> waypoint.setMouseTransparent(true));
      PatrolWaypointGraphic insertedWaypoint = insertWaypointGraphic(currentInsertIndex);
      LogTools.debug("Inserting waypoint {}", waypoints.size());
      waypoints.forEach(waypoint -> waypoint.setMouseTransparent(true));
      snappedPositionEditor.edit(insertedWaypoint, exitType ->
      {
         waypointInsertStateMachine.transition(exitType);
         waypoints.forEach(waypoint -> waypoint.setMouseTransparent(false));
      });
   }

   private final void mouseClicked(MouseEvent event)
   {
      if (!event.isConsumed() && event.isStillSincePress() && BehaviorUI.ACTIVE_EDITOR == null)
      {
         PickResult pickResult = event.getPickResult();
         Node intersectedNode = pickResult.getIntersectedNode();

         for (int i = 0; i < waypoints.size(); i++) // edit
         {
            if (waypoints.get(i).getSnappedPositionGraphic().getNode() == intersectedNode)
            {
               if (event.getButton() == MouseButton.PRIMARY)
               {
                  event.consume();
                  LogTools.debug("Editing patrol waypoint position: {}", i);
                  placeWaypoints.setDisable(true);
                  waypoints.forEach(waypoint -> waypoint.setMouseTransparent(true));
                  editWaypoint(waypoints.get(i));
               }
               else if (event.getButton() == MouseButton.SECONDARY) // insert
               {
                  event.consume();
                  LogTools.debug("Inserting patrol waypoint position: {}", i);
                  currentInsertIndex = i; // TODO avoid this global by extracting waypoint management
                  waypointInsertStateMachine.start();
               }
               else if (event.getButton() == MouseButton.MIDDLE) // delete
               {
                  event.consume();
                  LogTools.debug("Deleting patrol waypoint position: {}", i);
                  removeWaypointGraphic(i);
                  teleopUpdateWaypoints();
               }
            }
            else if (waypoints.get(i).getOrientationGraphic().getNode() == intersectedNode)
            {
               if (event.getButton() == MouseButton.PRIMARY) // edit
               {
                  event.consume();
                  LogTools.debug("Editing patrol waypoint orientation: {}", i);
                  placeWaypoints.setDisable(true);
                  orientationYawEditor.edit(waypoints.get(i), exitType ->
                  {
                     teleopUpdateWaypoints();
                     placeWaypoints.setDisable(false);
                  });
               }
            }
         }
      }
   }

   private void editWaypoint(PatrolWaypointGraphic waypointToEdit)
   {
      snappedPositionEditor.edit(waypointToEdit, exitType ->
      {
         teleopUpdateWaypoints();
         placeWaypoints.setDisable(false);
         waypoints.forEach(waypoint -> waypoint.setMouseTransparent(false));
      }); // TODO handle abort placement?
   }

   private void teleopUpdateWaypoints()
   {
      ArrayList<Pose3D> waypointsToSend = new ArrayList<>();
      waypoints.forEach(graphicWaypoint ->
                        {
                           Point3DReadOnly pos = graphicWaypoint.getSnappedPositionGraphic().getPose().getPosition();
                           double yaw = graphicWaypoint.getOrientationGraphic().getYaw();
                           waypointsToSend.add(new Pose3D(pos.getX(), pos.getY(), pos.getZ(), yaw, 0.0, 0.0));
                        });
      behaviorMessager.submitMessage(PatrolBehavior.API.Waypoints, waypointsToSend);
   }

   private PatrolWaypointGraphic createWaypointGraphic()
   {
      PatrolWaypointGraphic waypoint = new PatrolWaypointGraphic(waypoints.size());
      getChildren().add(waypoint);
      waypoints.add(waypoint);
      return waypoint;
   }

   private PatrolWaypointGraphic insertWaypointGraphic(int index)
   {
      PatrolWaypointGraphic waypoint = new PatrolWaypointGraphic(index);
      getChildren().add(waypoint);
      waypoints.add(index, waypoint);
      for (int i = 0; i < waypoints.size(); i++) // redraw numbers
      {
         waypoints.get(i).setIndex(i);
      }
      return waypoint;
   }

   private void removeWaypointGraphic(int index)
   {
      getChildren().remove(waypoints.get(index));
      waypoints.remove(index);
      for (int i = 0; i < waypoints.size(); i++) // redraw numbers
      {
         waypoints.get(i).setIndex(i);
      }
   }

   private void removeAllWaypointGraphics()
   {
      LogTools.debug("Removing all waypoint graphics.");
      waypoints.forEach(waypoint -> {
         getChildren().remove(waypoint);
      });
      waypoints.clear();
   }

   private void removeWaypoint(PatrolWaypointGraphic waypoint)
   {
      getChildren().remove(waypoint);
      waypoints.remove(waypoint);
   }

   @FXML
   public void placeWaypoints()
   {
      waypointPlacementStateMachine.start();
   }

   @FXML public void goToWaypoint()
   {
      behaviorMessager.submitMessage(PatrolBehavior.API.GoToWaypoint, waypointIndex.getValue());
   }

   @FXML public void stopWalking()
   {
      behaviorMessager.submitMessage(PatrolBehavior.API.Stop, new Object());
   }

   @FXML public void loopThroughWaypoints()
   {
      behaviorMessager.submitMessage(PatrolBehavior.API.Loop, loopThroughWaypoints.isSelected());
   }

   @FXML public void swingOverPlanarRegions()
   {
      behaviorMessager.submitMessage(PatrolBehavior.API.SwingOvers, swingOverPlanarRegions.isSelected());
   }

   @FXML public void operatorPlanReview()
   {
      behaviorMessager.submitMessage(PatrolBehavior.API.PlanReviewEnabled, operatorPlanReview.isSelected());
   }

   @FXML public void replan()
   {
      behaviorMessager.submitMessage(PatrolBehavior.API.PlanReviewResult, OperatorPlanReviewResult.REPLAN);
   }

   @FXML public void sendPlan()
   {
      behaviorMessager.submitMessage(PatrolBehavior.API.PlanReviewResult, OperatorPlanReviewResult.WALK);
   }

   @FXML public void upDownExploration()
   {
      placeWaypoints.setDisable(!upDownExploration.isSelected());
      removeAllWaypointGraphics();
      teleopUpdateWaypoints();
      behaviorMessager.submitMessage(API.UpDownExplorationEnabled, upDownExploration.isSelected());
   }
}
