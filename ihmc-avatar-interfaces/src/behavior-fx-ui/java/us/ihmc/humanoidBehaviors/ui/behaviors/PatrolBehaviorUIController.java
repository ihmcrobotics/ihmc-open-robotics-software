package us.ihmc.humanoidBehaviors.ui.behaviors;

import javafx.application.Platform;
import javafx.fxml.FXML;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.SubScene;
import javafx.scene.control.Button;
import javafx.scene.control.Spinner;
import javafx.scene.control.TextField;
import javafx.scene.input.MouseButton;
import javafx.scene.input.MouseEvent;
import javafx.scene.input.PickResult;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidBehaviors.patrol.PatrolBehavior;
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

   private JavaFXMessager uiMessager;
   private SubScene sceneNode;
   private Messager behaviorMessager;

   private ArrayList<PatrolWaypointGraphic> waypoints = new ArrayList<>();
   private final ExecutorService executorService = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));
   private FootstepPlanGraphic footstepPlanGraphic;

   private SnappedPositionEditor snappedPositionEditor;
   private OrientationYawEditor orientationYawEditor;

   private FXUIStateMachine waypointPlacementStateMachine;

   public void init(SubScene sceneNode, Messager behaviorMessager, DRCRobotModel robotModel)
   {
      this.sceneNode = sceneNode;
      this.behaviorMessager = behaviorMessager;

      footstepPlanGraphic = new FootstepPlanGraphic(robotModel);
      footstepPlanGraphic.start();
      getChildren().add(footstepPlanGraphic.getNode());

      snappedPositionEditor = new SnappedPositionEditor(sceneNode);
      orientationYawEditor = new OrientationYawEditor(sceneNode);

      behaviorMessager.registerTopicListener(PatrolBehavior.API.CurrentFootstepPlan, plan -> {
         executorService.submit(() -> {
            LogTools.debug("Received footstep plan containing {} steps", plan.size());
            footstepPlanGraphic.generateMeshes(plan);
         });
      });

      behaviorMessager.registerTopicListener(PatrolBehavior.API.CurrentState, state -> Platform.runLater(() -> remoteCurrentState.setText(state)));
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
         latestWaypoint.getOrientationGraphic().getArrow().setVisible(true);
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

      sceneNode.addEventHandler(MouseEvent.MOUSE_CLICKED, this::mouseClicked);
   }

   private void goToNextWaypointPositionEdit()
   {
      PatrolWaypointGraphic waypointGraphic = createWaypointGraphic();
      LogTools.debug("Placing waypoint {}", waypoints.size());
      snappedPositionEditor.edit(waypointGraphic, exitType -> waypointPlacementStateMachine.transition(exitType));
   }

   private final void mouseClicked(MouseEvent event)
   {
      if (!event.isConsumed() && event.isStillSincePress() && BehaviorUI.ACTIVE_EDITOR == null)
      {
         if (event.getButton() == MouseButton.PRIMARY)
         {
            LogTools.debug("consume mouseClicked");
            event.consume();
            PickResult pickResult = event.getPickResult();
            Node intersectedNode = pickResult.getIntersectedNode();

            for (int i = 0; i < waypoints.size(); i++)
            {
               if (waypoints.get(i).getSnappedPositionGraphic().getSphere() == intersectedNode)
               {
                  LogTools.debug("Editing patrol waypoint position: {}", i);
                  placeWaypoints.setDisable(true);
                  snappedPositionEditor.edit(waypoints.get(i), exitType ->
                  {
                     teleopUpdateWaypoints();
                     placeWaypoints.setDisable(false);
                  }); // TODO handle right click?
               }
               else if (waypoints.get(i).getOrientationGraphic().getArrow() == intersectedNode)
               {
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

   private void teleopUpdateWaypoints()
   {
      ArrayList<Pose3D> waypointsToSend = new ArrayList<>();
      waypoints.forEach(graphicWaypoint ->
                        {
                           Point3D pos = graphicWaypoint.getSnappedPositionGraphic().getPosition();
                           double yaw = graphicWaypoint.getOrientationGraphic().getYaw();
                           waypointsToSend.add(new Pose3D(pos.getX(), pos.getY(), pos.getZ(), yaw, 0.0, 0.0));
                        });
      behaviorMessager.submitMessage(PatrolBehavior.API.Waypoints, waypointsToSend);
   }

   private PatrolWaypointGraphic createWaypointGraphic()
   {
      PatrolWaypointGraphic waypoint = new PatrolWaypointGraphic();
      getChildren().add(waypoint);
      waypoints.add(waypoint);
      return waypoint;
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
}
