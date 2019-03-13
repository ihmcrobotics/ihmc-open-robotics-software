package us.ihmc.humanoidBehaviors.ui.behaviors;

import javafx.fxml.FXML;
import javafx.scene.Node;
import javafx.scene.control.Button;
import javafx.scene.input.MouseButton;
import javafx.scene.input.MouseEvent;
import javafx.scene.input.PickResult;
import us.ihmc.communication.controllerAPI.RobotLowLevelMessenger;
import us.ihmc.humanoidBehaviors.ui.BehaviorUI;
import us.ihmc.humanoidBehaviors.ui.model.*;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.log.LogTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;

public class PatrolBehaviorUIController extends FXUIBehavior
{
   @FXML private Button continuePatrol;
   @FXML private Button pausePatrol;
   @FXML private Button destroyPatrol;
   @FXML private Button placeWaypoints;

   private JavaFXMessager messager;
   private AtomicReference<FXUIEditor> activeEditor;

   private RobotLowLevelMessenger robotLowLevelMessenger;

   private HumanoidReferenceFrames humanoidReferenceFrames;

   private ArrayList<PatrolWaypoint> waypoints = new ArrayList<>();

   private FXUIStateMachine waypointPlacementStateMachine;

   public void init(JavaFXMessager messager, Node sceneNode)
   {
      this.messager = messager;

      activeEditor = messager.createInput(BehaviorUI.API.ActiveEditor, null);

      waypointPlacementStateMachine = new FXUIStateMachine(messager, FXUIStateTransitionTrigger.RIGHT_CLICK, trigger ->
      {
         removeAllWaypointGraphics();
         goToNextWaypointPositionEdit();
      });
      waypointPlacementStateMachine.mapTransition(FXUIStateTransitionTrigger.POSITION_LEFT_CLICK, trigger ->
      {
         PatrolWaypoint latestWaypoint = waypoints.get(waypoints.size() - 1);
         latestWaypoint.getOrientationGraphic().getArrow().setVisible(true);
         messager.submitMessage(BehaviorUI.API.ActiveEditor, BehaviorUI.ORIENTATION_EDITOR);
         messager.submitMessage(BehaviorUI.API.SelectedGraphic, latestWaypoint);
      });
      waypointPlacementStateMachine.mapTransition(FXUIStateTransitionTrigger.ORIENTATION_LEFT_CLICK, trigger ->
      {
         goToNextWaypointPositionEdit();
      });
      waypointPlacementStateMachine.mapTransition(FXUIStateTransitionTrigger.RIGHT_CLICK, trigger ->
      {
         LogTools.debug("Completed waypoint placement.");

         removeWaypoint(waypoints.get(waypoints.size() - 1));

         messager.submitMessage(BehaviorUI.API.ActiveEditor, null);
         messager.submitMessage(BehaviorUI.API.SelectedGraphic, null);
      });

      sceneNode.addEventHandler(MouseEvent.MOUSE_CLICKED, this::mouseClicked);
   }

   private void goToNextWaypointPositionEdit()
   {
      PatrolWaypoint waypointGraphic = createWaypointGraphic(messager);
      LogTools.debug("Placing waypoint {}", waypoints.size());
      messager.submitMessage(BehaviorUI.API.ActiveEditor, BehaviorUI.SNAPPED_POSITION_EDITOR);
      messager.submitMessage(BehaviorUI.API.SelectedGraphic, waypointGraphic);
   }

   private final void mouseClicked(MouseEvent event)
   {
      if (event.isStillSincePress())
      {
         LogTools.debug("{} mouseClicked", getClass().getSimpleName());
         if (activeEditor.get() == null)
         {
            if (event.getButton() == MouseButton.PRIMARY)
            {
               PickResult pickResult = event.getPickResult();
               Node intersectedNode = pickResult.getIntersectedNode();


               for (int i = 0; i < waypoints.size(); i++)
               {
                  if (waypoints.get(i).getSnappedPositionGraphic().getSphere() == intersectedNode)
                  {
                     LogTools.debug("Editing patrol waypoint position: {}", i);
                     messager.submitMessage(BehaviorUI.API.SelectedGraphic, waypoints.get(i));
                     BehaviorUI.SNAPPED_POSITION_EDITOR.activate();
                  }
                  else if (waypoints.get(i).getOrientationGraphic().getArrow() == intersectedNode)
                  {
                     LogTools.debug("Editing patrol waypoint orientation: {}", i);
                     messager.submitMessage(BehaviorUI.API.SelectedGraphic, waypoints.get(i));
                     BehaviorUI.ORIENTATION_EDITOR.activate();
                  }
               }
            }
         }
      }
   }

   private PatrolWaypoint createWaypointGraphic(JavaFXMessager messager)
   {
      PatrolWaypoint waypoint = new PatrolWaypoint();
      registerGraphic(waypoint);
      waypoints.add(waypoint);
      return waypoint;
   }

   private void removeAllWaypointGraphics()
   {
      LogTools.debug("Removing all waypoint graphics.");
      waypoints.forEach(waypoint -> {
         removeGraphic(waypoint);
      });
      waypoints.clear();
   }

   private void removeWaypoint(PatrolWaypoint waypoint)
   {
      removeGraphic(waypoint);
      waypoints.remove(waypoint);
   }

   public void setFullRobotModel(FullHumanoidRobotModel fullHumanoidRobotModel)
   {
      this.humanoidReferenceFrames = new HumanoidReferenceFrames(fullHumanoidRobotModel);
   }

   @FXML public void placeWaypoints()
   {
      messager.submitMessage(BehaviorUI.API.ActiveStateMachine, waypointPlacementStateMachine);

      waypointPlacementStateMachine.start();
   }

   @FXML public void continuePatrol()
   {

   }

   @FXML public void pausePatrol()
   {

   }

   @FXML public void destroyPatrol()
   {

   }
}
