package us.ihmc.humanoidBehaviors.ui.behaviors;

import javafx.fxml.FXML;
import javafx.scene.Node;
import javafx.scene.control.Button;
import javafx.scene.control.Spinner;
import javafx.scene.input.MouseButton;
import javafx.scene.input.MouseEvent;
import javafx.scene.input.PickResult;
import us.ihmc.communication.controllerAPI.RobotLowLevelMessenger;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidBehaviors.BehaviorTeleop;
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
   @FXML private Button placeWaypoints;
   @FXML private Button goToWaypoint;
   @FXML private Spinner<Integer> waypointIndex;
   @FXML private Button pauseWalking;

   private JavaFXMessager messager;
   private BehaviorTeleop teleop;
   private AtomicReference<FXUIEditor> activeEditor;

   private RobotLowLevelMessenger robotLowLevelMessenger;

   private HumanoidReferenceFrames humanoidReferenceFrames;

   private ArrayList<PatrolWaypointGraphic> waypoints = new ArrayList<>();

   private FXUIStateMachine waypointPlacementStateMachine;

   public void init(JavaFXMessager messager, Node sceneNode, BehaviorTeleop teleop)
   {
      this.messager = messager;
      this.teleop = teleop;

      activeEditor = messager.createInput(BehaviorUI.API.ActiveEditor, null);
      messager.registerTopicListener(BehaviorUI.API.ActiveEditor, value ->
      {
         if (value == null)
         {
            teleopUpdateWaypoints();
         }
      });

      waypointPlacementStateMachine = new FXUIStateMachine(messager, FXUIStateTransitionTrigger.RIGHT_CLICK, trigger ->
      {
         removeAllWaypointGraphics();
         goToNextWaypointPositionEdit();
      });
      waypointPlacementStateMachine.mapTransition(FXUIStateTransitionTrigger.POSITION_LEFT_CLICK, trigger ->
      {
         PatrolWaypointGraphic latestWaypoint = waypoints.get(waypoints.size() - 1);
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

         teleopUpdateWaypoints();

         messager.submitMessage(BehaviorUI.API.ActiveEditor, null);
         messager.submitMessage(BehaviorUI.API.SelectedGraphic, null);
      });

      sceneNode.addEventHandler(MouseEvent.MOUSE_CLICKED, this::mouseClicked);
   }

   private void goToNextWaypointPositionEdit()
   {
      PatrolWaypointGraphic waypointGraphic = createWaypointGraphic(messager);
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

   private void teleopUpdateWaypoints()
   {
      ArrayList<Pose3D> waypointsToSend = new ArrayList<>();
      waypoints.forEach(graphicWaypoint ->
                        {
                           Point3D pos = graphicWaypoint.getSnappedPositionGraphic().getPosition();
                           double yaw = graphicWaypoint.getOrientationGraphic().getYaw();
                           waypointsToSend.add(new Pose3D(pos.getX(), pos.getY(), pos.getZ(), yaw, 0.0, 0.0));
                        });
      teleop.setWaypoints(waypointsToSend);
   }

   private PatrolWaypointGraphic createWaypointGraphic(JavaFXMessager messager)
   {
      PatrolWaypointGraphic waypoint = new PatrolWaypointGraphic();
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

   private void removeWaypoint(PatrolWaypointGraphic waypoint)
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

   @FXML public void goToWaypoint()
   {
      teleop.goToWaypoint(waypointIndex.getValue());
   }

   @FXML public void pauseWalking()
   {
      teleop.stopPatrolling();
   }
}
