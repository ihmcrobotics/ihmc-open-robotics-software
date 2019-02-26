package us.ihmc.humanoidBehaviors.ui.behaviors;

import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.paint.Color;
import us.ihmc.communication.controllerAPI.RobotLowLevelMessenger;
import us.ihmc.humanoidBehaviors.ui.BehaviorUI;
import us.ihmc.humanoidBehaviors.ui.model.FXUIMessagerAPIFactory;
import us.ihmc.humanoidBehaviors.ui.model.*;
import us.ihmc.humanoidBehaviors.ui.graphics.SnappedPositionGraphic;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.log.LogTools;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.robotModels.FullHumanoidRobotModel;

import java.util.ArrayList;

public class PatrolBehaviorUIController extends FXUIBehavior
{
   @FXML private Button continuePatrol;
   @FXML private Button pausePatrol;
   @FXML private Button destroyPatrol;
   @FXML private Button placeWaypoints;

   private JavaFXMessager messager;
   private RobotLowLevelMessenger robotLowLevelMessenger;

   private HumanoidReferenceFrames humanoidReferenceFrames;

   private ArrayList<SnappedPositionGraphic> waypoints = new ArrayList<>();

   private FXUIStateMachine waypointPlacementStateMachine;

   public void init(JavaFXMessager messager)
   {
      this.messager = messager;

      waypointPlacementStateMachine = new FXUIStateMachine(messager,
                                                           FXUIState.SNAPPED_POSITION_EDITOR,
                                                           FXUIStateTransition.SNAPPED_POSITION_RIGHT_CLICK)
      {
         @Override
         protected void handleTransition(FXUIStateTransition transition)
         {
            if (transition.isStart() || transition == FXUIStateTransition.SNAPPED_POSITION_LEFT_CLICK)
            {
               if (transition.isStart())
               {
                  removeAllWaypointGraphics();
               }

               SnappedPositionGraphic waypointGraphic = createWaypointGraphic(messager);
               LogTools.debug("Placing waypoint {}", waypoints.size());
               messager.submitMessage(BehaviorUI.API.ActiveEditor, BehaviorUI.SNAPPED_POSITION_EDITOR);
               messager.submitMessage(BehaviorUI.API.SelectedGraphic, waypointGraphic.getSphere());
            }
            else if (transition == FXUIStateTransition.SNAPPED_POSITION_RIGHT_CLICK)
            {
               LogTools.debug("Completed waypoint placement.");

               removeWaypointGraphic(waypoints.get(waypoints.size() - 1));

               messager.submitMessage(BehaviorUI.API.ActiveEditor, null);
               messager.submitMessage(BehaviorUI.API.SelectedGraphic, null);
            }
         }
      };
      waypointPlacementStateMachine.mapTransitionToState(FXUIStateTransition.SNAPPED_POSITION_LEFT_CLICK, FXUIState.SNAPPED_POSITION_EDITOR);
   }

   private SnappedPositionGraphic createWaypointGraphic(JavaFXMessager messager)
   {
      SnappedPositionGraphic waypointGraphic = new SnappedPositionGraphic(messager, Color.YELLOW);
      registerGraphic(waypointGraphic);
      waypoints.add(waypointGraphic);
      return waypointGraphic;
   }

   private void removeAllWaypointGraphics()
   {
      LogTools.debug("Removing all waypoint graphics.");
      waypoints.forEach(this::removeGraphic);
      waypoints.clear();
   }

   private void removeWaypointGraphic(SnappedPositionGraphic waypointGraphic)
   {
      removeGraphic(waypointGraphic);
      waypoints.remove(waypointGraphic);
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

   public static class API
   {
      private static final FXUIMessagerAPIFactory apiFactory = new FXUIMessagerAPIFactory(BehaviorUI.class);


      public static final MessagerAPI create()
      {
         return apiFactory.getAPIAndCloseFactory();
      }
   }
}
