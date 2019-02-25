package us.ihmc.humanoidBehaviors.ui.behaviors;

import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.paint.Color;
import us.ihmc.communication.controllerAPI.RobotLowLevelMessenger;
import us.ihmc.humanoidBehaviors.ui.BehaviorUI;
import us.ihmc.humanoidBehaviors.ui.BehaviorUI.API;
import us.ihmc.humanoidBehaviors.ui.SimpleMessagerAPIFactory;
import us.ihmc.humanoidBehaviors.ui.editors.FXUIEditor;
import us.ihmc.humanoidBehaviors.ui.graphics.FXUIGraphic;
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

   /**
    * Nece
    * @param messager
    */
   public void init(JavaFXMessager messager)
   {
      this.messager = messager;

      waypointPlacementStateMachine = new FXUIStateMachine()
      {
         @Override
         protected void handleTransition(FXUIStateTransition transition)
         {
            if (transition == FXUIStateTransition.BEGIN || transition == FXUIStateTransition.SNAPPED_POSITION_LEFT_CLICK)
            {
               if (transition == FXUIStateTransition.BEGIN)
               {
                  removeAllWaypointGraphics();
               }

               SnappedPositionGraphic waypointGraphic = createWaypointGraphic(messager);
               LogTools.debug("Placing waypoint {}", waypoints.size());
               messager.submitMessage(BehaviorUI.API.ActiveEditor, BehaviorUI.SNAPPED_POSITION_EDITOR);
               messager.submitMessage(BehaviorUI.API.SelectedGraphic, waypointGraphic);
            }
            else if (transition == FXUIStateTransition.SNAPPED_POSITION_RIGHT_CLICK)
            {
               LogTools.debug("Completed waypoint placement.");
               messager.submitMessage(BehaviorUI.API.ActiveEditor, FXUIEditor.NONE);
               messager.submitMessage(BehaviorUI.API.SelectedGraphic, FXUIGraphic.NONE);
            }
         }
      };
      waypointPlacementStateMachine.mapTransitionToState(FXUIStateTransition.BEGIN, FXUIState.SNAPPED_POSITION_EDITOR);
      waypointPlacementStateMachine.mapTransitionToState(FXUIStateTransition.SNAPPED_POSITION_LEFT_CLICK, FXUIState.SNAPPED_POSITION_EDITOR);
      waypointPlacementStateMachine.mapTransitionToState(FXUIStateTransition.SNAPPED_POSITION_RIGHT_CLICK, FXUIState.INACTIVE);
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

   public void setFullRobotModel(FullHumanoidRobotModel fullHumanoidRobotModel)
   {
      this.humanoidReferenceFrames = new HumanoidReferenceFrames(fullHumanoidRobotModel);
   }

   @FXML public void placeWaypoints()
   {
      messager.submitMessage(BehaviorUI.API.ActiveStateMachine, waypointPlacementStateMachine);

      waypointPlacementStateMachine.begin();
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
      private static final SimpleMessagerAPIFactory apiFactory = new SimpleMessagerAPIFactory(BehaviorUI.class);


      public static final MessagerAPI create()
      {
         return apiFactory.getAPIAndCloseFactory();
      }
   }
}
