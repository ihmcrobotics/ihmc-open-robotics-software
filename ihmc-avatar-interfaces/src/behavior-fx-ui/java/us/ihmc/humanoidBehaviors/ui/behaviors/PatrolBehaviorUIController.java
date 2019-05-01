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
import us.ihmc.humanoidBehaviors.waypoints.Waypoint;
import us.ihmc.humanoidBehaviors.waypoints.WaypointManager;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;

import java.util.ArrayList;
import java.util.HashMap;
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

   private WaypointManager waypointManager;
//   private ArrayList<PatrolWaypointGraphic> waypointGraphics = new ArrayList<>();
   private HashMap<Long, PatrolWaypointGraphic> waypointGraphics = new HashMap<>(); // map unique id to graphic
   private final ExecutorService executorService = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));
   private FootstepPlanGraphic footstepPlanGraphic;

   private SnappedPositionEditor snappedPositionEditor;
   private OrientationYawEditor orientationYawEditor;

   private FXUIStateMachine waypointPlacementStateMachine;
   private FXUIStateMachine waypointInsertStateMachine;

   private int currentInsertIndex; // TODO this should be in extracted functionality

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
         goToWaypoint.setDisable(true);
         waypointIndex.getValueFactory().valueProperty().setValue(-1);
         waypointIndex.setDisable(true);
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

      waypointManager = WaypointManager.createForUI(behaviorMessager, API.WaypointsToUI, API.WaypointsToModule, () ->
      {
         Platform.runLater(() ->
         {
            updateUIFromWaypointManager();

            // update waypoint graphics from manager
            for (Long waypointId : waypointGraphics.keySet())
            {
               waypointGraphics.get(waypointId).setPosition(waypointManager.getPoseFromId(waypointId).getPosition());
               waypointGraphics.get(waypointId).setOrientation(waypointManager.getPoseFromId(waypointId).getOrientation());
            }
         });
      });

      behaviorMessager.registerTopicListener(PatrolBehavior.API.CurrentWaypointIndexStatus,
                                             index ->
                                             {
                                                Platform.runLater(() ->
                                                                  {
                                                                     remoteCurrentWaypointIndex.setText(index.toString());
                                                                     waypointIndex.getValueFactory().valueProperty().setValue(index);
                                                                  });
                                             });

      waypointPlacementStateMachine = new FXUIStateMachine(uiMessager, FXUIStateTransitionTrigger.RIGHT_CLICK, trigger ->
      {
         placeWaypoints.setDisable(true);
         removeAllWaypointGraphics();
         goToNextWaypointPositionEdit();
      });
      waypointPlacementStateMachine.mapTransition(FXUIStateTransitionTrigger.POSITION_LEFT_CLICK, trigger ->
      {
         PatrolWaypointGraphic latestWaypoint = waypointGraphics.get(waypointManager.lastId());
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
         removeLastWaypoint();
         publishWaypointsToModule();
         placeWaypoints.setDisable(false);
      });

      waypointInsertStateMachine = new FXUIStateMachine(uiMessager, FXUIStateTransitionTrigger.RIGHT_CLICK, trigger ->
      {
         placeWaypoints.setDisable(true);
         goToNextInsertedWaypointPositionEdit();
      });
      waypointInsertStateMachine.mapTransition(FXUIStateTransitionTrigger.POSITION_LEFT_CLICK, trigger ->
      {
         PatrolWaypointGraphic latestWaypoint = waypointGraphics.get(waypointManager.idFromIndex(currentInsertIndex));
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
         publishWaypointsToModule();
         placeWaypoints.setDisable(false);
      });

      sceneNode.addEventHandler(MouseEvent.MOUSE_CLICKED, this::mouseClicked);
   }

   private void goToNextWaypointPositionEdit()
   {
      PatrolWaypointGraphic waypointGraphic = createWaypointGraphic();
      LogTools.debug("Placing waypoint {}", waypointManager.size());
      waypointGraphics.values().forEach(waypoint -> waypoint.setMouseTransparent(true));
      snappedPositionEditor.edit(waypointGraphic, exitType ->
      {
         waypointPlacementStateMachine.transition(exitType);
         waypointGraphics.values().forEach(waypoint -> waypoint.setMouseTransparent(false));
      });
   }

   private void goToNextInsertedWaypointPositionEdit()
   {
      currentInsertIndex++;
      waypointGraphics.values().forEach(waypoint -> waypoint.setMouseTransparent(true));
      PatrolWaypointGraphic insertedWaypoint = insertWaypointGraphic(currentInsertIndex);
      LogTools.debug("Inserting waypoint {}", waypointManager.size());
      waypointGraphics.values().forEach(waypoint -> waypoint.setMouseTransparent(true));
      snappedPositionEditor.edit(insertedWaypoint, exitType ->
      {
         waypointInsertStateMachine.transition(exitType);
         waypointGraphics.values().forEach(waypoint -> waypoint.setMouseTransparent(false));
      });
   }

   private final void mouseClicked(MouseEvent event)
   {
      if (!event.isConsumed() && event.isStillSincePress() && BehaviorUI.ACTIVE_EDITOR == null)
      {
         PickResult pickResult = event.getPickResult();
         Node intersectedNode = pickResult.getIntersectedNode();

         for (Long id : new ArrayList<>(waypointGraphics.keySet()))
         {
            if (waypointGraphics.get(id).getSnappedPositionGraphic().getNode() == intersectedNode)
            {
               if (event.getButton() == MouseButton.PRIMARY)
               {
                  event.consume();
                  LogTools.debug("Editing patrol waypoint position: {}", waypointManager.indexOfId(id));
                  placeWaypoints.setDisable(true);
                  waypointGraphics.values().forEach(waypoint -> waypoint.setMouseTransparent(true));
                  editWaypoint(waypointGraphics.get(id));
               }
               else if (event.getButton() == MouseButton.SECONDARY) // insert
               {
                  event.consume();
                  int insertIndex = waypointManager.indexOfId(id);
                  LogTools.debug("Inserting patrol waypoint position: {}", insertIndex);
                  currentInsertIndex = insertIndex; // TODO avoid this global by extracting waypoint management
                  waypointInsertStateMachine.start();
               }
               else if (event.getButton() == MouseButton.MIDDLE) // delete
               {
                  event.consume();
                  LogTools.debug("Deleting patrol waypoint position: {}", waypointManager.indexOfId(id));
                  removeWaypointGraphicById(id);
                  publishWaypointsToModule();
               }
            }
            else if (waypointGraphics.get(id).getOrientationGraphic().getNode() == intersectedNode)
            {
               if (event.getButton() == MouseButton.PRIMARY) // edit
               {
                  event.consume();
                  LogTools.debug("Editing patrol waypoint orientation: {}", waypointManager.indexOfId(id));
                  placeWaypoints.setDisable(true);
                  orientationYawEditor.edit(waypointGraphics.get(id), exitType ->
                  {
                     publishWaypointsToModule();
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
         publishWaypointsToModule();
         placeWaypoints.setDisable(false);
         waypointGraphics.values().forEach(waypoint -> waypoint.setMouseTransparent(false));
      }); // TODO handle abort placement?
   }

   /** Should only be called from JavaFX thread */
   private void publishWaypointsToModule()
   {
      // update manager from graphics
      for (Long waypointId : waypointGraphics.keySet())
      {
         waypointManager.getPoseFromId(waypointId).setPosition(waypointGraphics.get(waypointId).getPosition());
         waypointManager.getPoseFromId(waypointId).setOrientation(waypointGraphics.get(waypointId).getOrientation());
      }

      updateUIFromWaypointManager();

      waypointManager.publish();
   }

   private void updateUIFromWaypointManager()
   {
      goToWaypoint.setDisable(!waypointManager.hasWaypoints());
      waypointIndex.setDisable(!waypointManager.hasWaypoints());
      waypointIndex.getValueFactory().valueProperty().setValue(waypointManager.peekNextIndex());
      waypointIndex.getValueFactory().setValue(waypointManager.peekNextIndex());
   }

   private PatrolWaypointGraphic createWaypointGraphic()
   {
      Waypoint createdWaypoint = waypointManager.appendNewWaypoint();
      PatrolWaypointGraphic createdWaypointGraphic = new PatrolWaypointGraphic(waypointManager.indexOfId(createdWaypoint.getUniqueId()));
      waypointGraphics.put(createdWaypoint.getUniqueId(), createdWaypointGraphic);
      getChildren().add(createdWaypointGraphic);
      return createdWaypointGraphic;
   }

   private PatrolWaypointGraphic insertWaypointGraphic(int index)
   {
      Waypoint insertedWaypoint = waypointManager.insertNewWaypoint(index);
      PatrolWaypointGraphic insertedWaypointGraphic = new PatrolWaypointGraphic(index);
      waypointGraphics.put(insertedWaypoint.getUniqueId(), insertedWaypointGraphic);
      getChildren().add(insertedWaypointGraphic);
      redrawIndexNumbers();
      return insertedWaypointGraphic;
   }

   private void removeWaypointGraphic(int index)
   {
      removeWaypointGraphicById(waypointManager.idFromIndex(index));
   }

   private void removeWaypointGraphicById(long id)
   {
      getChildren().remove(waypointGraphics.get(id));
      waypointGraphics.remove(id);
      waypointManager.remove(id);
      redrawIndexNumbers();
   }

   private void redrawIndexNumbers()
   {
      for (Long id : waypointGraphics.keySet())
      {
         waypointGraphics.get(id).redrawIndex(waypointManager.indexOfId(id));
      }
   }

   private void removeAllWaypointGraphics()
   {
      LogTools.debug("Removing all waypoint graphics.");
      waypointGraphics.values().forEach(graphic -> getChildren().remove(graphic));
      waypointGraphics.clear();
      waypointManager.clearWaypoints();
   }

   private void removeLastWaypoint()
   {
      long lastId = waypointManager.lastId();

      getChildren().remove(waypointGraphics.get(lastId));
      waypointGraphics.remove(lastId);
      waypointManager.remove(lastId);
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
      placeWaypoints.setDisable(upDownExploration.isSelected());
      removeAllWaypointGraphics();
      publishWaypointsToModule();
      behaviorMessager.submitMessage(API.UpDownExplorationEnabled, upDownExploration.isSelected());
   }
}
