package us.ihmc.humanoidBehaviors.ui.behaviors;

import javafx.application.Platform;
import javafx.fxml.FXML;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.SubScene;
import javafx.scene.control.Button;
import javafx.scene.control.CheckBox;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory.DoubleSpinnerValueFactory;
import javafx.scene.control.SpinnerValueFactory.IntegerSpinnerValueFactory;
import javafx.scene.control.TextField;
import javafx.scene.input.MouseButton;
import javafx.scene.input.MouseEvent;
import javafx.scene.input.PickResult;
import javafx.scene.paint.Color;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidBehaviors.patrol.PatrolBehavior.OperatorPlanReviewResult;
import us.ihmc.humanoidBehaviors.patrol.PatrolBehavior.PatrolBehaviorState;
import us.ihmc.humanoidBehaviors.tools.footstepPlanner.RemoteFootstepPlannerInterface;
import us.ihmc.humanoidBehaviors.ui.BehaviorUI;
import us.ihmc.humanoidBehaviors.ui.editors.OrientationYawEditor;
import us.ihmc.humanoidBehaviors.ui.editors.SnappedPositionEditor;
import us.ihmc.humanoidBehaviors.ui.editors.SnappedPositionEditor.EditMode;
import us.ihmc.humanoidBehaviors.ui.graphics.FootstepPlanGraphic;
import us.ihmc.humanoidBehaviors.ui.graphics.PositionGraphic;
import us.ihmc.humanoidBehaviors.ui.graphics.UpDownGoalGraphic;
import us.ihmc.humanoidBehaviors.ui.model.FXUIActionMap;
import us.ihmc.humanoidBehaviors.ui.model.FXUITrigger;
import us.ihmc.humanoidBehaviors.waypoints.Waypoint;
import us.ihmc.humanoidBehaviors.waypoints.WaypointManager;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;

import java.util.ArrayList;
import java.util.HashMap;

import static us.ihmc.humanoidBehaviors.patrol.PatrolBehaviorAPI.*;

public class PatrolBehaviorUIController extends Group
{
   @FXML private Button placeWaypoints;
   @FXML private Button goToWaypoint;
   @FXML private Button cancelPlanning2;
   @FXML private Button skip;
   @FXML private Spinner<Integer> waypointIndex;
   @FXML private TextField remoteCurrentWaypointIndex;
   @FXML private TextField remoteCurrentState;
   @FXML private CheckBox loopThroughWaypoints;
   @FXML private CheckBox swingOverPlanarRegions;
   @FXML private CheckBox operatorPlanReview;
   @FXML private CheckBox upDownExploration;
   @FXML private Button placeUpDownCenter;
   @FXML private Spinner<Double> perceiveDuration;
   @FXML private Button replan;
   @FXML private Button sendPlan;

   private Messager behaviorMessager;

   private WaypointManager waypointManager;
   private HashMap<Long, PatrolWaypointGraphic> waypointGraphics = new HashMap<>(); // map unique id to graphic
   private FootstepPlanGraphic footstepPlanGraphic;
   private UpDownGoalGraphic upDownGoalGraphic;
   private PositionGraphic upDownCenterGraphic = new PositionGraphic(Color.AZURE, 0.03);

   private SnappedPositionEditor snappedPositionEditor;
   private OrientationYawEditor orientationYawEditor;

   private FXUIActionMap waypointPlacementActionMap;
   private FXUIActionMap waypointInsertActionMap;
   private FXUIActionMap upDownCenterPlacementActionMap;

   private int currentInsertIndex; // TODO this should be in extracted functionality

   public void init(SubScene sceneNode, Messager behaviorMessager, DRCRobotModel robotModel)
   {
      this.behaviorMessager = behaviorMessager;

      footstepPlanGraphic = new FootstepPlanGraphic(robotModel);
      getChildren().add(footstepPlanGraphic);

      upDownGoalGraphic = new UpDownGoalGraphic();
      getChildren().addAll(upDownGoalGraphic.getNodes());
      getChildren().add(upDownCenterGraphic.getNode());

      snappedPositionEditor = new SnappedPositionEditor(sceneNode);
      orientationYawEditor = new OrientationYawEditor(sceneNode);

      behaviorMessager.registerTopicListener(CurrentFootstepPlan, footstepPlanGraphic::generateMeshesAsynchronously);
      behaviorMessager.registerTopicListener(UpDownGoalPoses, result -> Platform.runLater(() -> upDownGoalGraphic.setResult(result)));

      Platform.runLater(() ->
      {
         replan.setDisable(true);
         sendPlan.setDisable(true);
         cancelPlanning2.setDisable(true);
         skip.setDisable(true);
         goToWaypoint.setDisable(true);
         waypointIndex.setValueFactory(new IntegerSpinnerValueFactory(-1, -1, -1, 1));
         waypointIndex.getValueFactory().valueProperty().setValue(-1);
         waypointIndex.setDisable(true);
         perceiveDuration.setValueFactory(new DoubleSpinnerValueFactory(0.0,
                                                                        500.0,
                                                                        RemoteFootstepPlannerInterface.DEFAULT_PERCEIVE_TIME_REQUIRED,
                                                                        1.0));
         perceiveDuration.getValueFactory().valueProperty().addListener((ChangeListener) -> publishPerceiveDuration());
      });
      behaviorMessager.registerTopicListener(CurrentState, state -> Platform.runLater(() ->
      {
         remoteCurrentState.setText(state.name());
         replan.setDisable(!(state == PatrolBehaviorState.REVIEW && operatorPlanReview.isSelected()));
         sendPlan.setDisable(!(state == PatrolBehaviorState.REVIEW && operatorPlanReview.isSelected()));
         cancelPlanning2.setDisable(!(state == PatrolBehaviorState.NAVIGATE || state == PatrolBehaviorState.PLAN));
         skip.setDisable(!(state == PatrolBehaviorState.PERCEIVE));
      }));

      waypointManager = WaypointManager.createForUI(behaviorMessager, WaypointsToUI, WaypointsToModule, () ->
      {
         Platform.runLater(() ->
         {
            updateUIFromWaypointManager();

            // update waypoint graphics from manager
            removeAllWaypointGraphics();
            for (int i = 0; i < waypointManager.size(); i++)
            {
               PatrolWaypointGraphic graphic = new PatrolWaypointGraphic(i);
               graphic.setPosition(waypointManager.getPoseFromIndex(i).getPosition());
               graphic.setOrientation(waypointManager.getPoseFromIndex(i).getOrientation());
               graphic.getOrientationGraphic().getNode().setVisible(true);
               waypointGraphics.put(waypointManager.idFromIndex(i), graphic);
               getChildren().add(graphic);
            }
         });
      });

      behaviorMessager.registerTopicListener(CurrentWaypointIndexStatus,
                                             index ->
                                             {
                                                waypointManager.setNextFromIndex(index);
                                                Platform.runLater(() ->
                                                                  {
                                                                     remoteCurrentWaypointIndex.setText(index.toString());
                                                                     waypointIndex.getValueFactory().valueProperty().setValue(index);
                                                                  });
                                             });

      waypointPlacementActionMap = new FXUIActionMap(trigger ->
      {
         placeWaypoints.setDisable(true);
         removeAllWaypointGraphics();
         waypointManager.clearWaypoints();
         goToNextWaypointPositionEdit();
      });
      waypointPlacementActionMap.mapAction(FXUITrigger.POSITION_LEFT_CLICK, trigger ->
      {
         PatrolWaypointGraphic latestWaypoint = waypointGraphics.get(waypointManager.lastId());
         latestWaypoint.getOrientationGraphic().getNode().setVisible(true);
         orientationYawEditor.edit(latestWaypoint, exitType -> waypointPlacementActionMap.triggerAction(exitType));
      });
      waypointPlacementActionMap.mapAction(FXUITrigger.ORIENTATION_LEFT_CLICK, trigger ->
      {
         goToNextWaypointPositionEdit();
      });
      waypointPlacementActionMap.mapAction(FXUITrigger.RIGHT_CLICK, trigger ->
      {
         LogTools.debug("Completed waypoint placement.");
         removeLastWaypoint();
         publishWaypointsToModule();
         placeWaypoints.setDisable(false);
      });

      waypointInsertActionMap = new FXUIActionMap(trigger ->
      {
         placeWaypoints.setDisable(true);
         goToNextInsertedWaypointPositionEdit();
      });
      waypointInsertActionMap.mapAction(FXUITrigger.POSITION_LEFT_CLICK, trigger ->
      {
         PatrolWaypointGraphic latestWaypoint = waypointGraphics.get(waypointManager.idFromIndex(currentInsertIndex));
         latestWaypoint.getOrientationGraphic().getNode().setVisible(true);
         orientationYawEditor.edit(latestWaypoint, exitType -> waypointInsertActionMap.triggerAction(exitType));
      });
      waypointInsertActionMap.mapAction(FXUITrigger.ORIENTATION_LEFT_CLICK, trigger ->
      {
         goToNextInsertedWaypointPositionEdit();
      });
      waypointInsertActionMap.mapAction(FXUITrigger.RIGHT_CLICK, trigger ->
      {
         LogTools.debug("Completed waypoint insertion.");
         removeWaypointGraphic(currentInsertIndex);
         publishWaypointsToModule();
         placeWaypoints.setDisable(false);
      });

      upDownCenterPlacementActionMap = new FXUIActionMap(startAction ->
      {
         placeUpDownCenter.setDisable(true);
         snappedPositionEditor.edit(EditMode.XY_PLANE, upDownCenterGraphic, exitType ->
         {
            upDownCenterPlacementActionMap.triggerAction(exitType);
         });
      });
      upDownCenterPlacementActionMap.mapAction(FXUITrigger.POSITION_LEFT_CLICK, trigger ->
      {
         LogTools.debug("Place up-down center.");
         behaviorMessager.submitMessage(UpDownCenter, new Point3D(upDownCenterGraphic.getPose().getPosition()));
         placeUpDownCenter.setDisable(false);
      });
      upDownCenterPlacementActionMap.mapAction(FXUITrigger.RIGHT_CLICK, trigger ->
      {
         LogTools.debug("Aborted up-down center placement.");
         upDownCenterGraphic.clear();
         placeUpDownCenter.setDisable(false);
      });

      sceneNode.addEventHandler(MouseEvent.MOUSE_CLICKED, this::mouseClicked);
   }

   private void goToNextWaypointPositionEdit()
   {
      PatrolWaypointGraphic waypointGraphic = createWaypointGraphic();
      LogTools.debug("Placing waypoint {}", waypointManager.size());
      waypointGraphics.values().forEach(waypoint -> waypoint.setMouseTransparent(true));
      snappedPositionEditor.edit(EditMode.REGION_SNAP, waypointGraphic, exitType ->
      {
         waypointPlacementActionMap.triggerAction(exitType);
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
      snappedPositionEditor.edit(EditMode.REGION_SNAP, insertedWaypoint, exitType ->
      {
         waypointInsertActionMap.triggerAction(exitType);
         waypointGraphics.values().forEach(waypoint -> waypoint.setMouseTransparent(false));
      });
   }

   private final void mouseClicked(MouseEvent event)
   {
      if (!event.isConsumed() && event.isStillSincePress() && BehaviorUI.ACTIVE_EDITOR == null && !upDownExploration.isSelected())
      {
         PickResult pickResult = event.getPickResult();
         Node intersectedNode = pickResult.getIntersectedNode();

         if (upDownCenterGraphic.getNode() == intersectedNode)
         {
            if (event.getButton() == MouseButton.PRIMARY)
            {
               event.consume();
               placeUpDownCenter();
            }
            else if (event.getButton() == MouseButton.MIDDLE) // remove
            {
               event.consume();
               upDownCenterGraphic.clear();
               behaviorMessager.submitMessage(UpDownCenter, new Point3D(upDownCenterGraphic.getPose().getPosition()));
            }
         }

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
                  waypointInsertActionMap.start();
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
      snappedPositionEditor.edit(EditMode.REGION_SNAP, waypointToEdit, exitType ->
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
      if (waypointManager.hasWaypoints())
      {
         waypointIndex.setValueFactory(new IntegerSpinnerValueFactory(0, waypointManager.size() - 1, waypointManager.peekNextIndex(), 1));
      }
      else
      {
         waypointIndex.setValueFactory(new IntegerSpinnerValueFactory(-1, -1, -1, 1));
      }
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
   }

   private void removeLastWaypoint()
   {
      long lastId = waypointManager.lastId();

      getChildren().remove(waypointGraphics.get(lastId));
      waypointGraphics.remove(lastId);
      waypointManager.remove(lastId);
   }

   public void publishPerceiveDuration()
   {
      behaviorMessager.submitMessage(PerceiveDuration, perceiveDuration.getValue());
   }

   @FXML
   public void placeWaypoints()
   {
      waypointPlacementActionMap.start();
   }

   @FXML public void goToWaypoint()
   {
      waypointManager.setNextFromIndex(waypointIndex.getValue()); // this might not be necessary
      behaviorMessager.submitMessage(GoToWaypoint, waypointIndex.getValue());
   }

   @FXML public void cancelPlanning2()
   {
      behaviorMessager.submitMessage(CancelPlanning, new Object());
   }

   @FXML public void skip()
   {
      behaviorMessager.submitMessage(SkipPerceive, new Object());
   }

   @FXML public void stopWalking()
   {
      behaviorMessager.submitMessage(Stop, new Object());
   }

   @FXML public void loopThroughWaypoints()
   {
      behaviorMessager.submitMessage(Loop, loopThroughWaypoints.isSelected());
   }

   @FXML public void swingOverPlanarRegions()
   {
      behaviorMessager.submitMessage(SwingOvers, swingOverPlanarRegions.isSelected());
   }

   @FXML public void operatorPlanReview()
   {
      behaviorMessager.submitMessage(PlanReviewEnabled, operatorPlanReview.isSelected());
   }

   @FXML public void replan()
   {
      behaviorMessager.submitMessage(PlanReviewResult, OperatorPlanReviewResult.REPLAN);
   }

   @FXML public void sendPlan()
   {
      behaviorMessager.submitMessage(PlanReviewResult, OperatorPlanReviewResult.WALK);
   }

   @FXML public void upDownExploration()
   {
      placeWaypoints.setDisable(upDownExploration.isSelected());
      behaviorMessager.submitMessage(UpDownExplorationEnabled, upDownExploration.isSelected());
   }

   @FXML public void placeUpDownCenter()
   {
      upDownCenterPlacementActionMap.start();
   }
}
