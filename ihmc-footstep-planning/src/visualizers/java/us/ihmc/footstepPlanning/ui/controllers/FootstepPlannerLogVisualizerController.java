package us.ihmc.footstepPlanning.ui.controllers;

import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.event.Event;
import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.control.TableColumn;
import javafx.scene.control.TableRow;
import javafx.scene.control.TableView;
import javafx.scene.control.cell.PropertyValueFactory;
import javafx.scene.input.ScrollEvent;
import javafx.scene.layout.Pane;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.SimplePlanarRegionFootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.log.FootstepPlannerEdgeData;
import us.ihmc.footstepPlanning.log.FootstepPlannerIterationData;
import us.ihmc.footstepPlanning.log.FootstepPlannerLog;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogLoader;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.messager.Messager;
import us.ihmc.pathPlanning.graph.structure.GraphEdge;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.DefaultVisibilityGraphParameters;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.text.DecimalFormat;
import java.util.*;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

public class FootstepPlannerLogVisualizerController
{
   private final ObservableList<ChildStepProperty> childTableItems = FXCollections.observableArrayList();
   private final ObservableList<ParentStepProperty> parentTableItems = FXCollections.observableArrayList();
   private TableColumnHolder parentColumnHolder;
   private TableColumnHolder childColumnHolder;
   private Messager messager;
   private FootstepPlannerLog footstepPlannerLog = null;
   private List<FootstepNode> path = new ArrayList<>();
   private final Stack<FootstepNode> parentStepStack = new Stack<>();
   private final AtomicReference<ChildStepProperty> selectedRow = new AtomicReference<>();

   private List<FootstepPlannerIterationData> iterationDataList;
   private Map<GraphEdge<FootstepNode>, FootstepPlannerEdgeData> edgeDataMap;
   private FootstepNodeSnapper snapper;

   private final AtomicBoolean loadingLog = new AtomicBoolean();

   @FXML
   private TableView debugParentStepTable;
   @FXML
   private TableView debugChildStepTable;

   @FXML
   private Button reset;
   @FXML
   private Button stepInto;
   @FXML
   private Button stepBack;

   public void attachMessager(Messager messager)
   {
      this.messager = messager;
   }

   public void bindControls()
   {
      messager.registerTopicListener(FootstepPlannerMessagerAPI.RequestLoadLog, b -> loadLog());
   }

   public void setup()
   {
      // TODO refactor so that foot polygons are logged
      SideDependentList<ConvexPolygon2D> footPolygons = PlannerTools.createDefaultFootPolygons();
      snapper = new SimplePlanarRegionFootstepNodeSnapper(footPolygons);

      parentColumnHolder = new TableColumnHolder(debugParentStepTable, true);
      childColumnHolder = new TableColumnHolder(debugChildStepTable, false);

      parentTableItems.clear();
      debugParentStepTable.setItems(parentTableItems);
      childTableItems.clear();
      debugChildStepTable.setItems(childTableItems);

      Pane header = (Pane) debugParentStepTable.lookup("TableHeaderRow");
      if (header.isVisible())
      {
         header.setMaxHeight(0);
         header.setMinHeight(0);
         header.setPrefHeight(0);
         header.setVisible(false);
      }

      debugChildStepTable.setRowFactory(tableValue ->
                                        {
                                           TableRow<ChildStepProperty> row = new TableRow<>();
                                           row.setOnMouseClicked(event ->
                                                                 {
                                                                    if (event.getClickCount() == 2 && (!row.isEmpty()))
                                                                    {
                                                                       ChildStepProperty rowData = row.getItem();
                                                                       if (!rowData.expanded)
                                                                          return;
                                                                       parentStepStack.push(rowData.edgeData.getCandidateNode());
                                                                       updateTable();
                                                                    }
                                                                 });
                                           return row;
                                        });

      debugParentStepTable.setRowFactory(tableValue ->
                                         {
                                            TableRow<ParentStepProperty> row = new TableRow<>();
                                            row.setOnMouseClicked(event ->
                                                                  {
                                                                     if (event.getClickCount() == 2)
                                                                        stepBack();
                                                                  });
                                            return row;
                                         });

      debugParentStepTable.addEventFilter(ScrollEvent.ANY, Event::consume);
   }

   public void loadLog()
   {
      if(loadingLog.get())
         return;

      loadingLog.set(true);
      FootstepPlannerLogLoader logLoader = new FootstepPlannerLogLoader();
      messager.submitMessage(FootstepPlannerMessagerAPI.LoadLogStatus, "Loading log...");

      if(logLoader.load())
      {
         footstepPlannerLog = logLoader.getLog();
         loadLog(footstepPlannerLog);
      }
      else
      {
         messager.submitMessage(FootstepPlannerMessagerAPI.LoadLogStatus, "Error loading log");
      }

      loadingLog.set(false);
   }

   public void loadLog(FootstepPlannerLog footstepPlannerLog)
   {
      // publish log name
      messager.submitMessage(FootstepPlannerMessagerAPI.LoadLogStatus, footstepPlannerLog.getLogName());

      // publish footstep parameters
      DefaultFootstepPlannerParameters footstepPlannerParameters = new DefaultFootstepPlannerParameters();
      footstepPlannerParameters.set(footstepPlannerLog.getFootstepParametersPacket());
      messager.submitMessage(FootstepPlannerMessagerAPI.PlannerParameters, footstepPlannerParameters);

      // publish body parameters
      DefaultVisibilityGraphParameters visibilityGraphParameters = new DefaultVisibilityGraphParameters();
      visibilityGraphParameters.set(footstepPlannerLog.getBodyPathParametersPacket());
      messager.submitMessage(FootstepPlannerMessagerAPI.VisibilityGraphsParameters, visibilityGraphParameters);

      // publish request parameters
      messager.submitMessage(FootstepPlannerMessagerAPI.InitialSupportSide, RobotSide.fromByte(footstepPlannerLog.getRequestPacket().getInitialStanceRobotSide()));
      messager.submitMessage(FootstepPlannerMessagerAPI.GoalPosition, footstepPlannerLog.getRequestPacket().getGoalPositionInWorld());
      messager.submitMessage(FootstepPlannerMessagerAPI.GoalOrientation, footstepPlannerLog.getRequestPacket().getGoalOrientationInWorld());
      messager.submitMessage(FootstepPlannerMessagerAPI.GoalDistanceProximity, footstepPlannerLog.getRequestPacket().getGoalDistanceProximity());
      messager.submitMessage(FootstepPlannerMessagerAPI.GoalYawProximity, footstepPlannerLog.getRequestPacket().getGoalYawProximity());
      messager.submitMessage(FootstepPlannerMessagerAPI.PlannerType, FootstepPlannerType.fromByte(footstepPlannerLog.getRequestPacket().getRequestedFootstepPlannerType()));
      messager.submitMessage(FootstepPlannerMessagerAPI.PlannerTimeout, footstepPlannerLog.getRequestPacket().getTimeout());
      messager.submitMessage(FootstepPlannerMessagerAPI.PlannerHorizonLength, footstepPlannerLog.getRequestPacket().getHorizonLength());
      messager.submitMessage(FootstepPlannerMessagerAPI.AssumeFlatGround, footstepPlannerLog.getRequestPacket().getAssumeFlatGround());
      PlanarRegionsList planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsList(footstepPlannerLog.getRequestPacket().getPlanarRegionsListMessage());
      messager.submitMessage(FootstepPlannerMessagerAPI.PlanarRegionData, planarRegionsList);
      messager.submitMessage(FootstepPlannerMessagerAPI.PlannerRequestId, footstepPlannerLog.getRequestPacket().getPlannerRequestId());
      messager.submitMessage(FootstepPlannerMessagerAPI.StartPosition, footstepPlannerLog.getRequestPacket().getStanceFootPositionInWorld());
      messager.submitMessage(FootstepPlannerMessagerAPI.StartOrientation, footstepPlannerLog.getRequestPacket().getStanceFootOrientationInWorld());

      // publish status
      messager.submitMessage(FootstepPlannerMessagerAPI.BodyPathData, footstepPlannerLog.getStatusPacket().getBodyPath());
      messager.submitMessage(FootstepPlannerMessagerAPI.PlanningResult, FootstepPlanningResult.fromByte(footstepPlannerLog.getStatusPacket().getFootstepPlanningResult()));
      messager.submitMessage(FootstepPlannerMessagerAPI.LowLevelGoalPosition, footstepPlannerLog.getStatusPacket().getLowLevelPlannerGoal().getPosition());
      messager.submitMessage(FootstepPlannerMessagerAPI.LowLevelGoalOrientation, footstepPlannerLog.getStatusPacket().getLowLevelPlannerGoal().getOrientation());
      messager.submitMessage(FootstepPlannerMessagerAPI.FootstepPlanResponse, footstepPlannerLog.getStatusPacket().getFootstepDataList());

      this.iterationDataList = footstepPlannerLog.getIterationData();
      this.edgeDataMap = footstepPlannerLog.getEdgeDataMap();
      this.snapper.setPlanarRegions(planarRegionsList);

      parentStepStack.clear();
      selectedRow.set(null);

      path.clear();
      recursivelyBuildPath(iterationDataList.get(0), iterationDataList, edgeDataMap);

      FootstepNode startNode = path.get(0);
      parentStepStack.push(startNode);
      updateTable();
   }

   private void recursivelyBuildPath(FootstepPlannerIterationData iterationData, List<FootstepPlannerIterationData> iterationDataList, Map<GraphEdge<FootstepNode>, FootstepPlannerEdgeData> edgeDataMap)
   {
      FootstepNode stanceNode = iterationData.getStanceNode();
      path.add(stanceNode);

      for (int i = 0; i < iterationData.getChildNodes().size(); i++)
      {
         FootstepNode childNode = iterationData.getChildNodes().get(i);
         if(edgeDataMap.get(new GraphEdge<>(stanceNode, childNode)).getSolutionEdge())
         {
            iterationDataList.stream().filter(data -> data.getStanceNode().equals(childNode)).findAny().ifPresent(nextData -> recursivelyBuildPath(nextData, iterationDataList, edgeDataMap));
            return;
         }
      }
   }

   private void updateTable()
   {
      FootstepNode parentNode = parentStepStack.peek();
      Optional<FootstepPlannerIterationData> iterationDataOptional = iterationDataList.stream().filter(data -> data.getStanceNode().equals(parentNode)).findFirst();

      FootstepPlannerIterationData iterationData = iterationDataOptional.get();

      parentTableItems.clear();
      childTableItems.clear();

      for (int i = 0; i < iterationData.getChildNodes().size(); i++)
      {
         FootstepNode childNode = iterationData.getChildNodes().get(i);
         FootstepPlannerEdgeData edgeData = edgeDataMap.get(new GraphEdge<>(iterationData.getStanceNode(), childNode));
         boolean expanded = iterationDataList.stream().anyMatch(data -> data.getStanceNode().equals(childNode));
         ChildStepProperty stepProperty = new ChildStepProperty(edgeData, expanded);
         childTableItems.add(stepProperty);
      }

      ParentStepProperty stepProperty = new ParentStepProperty(iterationData);
      parentTableItems.add(stepProperty);

      debugChildStepTable.getSortOrder().clear();
      debugChildStepTable.getSortOrder().add(childColumnHolder.solutionStep);
      childColumnHolder.solutionStep.setSortType(TableColumn.SortType.DESCENDING);
      debugChildStepTable.getSortOrder().add(childColumnHolder.totalCostColumn);
      childColumnHolder.totalCostColumn.setSortType(TableColumn.SortType.ASCENDING);
      debugChildStepTable.sort();

      // TODO setup visualizer
//      messager.submitMessage(ValkyriePlannerMessagerAPI.parentDebugStep, Pair.of(stepProperty.transform, stepProperty.snapData.getCroppedFoothold()));
//      messager.submitMessage(ValkyriePlannerMessagerAPI.idealDebugStep, stepProperty.idealStepTransform);
//
//      debugChildStepTable.getSelectionModel().selectedItemProperty().addListener((observer, oldValue, newValue) ->
//                                                                                 {
//                                                                                    if (newValue != null)
//                                                                                    {
//                                                                                       ChildStepProperty property = (ChildStepProperty) newValue;
//                                                                                       messager.submitMessage(ValkyriePlannerMessagerAPI.childDebugStep,
//                                                                                                              Pair.of(property.transform,
//                                                                                                                      property.edgeData.getCandidateNodeSnapData().getCroppedFoothold()));
//                                                                                       selectedRow.set(property);
//                                                                                    }
//                                                                                 });

      debugChildStepTable.getSelectionModel().select(0);
      debugChildStepTable.getFocusModel().focus(0);
   }

   public void stepInto()
   {
      ChildStepProperty selectedRow = this.selectedRow.get();
      if (selectedRow != null && selectedRow.expanded)
      {
         parentStepStack.push(selectedRow.edgeData.getCandidateNode());
         updateTable();
      }
   }

   public void stepBack()
   {
      if (parentStepStack.size() <= 1)
      {
         return;
      }

      parentStepStack.pop();
      updateTable();
   }

   private final DecimalFormat doubleFormat = new DecimalFormat("#0.000");

   private class TableColumnHolder
   {
      private final TableColumn<ChildStepProperty, String> xColumn = new TableColumn<>("X");
      private final TableColumn<ChildStepProperty, String> yColumn = new TableColumn<>("Y");
      private final TableColumn<ChildStepProperty, String> zColumn = new TableColumn<>("Z");
      private final TableColumn<ChildStepProperty, String> yawColumn = new TableColumn<>("Yaw");
      private final TableColumn<ChildStepProperty, String> pitchColumn = new TableColumn<>("Pitch");
      private final TableColumn<ChildStepProperty, String> rollColumn = new TableColumn<>("Roll");
      private final TableColumn<ChildStepProperty, String> widthColumn = new TableColumn<>("Width");
      private final TableColumn<ChildStepProperty, String> lengthColumn = new TableColumn<>("Length");
      private final TableColumn<ChildStepProperty, String> heightColumn = new TableColumn<>("Height");
      private final TableColumn<ChildStepProperty, String> reachColumn = new TableColumn<>("Reach");
      private final TableColumn<ChildStepProperty, String> stepYawColumn = new TableColumn<>("dYaw");
      private final TableColumn<ChildStepProperty, String> areaPercentageColumn = new TableColumn<>("Area %");
      private final TableColumn<ChildStepProperty, String> edgeCostColumn = new TableColumn<>("Edge Cost");
      private final TableColumn<ChildStepProperty, String> heuristicCostColumn = new TableColumn<>("Heuristic Cost");
      private final TableColumn<ChildStepProperty, String> totalCostColumn = new TableColumn<>("Total Cost");
      private final TableColumn<ChildStepProperty, String> rejectionReasonColumn = new TableColumn<>("Rejection Reason");
      private final TableColumn<ChildStepProperty, String> expandedColumn = new TableColumn<>("Expanded");
      private final TableColumn<ChildStepProperty, String> solutionStep = new TableColumn<>("Solution");

      public TableColumnHolder(TableView table, boolean parentTable)
      {
         xColumn.setCellValueFactory(new PropertyValueFactory("x"));
         yColumn.setCellValueFactory(new PropertyValueFactory("y"));
         zColumn.setCellValueFactory(new PropertyValueFactory("z"));
         yawColumn.setCellValueFactory(new PropertyValueFactory("yaw"));
         pitchColumn.setCellValueFactory(new PropertyValueFactory("pitch"));
         rollColumn.setCellValueFactory(new PropertyValueFactory("roll"));
         widthColumn.setCellValueFactory(new PropertyValueFactory<>("width"));
         lengthColumn.setCellValueFactory(new PropertyValueFactory<>("length"));
         heightColumn.setCellValueFactory(new PropertyValueFactory<>("height"));
         reachColumn.setCellValueFactory(new PropertyValueFactory<>("reach"));
         stepYawColumn.setCellValueFactory(new PropertyValueFactory<>("stepYaw"));
         areaPercentageColumn.setCellValueFactory(new PropertyValueFactory<>("areaPercentage"));
         edgeCostColumn.setCellValueFactory(new PropertyValueFactory("edgeCost"));
         heuristicCostColumn.setCellValueFactory(new PropertyValueFactory("heuristicCost"));
         totalCostColumn.setCellValueFactory(new PropertyValueFactory("totalCost"));
         rejectionReasonColumn.setCellValueFactory(new PropertyValueFactory("rejectionReason"));
         expandedColumn.setCellValueFactory(new PropertyValueFactory("expanded"));
         solutionStep.setCellValueFactory(new PropertyValueFactory<>("solution"));

         table.getColumns().add(xColumn);
         table.getColumns().add(yColumn);
         table.getColumns().add(zColumn);
         table.getColumns().add(yawColumn);
         table.getColumns().add(pitchColumn);
         table.getColumns().add(rollColumn);
         table.getColumns().add(widthColumn);

         if(!parentTable)
         {
            table.getColumns().add(lengthColumn);
            table.getColumns().add(heightColumn);
            table.getColumns().add(reachColumn);
            table.getColumns().add(stepYawColumn);
            table.getColumns().add(areaPercentageColumn);
            table.getColumns().add(edgeCostColumn);
            table.getColumns().add(heuristicCostColumn);
            table.getColumns().add(totalCostColumn);
            table.getColumns().add(rejectionReasonColumn);
            table.getColumns().add(expandedColumn);
            table.getColumns().add(solutionStep);
         }

         xColumn.setMaxWidth(60);
         yColumn.setMaxWidth(60);
         zColumn.setMaxWidth(60);
         yawColumn.setMaxWidth(60);
         pitchColumn.setMaxWidth(60);
         rollColumn.setMaxWidth(60);
         widthColumn.setPrefWidth(60);
         lengthColumn.setPrefWidth(60);
         heightColumn.setPrefWidth(60);
         reachColumn.setPrefWidth(60);
         stepYawColumn.setPrefWidth(60);
         areaPercentageColumn.setPrefWidth(60);
         edgeCostColumn.setPrefWidth(80);
         heuristicCostColumn.setPrefWidth(80);
         totalCostColumn.setPrefWidth(110);
         rejectionReasonColumn.setPrefWidth(200);
         expandedColumn.setPrefWidth(80);
         solutionStep.setPrefWidth(80);
      }
   }

   public class ParentStepProperty
   {
      private final FootstepNode stanceNode;
      private final FootstepNodeSnapData snapData;
      private final RigidBodyTransform transform = new RigidBodyTransform();
      private final RigidBodyTransform idealStepTransform = new RigidBodyTransform();

      public ParentStepProperty(FootstepPlannerIterationData iterationData)
      {
         this.stanceNode = iterationData.getStanceNode();
         this.snapData = iterationData.getStanceNodeSnapData();
         FootstepNodeTools.getSnappedNodeTransform(stanceNode, snapData.getSnapTransform(), transform);

         FootstepNode idealStep = iterationData.getIdealStep();
         FootstepNodeSnapData idealStepSnapData = snapper.snapFootstepNode(idealStep);
         if(idealStepSnapData == null || idealStepSnapData.getSnapTransform().containsNaN())
         {
            FootstepNodeTools.getNodeTransform(idealStep, idealStepTransform);
            idealStepTransform.setTranslationZ(transform.getTranslationZ());
         }
         else
         {
            FootstepNodeTools.getSnappedNodeTransform(idealStep, idealStepSnapData.getSnapTransform(), idealStepTransform);
         }
      }

      public String getX()
      {
         return doubleFormat.format(transform.getTranslationX());
      }

      public String getY()
      {
         return doubleFormat.format(transform.getTranslationY());
      }

      public String getZ()
      {
         return doubleFormat.format(transform.getTranslationZ());
      }

      public String getYaw()
      {
         return doubleFormat.format(transform.getRotation().getYaw());
      }

      public String getPitch()
      {
         return doubleFormat.format(transform.getRotation().getPitch());
      }

      public String getRoll()
      {
         return doubleFormat.format(transform.getRotation().getRoll());
      }
   }

   public class ChildStepProperty
   {
      private final FootstepPlannerEdgeData edgeData;
      private final RigidBodyTransform transform = new RigidBodyTransform();
      private final boolean expanded;
      private final double stepYaw;

      public ChildStepProperty(FootstepPlannerEdgeData edgeData,
                               boolean expanded)
      {
         this.edgeData = edgeData;
         this.expanded = expanded;

         FootstepNode candidateNode = edgeData.getCandidateNode();
         FootstepNode stanceNode = edgeData.getCandidateNode();

         if (edgeData.getCandidateNodeSnapData().getSnapTransform().containsNaN())
         {
            FootstepNodeTools.getSnappedNodeTransform(candidateNode, new RigidBodyTransform(), transform);
            stepYaw = Double.NaN;
         }
         else
         {
            FootstepNodeTools.getSnappedNodeTransform(candidateNode, edgeData.getCandidateNodeSnapData().getSnapTransform(), transform);
            stepYaw = candidateNode.getRobotSide().negateIfLeftSide(AngleTools.computeAngleDifferenceMinusPiToPi(candidateNode.getYaw(), stanceNode.getYaw()));
         }
      }

      public String getX()
      {
         return doubleFormat.format(transform.getTranslationX());
      }

      public String getY()
      {
         return doubleFormat.format(transform.getTranslationY());
      }

      public String getZ()
      {
         return doubleFormat.format(transform.getTranslationZ());
      }

      public String getYaw()
      {
         return doubleFormat.format(transform.getRotation().getYaw());
      }

      public String getPitch()
      {
         return doubleFormat.format(transform.getRotation().getPitch());
      }

      public String getRoll()
      {
         return doubleFormat.format(transform.getRotation().getRoll());
      }

      public String getWidth()
      {
         return doubleFormat.format(edgeData.getStepWidth());
      }

      public String getLength()
      {
         return doubleFormat.format(edgeData.getStepLength());
      }

      public String getHeight()
      {
         return doubleFormat.format(edgeData.getStepHeight());
      }

      public String getReach()
      {
         return doubleFormat.format(edgeData.getStepReach());
      }

      public String getStepYaw()
      {
         return doubleFormat.format(stepYaw);
      }

      public String getAreaPercentage()
      {
         return doubleFormat.format(edgeData.getFootAreaPercentage());
      }

      public String getEdgeCost()
      {
         return doubleFormat.format(edgeData.getEdgeCost());
      }

      public String getHeuristicCost()
      {
         return doubleFormat.format(edgeData.getHeuristicCost());
      }

      public String getTotalCost()
      {
         return doubleFormat.format(edgeData.getCostFromStart() + edgeData.getHeuristicCost());
      }

      public String getExpanded()
      {
         return Boolean.toString(expanded);
      }

      public String getRejectionReason()
      {
         return edgeData.getRejectionReason() == null ? "" : edgeData.getRejectionReason().toString();
      }

      public String getSolution()
      {
         return Boolean.toString(edgeData.getSolutionEdge());
      }
   }
}
