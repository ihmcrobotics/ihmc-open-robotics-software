package us.ihmc.footstepPlanning.ui.controllers;

import javafx.application.Platform;
import javafx.beans.value.ChangeListener;
import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.event.Event;
import javafx.fxml.FXML;
import javafx.scene.control.*;
import javafx.scene.control.cell.PropertyValueFactory;
import javafx.scene.input.ScrollEvent;
import javafx.scene.layout.Pane;
import org.apache.commons.lang3.NotImplementedException;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.icp.DefaultSplitFractionCalculatorParameters;
import us.ihmc.footstepPlanning.log.FootstepPlannerEdgeData;
import us.ihmc.footstepPlanning.log.FootstepPlannerIterationData;
import us.ihmc.footstepPlanning.log.FootstepPlannerLog;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogLoader;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogLoader.LoadRequestType;
import us.ihmc.footstepPlanning.swing.DefaultSwingPlannerParameters;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.pathPlanning.graph.structure.GraphEdge;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityGraphHolder;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityMap;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityMapHolder;
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
   private JavaFXMessager messager;
   private FootstepPlannerLog footstepPlannerLog = null;
   private List<FootstepNode> path = new ArrayList<>();
   private final Stack<FootstepNode> parentStepStack = new Stack<>();
   private final AtomicReference<ChildStepProperty> selectedRow = new AtomicReference<>();

   private List<FootstepPlannerIterationData> iterationDataList;
   private Map<GraphEdge<FootstepNode>, FootstepPlannerEdgeData> edgeDataMap;
   private FootstepNodeSnapAndWiggler snapper = new FootstepNodeSnapAndWiggler(PlannerTools.createDefaultFootPolygons(), new DefaultFootstepPlannerParameters()); // TODO

   private final AtomicBoolean loadingLog = new AtomicBoolean();

   @FXML
   private TableView stanceStepTable;
   @FXML
   private TableView candidateStepTable;

   @FXML
   private Button reset;
   @FXML
   private Button stepInto;
   @FXML
   private Button stepBack;

   @FXML
   private CheckBox showStanceStep;
   @FXML
   private CheckBox showUnsnappedStep;
   @FXML
   private CheckBox showSnappedStep;
   @FXML
   private CheckBox showSnapAndWiggledStep;
   @FXML
   private CheckBox showIdealStep;

   public void attachMessager(JavaFXMessager messager)
   {
      this.messager = messager;
   }

   public void bindControls()
   {
      messager.registerTopicListener(FootstepPlannerMessagerAPI.RequestLoadLog, type -> loadLog(type));

      AtomicReference<PlanarRegionsList> planarRegionData = messager.createInput(FootstepPlannerMessagerAPI.PlanarRegionData);
      messager.registerTopicListener(FootstepPlannerMessagerAPI.GraphData,
                                     graphData -> Platform.runLater(() -> updateGraphData(planarRegionData.get(), graphData.getLeft(), graphData.getRight())));

      messager.bindBidirectional(FootstepPlannerMessagerAPI.ShowLoggedStanceStep, showStanceStep.selectedProperty(), true);
      messager.bindBidirectional(FootstepPlannerMessagerAPI.ShowLoggedUnsnappedCandidateStep, showUnsnappedStep.selectedProperty(), true);
      messager.bindBidirectional(FootstepPlannerMessagerAPI.ShowLoggedSnappedCandidateStep, showSnappedStep.selectedProperty(), true);
      messager.bindBidirectional(FootstepPlannerMessagerAPI.ShowLoggedWiggledCandidateStep, showSnapAndWiggledStep.selectedProperty(), true);
      messager.bindBidirectional(FootstepPlannerMessagerAPI.ShowLoggedIdealStep, showIdealStep.selectedProperty(), true);

      messager.registerTopicListener(FootstepPlannerMessagerAPI.ShowLogGraphics, show ->
      {
         messager.submitMessage(FootstepPlannerMessagerAPI.ShowLoggedStanceStep, show);
         messager.submitMessage(FootstepPlannerMessagerAPI.ShowLoggedUnsnappedCandidateStep, show);
         messager.submitMessage(FootstepPlannerMessagerAPI.ShowLoggedSnappedCandidateStep, show);
         messager.submitMessage(FootstepPlannerMessagerAPI.ShowLoggedWiggledCandidateStep, show);
         messager.submitMessage(FootstepPlannerMessagerAPI.ShowLoggedIdealStep, show);
      });
   }

   public void onPrimaryStageLoaded()
   {
      parentColumnHolder = new TableColumnHolder(stanceStepTable, true);
      childColumnHolder = new TableColumnHolder(candidateStepTable, false);

      parentTableItems.clear();
      stanceStepTable.setItems(parentTableItems);
      childTableItems.clear();
      candidateStepTable.setItems(childTableItems);

      Pane header = (Pane) stanceStepTable.lookup("TableHeaderRow");
      if (header.isVisible())
      {
         header.setMaxHeight(0);
         header.setMinHeight(0);
         header.setPrefHeight(0);
         header.setVisible(false);
      }

      candidateStepTable.setRowFactory(tableValue ->
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

      stanceStepTable.setRowFactory(tableValue ->
                                         {
                                            TableRow<ParentStepProperty> row = new TableRow<>();
                                            row.setOnMouseClicked(event ->
                                                                  {
                                                                     if (event.getClickCount() == 2)
                                                                        stepBack();
                                                                  });
                                            return row;
                                         });

      stanceStepTable.addEventFilter(ScrollEvent.ANY, Event::consume);
   }

   public void setContactPointParameters(SideDependentList<List<Point2D>> defaultContactPoints)
   {
      SideDependentList<ConvexPolygon2D> footPolygons = new SideDependentList<>(side ->
                                                                                {
                                                                                   List<Point2D> footPoints = defaultContactPoints.get(side);
                                                                                   return new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(footPoints));
                                                                                });
      snapper = new FootstepNodeSnapAndWiggler(footPolygons, new DefaultFootstepPlannerParameters());
   }

   public void loadLog(LoadRequestType loadRequestType)
   {
      if(loadingLog.get())
         return;

      loadingLog.set(true);
      FootstepPlannerLogLoader logLoader = new FootstepPlannerLogLoader();

      FootstepPlannerLogLoader.LoadResult loadResult = logLoader.load(loadRequestType, footstepPlannerLog);
      if(loadResult == FootstepPlannerLogLoader.LoadResult.LOADED)
      {
         footstepPlannerLog = logLoader.getLog();
         loadLog(footstepPlannerLog);
      }
      else if (loadResult == FootstepPlannerLogLoader.LoadResult.ERROR)
      {
         messager.submitMessage(FootstepPlannerMessagerAPI.LoadLogStatus, "Error loading log");
      }

      loadingLog.set(false);
   }

   private void loadLog(FootstepPlannerLog footstepPlannerLog)
   {
      // publish log name
      messager.submitMessage(FootstepPlannerMessagerAPI.LoadLogStatus, footstepPlannerLog.getLogName());

      // publish body path planner parameters
      DefaultVisibilityGraphParameters visibilityGraphParameters = new DefaultVisibilityGraphParameters();
      visibilityGraphParameters.set(footstepPlannerLog.getBodyPathParametersPacket());
      messager.submitMessage(FootstepPlannerMessagerAPI.VisibilityGraphsParameters, visibilityGraphParameters);

      // publish footstep parameters
      DefaultFootstepPlannerParameters footstepPlannerParameters = new DefaultFootstepPlannerParameters();
      footstepPlannerParameters.set(footstepPlannerLog.getFootstepParametersPacket());
      messager.submitMessage(FootstepPlannerMessagerAPI.PlannerParameters, footstepPlannerParameters);

      // publish swing parameters
      DefaultSwingPlannerParameters swingPlannerParameters = new DefaultSwingPlannerParameters();
      swingPlannerParameters.set(footstepPlannerLog.getSwingPlannerParametersPacket());
      messager.submitMessage(FootstepPlannerMessagerAPI.SwingPlannerParameters, swingPlannerParameters);

      // publish split fraction parameters
      DefaultSplitFractionCalculatorParameters splitFractionParameters = new DefaultSplitFractionCalculatorParameters();
      splitFractionParameters.set(footstepPlannerLog.getSplitFractionParametersPacket());
      messager.submitMessage(FootstepPlannerMessagerAPI.SplitFractionParameters, splitFractionParameters);

      // publish request parameters
      messager.submitMessage(FootstepPlannerMessagerAPI.InitialSupportSide, RobotSide.fromByte(footstepPlannerLog.getRequestPacket().getRequestedInitialStanceSide()));
      messager.submitMessage(FootstepPlannerMessagerAPI.LeftFootGoalPose, footstepPlannerLog.getRequestPacket().getGoalLeftFootPose());
      messager.submitMessage(FootstepPlannerMessagerAPI.RightFootGoalPose, footstepPlannerLog.getRequestPacket().getGoalRightFootPose());
      messager.submitMessage(FootstepPlannerMessagerAPI.GoalDistanceProximity, footstepPlannerLog.getRequestPacket().getGoalDistanceProximity());
      messager.submitMessage(FootstepPlannerMessagerAPI.GoalYawProximity, footstepPlannerLog.getRequestPacket().getGoalYawProximity());

      messager.submitMessage(FootstepPlannerMessagerAPI.PlannerTimeout, footstepPlannerLog.getRequestPacket().getTimeout());
      messager.submitMessage(FootstepPlannerMessagerAPI.PlannerHorizonLength, footstepPlannerLog.getRequestPacket().getHorizonLength());
      messager.submitMessage(FootstepPlannerMessagerAPI.AssumeFlatGround, footstepPlannerLog.getRequestPacket().getAssumeFlatGround());
      PlanarRegionsList planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsList(footstepPlannerLog.getRequestPacket().getPlanarRegionsListMessage());
      messager.submitMessage(FootstepPlannerMessagerAPI.PlanarRegionData, planarRegionsList);
      messager.submitMessage(FootstepPlannerMessagerAPI.PlannerRequestId, footstepPlannerLog.getRequestPacket().getPlannerRequestId());
      messager.submitMessage(FootstepPlannerMessagerAPI.LeftFootPose, footstepPlannerLog.getRequestPacket().getStartLeftFootPose());
      messager.submitMessage(FootstepPlannerMessagerAPI.RightFootPose, footstepPlannerLog.getRequestPacket().getStartRightFootPose());
      messager.submitMessage(FootstepPlannerMessagerAPI.PerformAStarSearch, footstepPlannerLog.getRequestPacket().getPerformAStarSearch());
      messager.submitMessage(FootstepPlannerMessagerAPI.PlanBodyPath, footstepPlannerLog.getRequestPacket().getPlanBodyPath());

      // publish status
      messager.submitMessage(FootstepPlannerMessagerAPI.FootstepPlanningResultTopic, FootstepPlanningResult.fromByte(footstepPlannerLog.getStatusPacket().getFootstepPlanningResult()));
      messager.submitMessage(FootstepPlannerMessagerAPI.LowLevelGoalPosition, footstepPlannerLog.getStatusPacket().getGoalPose().getPosition());
      messager.submitMessage(FootstepPlannerMessagerAPI.LowLevelGoalOrientation, footstepPlannerLog.getStatusPacket().getGoalPose().getOrientation());
      messager.submitMessage(FootstepPlannerMessagerAPI.FootstepPlanResponse, footstepPlannerLog.getStatusPacket().getFootstepDataList());

      // publish visibility graph data
      VisibilityGraphHolder visibilityGraphHolder = footstepPlannerLog.getVisibilityGraphHolder();
      messager.submitMessage(FootstepPlannerMessagerAPI.StartVisibilityMap, new VisibilityMapWrapper(visibilityGraphHolder.getStartMapId(), visibilityGraphHolder.getStartVisibilityMap()));
      messager.submitMessage(FootstepPlannerMessagerAPI.GoalVisibilityMap, new VisibilityMapWrapper(visibilityGraphHolder.getGoalMapId(), visibilityGraphHolder.getGoalVisibilityMap()));
      messager.submitMessage(FootstepPlannerMessagerAPI.InterRegionVisibilityMap, new VisibilityMapWrapper(visibilityGraphHolder.getInterRegionsMapId(), visibilityGraphHolder.getInterRegionsVisibilityMap()));
      messager.submitMessage(FootstepPlannerMessagerAPI.VisibilityMapWithNavigableRegionData, visibilityGraphHolder.getVisibilityMapsWithNavigableRegions());
      messager.submitMessage(FootstepPlannerMessagerAPI.BodyPathData, footstepPlannerLog.getStatusPacket().getBodyPath());

      // set graphics
      messager.submitMessage(FootstepPlannerMessagerAPI.ShowRobot, false);
      messager.submitMessage(FootstepPlannerMessagerAPI.ShowOccupancyMap, false);
      messager.submitMessage(FootstepPlannerMessagerAPI.ShowClusterNavigableExtrusions, false);
      messager.submitMessage(FootstepPlannerMessagerAPI.ShowClusterNonNavigableExtrusions, false);
      messager.submitMessage(FootstepPlannerMessagerAPI.ShowClusterRawPoints, false);
      messager.submitMessage(FootstepPlannerMessagerAPI.ShowStartVisibilityMap, false);
      messager.submitMessage(FootstepPlannerMessagerAPI.ShowFootstepPlan, false); // hide plan by default
      messager.submitMessage(FootstepPlannerMessagerAPI.BindStartToRobot, false);
      messager.submitMessage(FootstepPlannerMessagerAPI.ShowLogGraphics, true);

      // set graph data
      updateGraphData(planarRegionsList, footstepPlannerLog.getEdgeDataMap(), footstepPlannerLog.getIterationData());
   }

   private class VisibilityMapWrapper implements VisibilityMapHolder
   {
      private final int id;
      private final VisibilityMap visibilityMap;

      VisibilityMapWrapper(int id, VisibilityMap visibilityMap)
      {
         this.id = id;
         this.visibilityMap = visibilityMap;
      }

      @Override
      public int getMapId()
      {
         return id;
      }

      @Override
      public VisibilityMap getVisibilityMapInLocal()
      {
         throw new NotImplementedException("Local visibility map not implemented by log loader");
      }

      @Override
      public VisibilityMap getVisibilityMapInWorld()
      {
         return visibilityMap;
      }
   }

   private void updateGraphData(PlanarRegionsList planarRegionsList,
                                Map<GraphEdge<FootstepNode>, FootstepPlannerEdgeData> edgeDataMap,
                                List<FootstepPlannerIterationData> iterationData)
   {
      this.iterationDataList = iterationData;
      this.edgeDataMap = edgeDataMap;
      this.snapper.setPlanarRegions(planarRegionsList);

      parentStepStack.clear();
      selectedRow.set(null);
      path.clear();

      if (!iterationDataList.isEmpty())
      {
         recursivelyBuildPath(iterationDataList.get(0), iterationDataList, this.edgeDataMap);
         FootstepNode startNode = path.get(0);
         parentStepStack.push(startNode);
         updateTable();
      }
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

      parentTableItems.clear();
      childTableItems.clear();

      if (!iterationDataOptional.isPresent())
      {
         return;
      }

      FootstepPlannerIterationData iterationData = iterationDataOptional.get();
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

      candidateStepTable.getSortOrder().clear();
      candidateStepTable.getSortOrder().add(childColumnHolder.solutionStep);
      childColumnHolder.solutionStep.setSortType(TableColumn.SortType.DESCENDING);
      candidateStepTable.getSortOrder().add(childColumnHolder.totalCostColumn);
      childColumnHolder.totalCostColumn.setSortType(TableColumn.SortType.ASCENDING);
      candidateStepTable.sort();

      messager.submitMessage(FootstepPlannerMessagerAPI.LoggedStanceStepToVisualize, Pair.of(stepProperty.stanceNode, stepProperty.snapData));
      messager.submitMessage(FootstepPlannerMessagerAPI.LoggedIdealStep, stepProperty.idealStepTransform);

      candidateStepTable.getSelectionModel().selectedItemProperty().addListener(onStepSelected());
      candidateStepTable.getSelectionModel().select(0);
      candidateStepTable.getFocusModel().focus(0);
   }

   private ChangeListener<ChildStepProperty> onStepSelected()
   {
      return (observer, oldValue, newValue) ->
      {
         if (newValue != null)
         {
            messager.submitMessage(FootstepPlannerMessagerAPI.LoggedCandidateStepToVisualize, newValue.edgeData);
            selectedRow.set(newValue);
         }
      };
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
      private final TableColumn<ChildStepProperty, Integer> stepIndex = new TableColumn<>("Index");
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

      public TableColumnHolder(TableView<ChildStepProperty> table, boolean parentTable)
      {
         stepIndex.setCellValueFactory(new PropertyValueFactory<>("index"));
         xColumn.setCellValueFactory(new PropertyValueFactory<>("x"));
         yColumn.setCellValueFactory(new PropertyValueFactory<>("y"));
         zColumn.setCellValueFactory(new PropertyValueFactory<>("z"));
         yawColumn.setCellValueFactory(new PropertyValueFactory<>("yaw"));
         pitchColumn.setCellValueFactory(new PropertyValueFactory<>("pitch"));
         rollColumn.setCellValueFactory(new PropertyValueFactory<>("roll"));
         widthColumn.setCellValueFactory(new PropertyValueFactory<>("width"));
         lengthColumn.setCellValueFactory(new PropertyValueFactory<>("length"));
         heightColumn.setCellValueFactory(new PropertyValueFactory<>("height"));
         reachColumn.setCellValueFactory(new PropertyValueFactory<>("reach"));
         stepYawColumn.setCellValueFactory(new PropertyValueFactory<>("stepYaw"));
         areaPercentageColumn.setCellValueFactory(new PropertyValueFactory<>("areaPercentage"));
         edgeCostColumn.setCellValueFactory(new PropertyValueFactory<>("edgeCost"));
         heuristicCostColumn.setCellValueFactory(new PropertyValueFactory<>("heuristicCost"));
         totalCostColumn.setCellValueFactory(new PropertyValueFactory<>("totalCost"));
         rejectionReasonColumn.setCellValueFactory(new PropertyValueFactory<>("rejectionReason"));
         expandedColumn.setCellValueFactory(new PropertyValueFactory<>("expanded"));
         solutionStep.setCellValueFactory(new PropertyValueFactory<>("solution"));

         table.getColumns().add(stepIndex);
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

         stepIndex.setMaxWidth(60);
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
      private final RigidBodyTransform snappedNodeTransform = new RigidBodyTransform();
      private final RigidBodyTransform idealStepTransform = new RigidBodyTransform();

      public ParentStepProperty(FootstepPlannerIterationData iterationData)
      {
         this.stanceNode = iterationData.getStanceNode();
         this.snapData = iterationData.getStanceNodeSnapData();
         snappedNodeTransform.set(snapData.getSnappedNodeTransform(stanceNode));

         FootstepNode idealStep = iterationData.getIdealStep();
         FootstepNodeSnapData idealStepSnapData = snapper.snapFootstepNode(idealStep);
         if(idealStepSnapData == null || idealStepSnapData.getSnapTransform().containsNaN())
         {
            FootstepNodeTools.getNodeTransform(idealStep, idealStepTransform);
            idealStepTransform.getTranslation().setZ(snappedNodeTransform.getTranslationZ());
         }
         else
         {
            FootstepNodeTools.getSnappedNodeTransform(idealStep, idealStepSnapData.getSnapTransform(), idealStepTransform);
         }
      }

      public int getIndex()
      {
         return 0;
      }

      public String getX()
      {
         return doubleFormat.format(snappedNodeTransform.getTranslationX());
      }

      public String getY()
      {
         return doubleFormat.format(snappedNodeTransform.getTranslationY());
      }

      public String getZ()
      {
         return doubleFormat.format(snappedNodeTransform.getTranslationZ());
      }

      public String getYaw()
      {
         return doubleFormat.format(snappedNodeTransform.getRotation().getYaw());
      }

      public String getPitch()
      {
         return doubleFormat.format(snappedNodeTransform.getRotation().getPitch());
      }

      public String getRoll()
      {
         return doubleFormat.format(snappedNodeTransform.getRotation().getRoll());
      }
   }

   public class ChildStepProperty
   {
      private final FootstepPlannerEdgeData edgeData;
      private final RigidBodyTransform snappedNodeTransform = new RigidBodyTransform();
      private final RigidBodyTransform snapAndWiggledNodeTransform = new RigidBodyTransform();
      private final boolean expanded;
      private final double stepYaw;

      public ChildStepProperty(FootstepPlannerEdgeData edgeData,
                               boolean expanded)
      {
         this.edgeData = edgeData;
         this.expanded = expanded;

         FootstepNode candidateNode = edgeData.getCandidateNode();
         FootstepNode stanceNode = edgeData.getStanceNode();

         stepYaw = candidateNode.getRobotSide().negateIfLeftSide(AngleTools.computeAngleDifferenceMinusPiToPi(candidateNode.getYaw(), stanceNode.getYaw()));
         FootstepNodeSnapData snapData = edgeData.getCandidateNodeSnapData();

         // TODO take yet another pass at this api, it doesn't read that clearly
         if (snapData.getSnapTransform().containsNaN())
         {
            snappedNodeTransform.setIdentity();
            snapAndWiggledNodeTransform.setIdentity();
         }
         else
         {
            FootstepNodeTools.getSnappedNodeTransform(candidateNode, snapData.getSnapTransform(), snappedNodeTransform);
            snapAndWiggledNodeTransform.set(snapData.getSnappedNodeTransform(candidateNode));
         }
      }

      public int getIndex()
      {
         return edgeData.getStepIndex();
      }

      public String getX()
      {
         return doubleFormat.format(snappedNodeTransform.getTranslationX());
      }

      public String getY()
      {
         return doubleFormat.format(snappedNodeTransform.getTranslationY());
      }

      public String getZ()
      {
         return doubleFormat.format(snappedNodeTransform.getTranslationZ());
      }

      public String getYaw()
      {
         return doubleFormat.format(snappedNodeTransform.getRotation().getYaw());
      }

      public String getPitch()
      {
         return doubleFormat.format(snappedNodeTransform.getRotation().getPitch());
      }

      public String getRoll()
      {
         return doubleFormat.format(snappedNodeTransform.getRotation().getRoll());
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
