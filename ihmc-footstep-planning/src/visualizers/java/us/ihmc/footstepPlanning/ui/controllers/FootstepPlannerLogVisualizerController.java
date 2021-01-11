package us.ihmc.footstepPlanning.ui.controllers;

import javafx.application.Platform;
import javafx.beans.property.ReadOnlyObjectWrapper;
import javafx.beans.value.ChangeListener;
import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.event.Event;
import javafx.fxml.FXML;
import javafx.scene.control.*;
import javafx.scene.control.cell.PropertyValueFactory;
import javafx.scene.input.MouseButton;
import javafx.scene.input.ScrollEvent;
import javafx.scene.layout.Pane;
import javafx.scene.text.Text;
import org.apache.commons.lang3.NotImplementedException;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapData;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstepTools;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepGraphNode;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.log.FootstepPlannerEdgeData;
import us.ihmc.footstepPlanning.log.FootstepPlannerIterationData;
import us.ihmc.footstepPlanning.log.FootstepPlannerLog;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogLoader;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogLoader.LoadRequestType;
import us.ihmc.footstepPlanning.log.*;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogLoader.LoadRequestType;
import us.ihmc.footstepPlanning.swing.DefaultSwingPlannerParameters;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.pathPlanning.graph.structure.GraphEdge;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityGraphHolder;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityMap;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityMapHolder;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.DefaultVisibilityGraphParameters;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.text.DecimalFormat;
import java.util.*;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

public class FootstepPlannerLogVisualizerController
{
   private final ObservableList<ParentStepProperty> parentTableItems = FXCollections.observableArrayList();
   private final ObservableList<ChildStepProperty> childTableItems = FXCollections.observableArrayList();

   private JavaFXMessager messager;
   private FootstepPlannerLog footstepPlannerLog = null;
   private List<FootstepGraphNode> path = new ArrayList<>();
   private final Stack<FootstepGraphNode> parentNodeStack = new Stack<>();
   private final AtomicReference<ChildStepProperty> selectedRow = new AtomicReference<>();

   private List<FootstepPlannerIterationData> iterationDataList;
   private Map<GraphEdge<FootstepGraphNode>, FootstepPlannerEdgeData> edgeDataMap;
   private List<VariableDescriptor> variableDescriptors;
   private FootstepSnapAndWiggler snapper = new FootstepSnapAndWiggler(PlannerTools.createDefaultFootPolygons(), new DefaultFootstepPlannerParameters()); // TODO

   private final AtomicBoolean loadingLog = new AtomicBoolean();
   private final List<VariableDescriptor> variablesToChart = new ArrayList<>();

   private static final List<TableColumn> parentTableDefaultColumns = createDefaultColumns();
   private static final List<TableColumn> childTableDefaultColumns = createDefaultColumns();

   private static final List<String> additionalDefaultColumnsIfPresent = Arrays.asList("stepLength",
                                                                                       "stepWidth",
                                                                                       "stepHeight",
                                                                                       "edgeCost",
                                                                                       "heuristicCost",
                                                                                       "rejectionReason");
   private boolean additionalColumnsLoaded = false;

   @FXML
   private TableView<ParentStepProperty> parentTable;
   @FXML
   private TableView<ChildStepProperty> childTable;
   @FXML
   private TextField searchTextField;
   @FXML
   private ListView<VariableDescriptor> variableListView;

   @FXML
   private Button reset;
   @FXML
   private Button stepInto;
   @FXML
   private Button stepBack;

   @FXML
   private CheckBox showStartOfSwingStep;
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

   @FXML
   private Text iterationRange;
   @FXML
   private Spinner<Integer> iterationLoadSpinner;

   public void attachMessager(JavaFXMessager messager)
   {
      this.messager = messager;
   }

   public void bindControls()
   {
      messager.registerTopicListener(FootstepPlannerMessagerAPI.RequestLoadLog, type -> loadLog(type));

      AtomicReference<PlanarRegionsList> planarRegionData = messager.createInput(FootstepPlannerMessagerAPI.PlanarRegionData);
      messager.registerTopicListener(FootstepPlannerMessagerAPI.GraphData,
                                     graphData -> Platform.runLater(() -> updateGraphData(planarRegionData.get(),
                                                                                          graphData.getLeft(),
                                                                                          graphData.getMiddle(),
                                                                                          graphData.getRight())));

      messager.bindBidirectional(FootstepPlannerMessagerAPI.ShowLoggedStartOfSwingStep, showStartOfSwingStep.selectedProperty(), true);
      messager.bindBidirectional(FootstepPlannerMessagerAPI.ShowLoggedStanceStep, showStanceStep.selectedProperty(), true);
      messager.bindBidirectional(FootstepPlannerMessagerAPI.ShowLoggedUnsnappedCandidateStep, showUnsnappedStep.selectedProperty(), true);
      messager.bindBidirectional(FootstepPlannerMessagerAPI.ShowLoggedSnappedCandidateStep, showSnappedStep.selectedProperty(), true);
      messager.bindBidirectional(FootstepPlannerMessagerAPI.ShowLoggedWiggledCandidateStep, showSnapAndWiggledStep.selectedProperty(), true);
      messager.bindBidirectional(FootstepPlannerMessagerAPI.ShowLoggedIdealStep, showIdealStep.selectedProperty(), true);

      messager.registerTopicListener(FootstepPlannerMessagerAPI.ShowLogGraphics, show ->
      {
         messager.submitMessage(FootstepPlannerMessagerAPI.ShowLoggedStartOfSwingStep, show);
         messager.submitMessage(FootstepPlannerMessagerAPI.ShowLoggedStanceStep, show);
         messager.submitMessage(FootstepPlannerMessagerAPI.ShowLoggedUnsnappedCandidateStep, show);
         messager.submitMessage(FootstepPlannerMessagerAPI.ShowLoggedSnappedCandidateStep, show);
         messager.submitMessage(FootstepPlannerMessagerAPI.ShowLoggedWiggledCandidateStep, show);
         messager.submitMessage(FootstepPlannerMessagerAPI.ShowLoggedIdealStep, show);
      });

      iterationLoadSpinner.setValueFactory(new SpinnerValueFactory.IntegerSpinnerValueFactory(0, Integer.MAX_VALUE, 0, 1));
      iterationLoadSpinner.valueProperty().addListener((obs, oldValue, newValue) -> loadIteration());
   }

   public void onPrimaryStageLoaded()
   {
      clearAndAddDefaultColumns(parentTable, parentTableDefaultColumns);
      clearAndAddDefaultColumns(childTable, childTableDefaultColumns);

      parentTableItems.clear();
      parentTable.setItems(parentTableItems);
      childTableItems.clear();
      childTable.setItems(childTableItems);

      Pane header = (Pane) parentTable.lookup("TableHeaderRow");
      if (header.isVisible())
      {
         header.setMaxHeight(0);
         header.setMinHeight(0);
         header.setPrefHeight(0);
         header.setVisible(false);
      }

      childTable.setRowFactory(tableValue ->
                                        {
                                           TableRow<ChildStepProperty> row = new TableRow<>();
                                           row.setOnMouseClicked(event ->
                                                                 {
                                                                    if (event.getButton() == MouseButton.PRIMARY)
                                                                    {
                                                                       if (event.isControlDown())
                                                                       {
                                                                          // remove column
                                                                          ObservableList<TablePosition> selectedCells = childTable.getSelectionModel()
                                                                                                                                  .getSelectedCells();
                                                                          if (!selectedCells.isEmpty())
                                                                          {
                                                                             String variableName = selectedCells.get(0).getTableColumn().getText();
                                                                             variablesToChart.removeIf(v -> v.getName().equals(variableName));
                                                                             updateTable();
                                                                          }
                                                                       }
                                                                       else if (event.getClickCount() == 2 && (!row.isEmpty()))
                                                                       {
                                                                          // step into node
                                                                          ChildStepProperty rowData = row.getItem();
                                                                          if (!rowData.expanded)
                                                                             return;
                                                                          parentNodeStack.push(rowData.edgeData.getChildNode());
                                                                          updateTable();
                                                                       }
                                                                    }

                                                                    event.consume();
                                                                 });
                                           return row;
                                        });

      parentTable.setRowFactory(tableValue ->
                                         {
                                            TableRow<ParentStepProperty> row = new TableRow<>();
                                            row.setOnMouseClicked(event ->
                                                                  {
                                                                     if (event.getClickCount() == 2)
                                                                        stepBack();
                                                                     event.consume();
                                                                  });
                                            return row;
                                         });

      parentTable.addEventFilter(ScrollEvent.ANY, Event::consume);
      searchTextField.textProperty().addListener((observable, oldValue, newValue) -> search(newValue));
      childTable.getSelectionModel().setCellSelectionEnabled(true);

      variableListView.setOnMouseClicked(event ->
                                         {
                                            if (event.getClickCount() == 2 && !variableListView.getSelectionModel().getSelectedItems().isEmpty())
                                            {
                                               VariableDescriptor variableDescriptor = variableListView.getSelectionModel().getSelectedItems().get(0);
                                               if (!variablesToChart.contains(variableDescriptor))
                                               {
                                                  variablesToChart.add(variableDescriptor);
                                                  updateTable();
                                               }
                                            }
                                            event.consume();
                                         });
   }

   private void search(String searchQuery)
   {
      List<VariableDescriptor> results = FootstepPlannerVisualizerTools.search(searchQuery, variableDescriptors);
      variableListView.getItems().clear();
      for (int i = 0; i < results.size(); i++)
      {
         variableListView.getItems().add(results.get(i));
      }
   }

   public void setContactPointParameters(SideDependentList<List<Point2D>> defaultContactPoints)
   {
      SideDependentList<ConvexPolygon2D> footPolygons = new SideDependentList<>(side ->
                                                                                {
                                                                                   List<Point2D> footPoints = defaultContactPoints.get(side);
                                                                                   return new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(footPoints));
                                                                                });
      snapper = new FootstepSnapAndWiggler(footPolygons, new DefaultFootstepPlannerParameters());
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
      messager.submitMessage(FootstepPlannerMessagerAPI.StartVisibilityMap,
                             new VisibilityMapWrapper(visibilityGraphHolder.getStartMapId(), visibilityGraphHolder.getStartVisibilityMap()));
      messager.submitMessage(FootstepPlannerMessagerAPI.GoalVisibilityMap,
                             new VisibilityMapWrapper(visibilityGraphHolder.getGoalMapId(), visibilityGraphHolder.getGoalVisibilityMap()));
      messager.submitMessage(FootstepPlannerMessagerAPI.InterRegionVisibilityMap,
                             new VisibilityMapWrapper(visibilityGraphHolder.getInterRegionsMapId(), visibilityGraphHolder.getInterRegionsVisibilityMap()));
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
      updateGraphData(planarRegionsList, footstepPlannerLog.getEdgeDataMap(), footstepPlannerLog.getIterationData(), footstepPlannerLog.getVariableDescriptors());
   }

   private static class VisibilityMapWrapper implements VisibilityMapHolder
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
                                Map<GraphEdge<FootstepGraphNode>, FootstepPlannerEdgeData> edgeDataMap,
                                List<FootstepPlannerIterationData> iterationData,
                                List<VariableDescriptor> variableDescriptors)
   {
      this.iterationDataList = iterationData;
      this.edgeDataMap = edgeDataMap;
      this.variableDescriptors = variableDescriptors;
      this.snapper.setPlanarRegions(planarRegionsList);

      parentNodeStack.clear();
      selectedRow.set(null);
      path.clear();

      if (!iterationDataList.isEmpty())
      {
         recursivelyBuildPath(iterationDataList.get(0), iterationDataList, this.edgeDataMap);
         FootstepGraphNode startNode = path.get(0);
         parentNodeStack.push(startNode);
         updateTable();
      }

      iterationRange.setText("0 - " + (iterationData.size() - 1));
      iterationLoadSpinner.getValueFactory().setValue(0);
   }

   private void recursivelyBuildPath(FootstepPlannerIterationData iterationData, List<FootstepPlannerIterationData> iterationDataList, Map<GraphEdge<FootstepGraphNode>, FootstepPlannerEdgeData> edgeDataMap)
   {
      FootstepGraphNode stanceNode = iterationData.getParentNode();
      path.add(stanceNode);

      for (int i = 0; i < iterationData.getChildNodes().size(); i++)
      {
         FootstepGraphNode childNode = iterationData.getChildNodes().get(i);
         if(edgeDataMap.get(new GraphEdge<>(stanceNode, childNode)).isSolutionEdge())
         {
            iterationDataList.stream().filter(data -> data.getParentNode().equals(childNode)).findAny().ifPresent(nextData -> recursivelyBuildPath(nextData, iterationDataList, edgeDataMap));
            return;
         }
      }
   }

   private void updateTable()
   {
      FootstepGraphNode parentNode = parentNodeStack.peek();
      Optional<FootstepPlannerIterationData> iterationDataOptional = iterationDataList.stream().filter(data -> data.getParentNode().equals(parentNode)).findFirst();

      parentTableItems.clear();
      childTableItems.clear();

      if (!iterationDataOptional.isPresent())
      {
         return;
      }

      clearAndAddDefaultColumns(childTable, childTableDefaultColumns);

      if (!additionalColumnsLoaded)
      {
         for (String variableName : additionalDefaultColumnsIfPresent)
         {
            variableDescriptors.stream().filter(v -> v.getName().equalsIgnoreCase(variableName)).findFirst().ifPresent(variablesToChart::add);
         }

         additionalColumnsLoaded = true;
      }

      for (int i = 0; i < variablesToChart.size(); i++)
      {
         VariableDescriptor variableDescriptor = variablesToChart.get(i);
         TableColumn<ChildStepProperty, ?> tableColumn = createTableColumn(variableDescriptor);
         tableColumn.setPrefWidth(125);
         childTable.getColumns().add(tableColumn);
      }

      FootstepPlannerIterationData iterationData = iterationDataOptional.get();
      for (int i = 0; i < iterationData.getChildNodes().size(); i++)
      {
         FootstepGraphNode childNode = iterationData.getChildNodes().get(i);
         FootstepPlannerEdgeData edgeData = edgeDataMap.get(new GraphEdge<>(iterationData.getParentNode(), childNode));
         boolean expanded = iterationDataList.stream().anyMatch(data -> data.getParentNode().equals(childNode));
         ChildStepProperty stepProperty = new ChildStepProperty(edgeData, expanded);
         childTableItems.add(stepProperty);
      }

      ParentStepProperty stepProperty = new ParentStepProperty(iterationData);
      parentTableItems.add(stepProperty);

      childTable.getSortOrder().clear();
      ObservableList<TableColumn<ChildStepProperty, ?>> columns = childTable.getColumns();
      TableColumn<ChildStepProperty, ?> solutionEdgeColumn = columns.get(0);
      TableColumn<ChildStepProperty, ?> expandedColumn = columns.get(1);
      solutionEdgeColumn.setSortType(TableColumn.SortType.DESCENDING);
      expandedColumn.setSortType(TableColumn.SortType.DESCENDING);
      childTable.getSortOrder().add(solutionEdgeColumn);
      childTable.getSortOrder().add(expandedColumn);
      childTable.sort();

      messager.submitMessage(FootstepPlannerMessagerAPI.StartOfSwingStepToVisualize, Pair.of(stepProperty.parentNode.getFirstStep(), stepProperty.startStepSnapData));
      messager.submitMessage(FootstepPlannerMessagerAPI.StanceStepToVisualize, Pair.of(stepProperty.parentNode.getSecondStep(), stepProperty.endStepSnapData));
      messager.submitMessage(FootstepPlannerMessagerAPI.LoggedIdealStep, stepProperty.idealStepTransform);

      childTable.getSelectionModel().selectedItemProperty().addListener(onStepSelected());
      childTable.getSelectionModel().select(0);
      childTable.getFocusModel().focus(0);
   }

   private TableColumn<ChildStepProperty, ?> createTableColumn(VariableDescriptor variableDescriptor)
   {
      int variableIndex = variableDescriptors.indexOf(variableDescriptor);
      switch (variableDescriptor.getType())
      {
         case DOUBLE:
            TableColumn<ChildStepProperty, Double> doubleColumn = new TableColumn<>(variableDescriptor.getName());
            doubleColumn.setCellValueFactory(c -> new ReadOnlyObjectWrapper<>(Double.longBitsToDouble(c.getValue().edgeData.getDataBuffer()[variableIndex])));
            doubleColumn.setCellFactory(c -> new TableCell<ChildStepProperty, Double>()
            {
               @Override
               public void updateItem(final Double value, boolean empty)
               {
                  if (value != null)
                     setText(doubleFormat.format(value));
               }
            });

            return doubleColumn;
         case BOOLEAN:
            TableColumn<ChildStepProperty, Boolean> booleanColumn = new TableColumn<>(variableDescriptor.getName());
            booleanColumn.setCellValueFactory(c -> new ReadOnlyObjectWrapper<>(c.getValue().edgeData.getDataBuffer()[variableIndex] == 1));
            booleanColumn.setCellFactory(c -> new TableCell<ChildStepProperty, Boolean>()
            {
               @Override
               public void updateItem(final Boolean value, boolean empty)
               {
                  if (value != null)
                     setText(Boolean.toString(value));
               }
            });

            return booleanColumn;
         case ENUM:
            TableColumn<ChildStepProperty, Long> enumColumn = new TableColumn<>(variableDescriptor.getName());
            enumColumn.setCellValueFactory(c -> new ReadOnlyObjectWrapper<>(c.getValue().edgeData.getDataBuffer()[variableIndex]));
            enumColumn.setCellFactory(c -> new TableCell<ChildStepProperty, Long>()
            {
               @Override
               public void updateItem(final Long value, boolean empty)
               {
                  if (value != null)
                     setText(value == -1 ? "null" : variableDescriptor.getEnumValues()[(int) value.longValue()]);
               }
            });

            return enumColumn;
         case LONG:
         case INTEGER:
         default:
            TableColumn<ChildStepProperty, Long> longColumn = new TableColumn<>(variableDescriptor.getName());
            longColumn.setCellValueFactory(c -> new ReadOnlyObjectWrapper<>(c.getValue().edgeData.getDataBuffer()[variableIndex]));
            longColumn.setCellFactory(c -> new TableCell<ChildStepProperty, Long>()
            {
               @Override
               public void updateItem(final Long value, boolean empty)
               {
                  if (value != null)
                     setText(value.toString());
               }
            });

            return longColumn;
      }
   }

   private ChangeListener<ChildStepProperty> onStepSelected()
   {
      return (observer, oldValue, newValue) ->
      {
         if (newValue != null)
         {
            messager.submitMessage(FootstepPlannerMessagerAPI.TouchdownStepToVisualize, Pair.of(newValue.graphNode.getSecondStep(), newValue.edgeData.getEndStepSnapData()));
            selectedRow.set(newValue);
         }
      };
   }

   public void stepInto()
   {
      ChildStepProperty selectedRow = this.selectedRow.get();
      if (selectedRow != null && selectedRow.expanded)
      {
         parentNodeStack.push(selectedRow.edgeData.getChildNode());
         updateTable();
      }
   }

   public void stepBack()
   {
      if (parentNodeStack.size() <= 1)
      {
         return;
      }

      parentNodeStack.pop();
      updateTable();
   }

   public void loadIteration()
   {
      Integer iterationToLoad = iterationLoadSpinner.getValue();

      if (iterationToLoad < iterationDataList.size())
      {
         FootstepPlannerIterationData iterationData = iterationDataList.get(iterationToLoad);
         parentNodeStack.push(iterationData.getParentNode());
         updateTable();
      }
   }

   private static List<TableColumn> createDefaultColumns()
   {
      TableColumn<ChildStepProperty, String> solutionColumn = new TableColumn<>("Solution");
      TableColumn<ChildStepProperty, String> expandedColumn = new TableColumn<>("Expanded");
      TableColumn<ChildStepProperty, String> xIndexColumn = new TableColumn<>("X Index");
      TableColumn<ChildStepProperty, String> yIndexColumn = new TableColumn<>("Y Index");
      TableColumn<ChildStepProperty, String> yawIndexColumn = new TableColumn<>("Yaw Index");
      TableColumn<ChildStepProperty, String> robotSideColumn = new TableColumn<>("Side");

      solutionColumn.setCellValueFactory(new PropertyValueFactory<>("solution"));
      expandedColumn.setCellValueFactory(new PropertyValueFactory<>("expanded"));
      xIndexColumn.setCellValueFactory(new PropertyValueFactory<>("xIndex"));
      yIndexColumn.setCellValueFactory(new PropertyValueFactory<>("yIndex"));
      yawIndexColumn.setCellValueFactory(new PropertyValueFactory<>("yawIndex"));
      robotSideColumn.setCellValueFactory(new PropertyValueFactory<>("side"));

      solutionColumn.setPrefWidth(90);
      expandedColumn.setPrefWidth(90);
      xIndexColumn.setPrefWidth(90);
      yIndexColumn.setPrefWidth(90);
      yawIndexColumn.setPrefWidth(90);
      robotSideColumn.setPrefWidth(90);

      List<TableColumn> defaultColumns = new ArrayList<>();
      defaultColumns.add(solutionColumn);
      defaultColumns.add(expandedColumn);
      defaultColumns.add(xIndexColumn);
      defaultColumns.add(yIndexColumn);
      defaultColumns.add(yawIndexColumn);
      defaultColumns.add(robotSideColumn);

      return defaultColumns;
   }

   private static void clearAndAddDefaultColumns(TableView tableView, List<TableColumn> tableColumns)
   {
      tableView.getColumns().clear();
      tableView.getColumns().addAll(tableColumns);
   }

   public class ParentStepProperty
   {
      private final FootstepGraphNode parentNode;
      private final FootstepSnapData endStepSnapData, startStepSnapData;
      private final RigidBodyTransform endStepTransform = new RigidBodyTransform();
      private final RigidBodyTransform startStepTransform = new RigidBodyTransform();
      private final RigidBodyTransform idealStepTransform = new RigidBodyTransform();

      public ParentStepProperty(FootstepPlannerIterationData iterationData)
      {
         this.parentNode = iterationData.getParentNode();
         this.endStepSnapData = iterationData.getParentEndSnapData();
         this.startStepSnapData = iterationData.getParentStartSnapData();
         endStepTransform.set(endStepSnapData.getSnappedStepTransform(parentNode.getSecondStep()));
         startStepTransform.set(startStepSnapData.getSnappedStepTransform(parentNode.getFirstStep()));

         DiscreteFootstep idealStep = iterationData.getIdealChildNode().getSecondStep();
         FootstepSnapData idealStepSnapData = snapper.snapFootstep(idealStep);
         if(idealStepSnapData == null || idealStepSnapData.getSnapTransform().containsNaN())
         {
            DiscreteFootstepTools.getStepTransform(idealStep, idealStepTransform);
            idealStepTransform.getTranslation().setZ(idealStepTransform.getTranslationZ());
         }
         else
         {
            DiscreteFootstepTools.getSnappedStepTransform(idealStep, idealStepSnapData.getSnapTransform(), idealStepTransform);
         }
      }

      public String getSolution()
      {
         return "";
      }

      public String getExpanded()
      {
         return "";
      }

      public String getXIndex()
      {
         return Integer.toString(parentNode.getSecondStep().getXIndex());
      }

      public String getYIndex()
      {
         return Integer.toString(parentNode.getSecondStep().getYIndex());
      }

      public String getYawIndex()
      {
         return Integer.toString(parentNode.getSecondStep().getYawIndex());
      }

      public String getSide()
      {
         return parentNode.getSecondStep().getRobotSide().toString();
      }
   }

   public class ChildStepProperty
   {
      private final FootstepGraphNode graphNode;
      private final FootstepPlannerEdgeData edgeData;
      private final RigidBodyTransform snappedEndStepTransform = new RigidBodyTransform();
      private final RigidBodyTransform snapAndWiggledEndStepTransform = new RigidBodyTransform();
      private final boolean expanded;

      public ChildStepProperty(FootstepPlannerEdgeData edgeData,
                               boolean expanded)
      {
         this.edgeData = edgeData;
         this.expanded = expanded;

         graphNode = edgeData.getChildNode();
         FootstepSnapData snapData = edgeData.getEndStepSnapData();

         // TODO take yet another pass at this api, it doesn't read that clearly
         if (snapData.getSnapTransform().containsNaN())
         {
            snappedEndStepTransform.setIdentity();
            snapAndWiggledEndStepTransform.setIdentity();
         }
         else
         {
            DiscreteFootstepTools.getSnappedStepTransform(graphNode.getSecondStep(), snapData.getSnapTransform(), snappedEndStepTransform);
            snapAndWiggledEndStepTransform.set(snapData.getSnappedStepTransform(graphNode.getSecondStep()));
         }
      }

      public String getSolution()
      {
         return Boolean.toString(edgeData.isSolutionEdge());
      }

      public String getExpanded()
      {
         return Boolean.toString(expanded);
      }

      public String getXIndex()
      {
         return Integer.toString(graphNode.getSecondStep().getXIndex());
      }

      public String getYIndex()
      {
         return Integer.toString(graphNode.getSecondStep().getYIndex());
      }

      public String getYawIndex()
      {
         return Integer.toString(graphNode.getSecondStep().getYawIndex());
      }

      public String getSide()
      {
         return graphNode.getSecondStep().getRobotSide().toString();
      }
   }

   private static final DecimalFormat doubleFormat = new DecimalFormat("#0.00000");

   private static String formatValue(long value, VariableDescriptor variableDescriptor)
   {
      switch (variableDescriptor.getType())
      {
         case DOUBLE:
            return doubleFormat.format(Double.longBitsToDouble(value));
         case BOOLEAN:
            return Boolean.toString(value == 1);
         case ENUM:
            return value == -1 ? "null" : variableDescriptor.getEnumValues()[(int) value];
         case LONG:
         case INTEGER:
         default:
            return Long.toString(value);
      }
   }
}
