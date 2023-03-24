package us.ihmc.footstepPlanning.ui.controllers;

import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.ShowBodyPathPlanData;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Stack;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import org.apache.commons.lang3.tuple.Pair;
import org.apache.commons.lang3.tuple.Triple;

import javafx.beans.property.ReadOnlyObjectWrapper;
import javafx.beans.value.ChangeListener;
import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.event.Event;
import javafx.fxml.FXML;
import javafx.scene.control.CheckBox;
import javafx.scene.control.ListView;
import javafx.scene.control.TableCell;
import javafx.scene.control.TableColumn;
import javafx.scene.control.TablePosition;
import javafx.scene.control.TableRow;
import javafx.scene.control.TableView;
import javafx.scene.control.TextField;
import javafx.scene.control.cell.PropertyValueFactory;
import javafx.scene.input.MouseButton;
import javafx.scene.input.ScrollEvent;
import javafx.scene.layout.Pane;
import us.ihmc.footstepPlanning.bodyPath.BodyPathLatticePoint;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.log.AStarBodyPathEdgeData;
import us.ihmc.footstepPlanning.log.AStarBodyPathIterationData;
import us.ihmc.footstepPlanning.log.FootstepPlannerLog;
import us.ihmc.footstepPlanning.log.VariableDescriptor;
import us.ihmc.messager.javafx.JavaFXMessager;
import us.ihmc.pathPlanning.graph.structure.GraphEdge;

public class BodyPathLogVisualizerController
{
   private final ObservableList<ParentStepProperty> parentTableItems = FXCollections.observableArrayList();
   private final ObservableList<ChildStepProperty> childTableItems = FXCollections.observableArrayList();

   private JavaFXMessager messager;
   private FootstepPlannerLog footstepPlannerLog = null;
   private List<BodyPathLatticePoint> path = new ArrayList<>();
   private final Stack<BodyPathLatticePoint> parentNodeStack = new Stack<>();
   private final AtomicReference<ChildStepProperty> selectedRow = new AtomicReference<>();

   private List<AStarBodyPathIterationData> iterationDataList;
   private Map<GraphEdge<BodyPathLatticePoint>, AStarBodyPathEdgeData> edgeDataMap;
   private List<VariableDescriptor> variableDescriptors;

   private final AtomicBoolean loadingLog = new AtomicBoolean();
   private final List<VariableDescriptor> variablesToChart = new ArrayList<>();

   private static final List<TableColumn> parentTableDefaultColumns = createDefaultColumns();
   private static final List<TableColumn> childTableDefaultColumns = createDefaultColumns();

   private static final List<String> additionalDefaultColumnsIfPresent = Arrays.asList("rejectionReason", "snapHeight", "leftTraversibility", "rightTraversibility", "traversibilitySide");
   private boolean additionalColumnsLoaded = false;

   @FXML
   private CheckBox showBodyPathPlanData;
   @FXML
   private TableView<ParentStepProperty> parentTable;
   @FXML
   private TableView<ChildStepProperty> childTable;
   @FXML
   private TextField searchTextField;
   @FXML
   private ListView<VariableDescriptor> variableListView;

   public void attachMessager(JavaFXMessager messager)
   {
      this.messager = messager;
   }

   public void bindControls()
   {
      messager.bindBidirectional(ShowBodyPathPlanData, showBodyPathPlanData.selectedProperty(), false);
      messager.addTopicListener(FootstepPlannerMessagerAPI.BodyPathGraphData, this::updateGraphData);
   }

   private void updateGraphData(Triple<Map<GraphEdge<BodyPathLatticePoint>, AStarBodyPathEdgeData>, List<AStarBodyPathIterationData>, List<VariableDescriptor>> graphData)
   {
      this.edgeDataMap = graphData.getLeft();
      this.iterationDataList = graphData.getMiddle();
      this.variableDescriptors = graphData.getRight();

      parentNodeStack.clear();
      selectedRow.set(null);
      path.clear();

      if (!iterationDataList.isEmpty())
      {
         recursivelyBuildPath(iterationDataList.get(0), iterationDataList, edgeDataMap);
         BodyPathLatticePoint startNode = path.get(0);
         parentNodeStack.push(startNode);
         updateTable();
      }
   }

   private void recursivelyBuildPath(AStarBodyPathIterationData iterationData, List<AStarBodyPathIterationData> iterationDataList, Map<GraphEdge<BodyPathLatticePoint>, AStarBodyPathEdgeData> edgeDataMap)
   {
      BodyPathLatticePoint stanceNode = iterationData.getParentNode();
      path.add(stanceNode);

      for (int i = 0; i < iterationData.getChildNodes().size(); i++)
      {
         BodyPathLatticePoint childNode = iterationData.getChildNodes().get(i);
         if(edgeDataMap.get(new GraphEdge<>(stanceNode, childNode)).isSolutionEdge())
         {
            iterationDataList.stream().filter(data -> data.getParentNode().equals(childNode)).findAny().ifPresent(nextData -> recursivelyBuildPath(nextData, iterationDataList, edgeDataMap));
            return;
         }
      }
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

   private void updateTable()
   {
      BodyPathLatticePoint parentNode = parentNodeStack.peek();
      Optional<AStarBodyPathIterationData> iterationDataOptional = iterationDataList.stream().filter(data -> data.getParentNode().equals(parentNode)).findFirst();

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

      AStarBodyPathIterationData iterationData = iterationDataOptional.get();
      for (int i = 0; i < iterationData.getChildNodes().size(); i++)
      {
         BodyPathLatticePoint childNode = iterationData.getChildNodes().get(i);
         AStarBodyPathEdgeData edgeData = edgeDataMap.get(new GraphEdge<>(iterationData.getParentNode(), childNode));
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

      messager.submitMessage(FootstepPlannerMessagerAPI.BodyPathStartNodeToVisualize, Pair.of(stepProperty.parentNode, stepProperty.height));
      childTable.getSelectionModel().select(0);
      childTable.getFocusModel().focus(0);

      childTable.getSelectionModel().selectedItemProperty().addListener(onStepSelected());
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
            messager.submitMessage(FootstepPlannerMessagerAPI.BodyPathCandidateNodeToVisualize, Pair.of(newValue.node, newValue.height));
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

   private static List<TableColumn> createDefaultColumns()
   {
      TableColumn<ChildStepProperty, String> solutionColumn = new TableColumn<>("Solution");
      TableColumn<ChildStepProperty, String> expandedColumn = new TableColumn<>("Expanded");
      TableColumn<ChildStepProperty, String> xIndexColumn = new TableColumn<>("X Index");
      TableColumn<ChildStepProperty, String> yIndexColumn = new TableColumn<>("Y Index");

      solutionColumn.setCellValueFactory(new PropertyValueFactory<>("solution"));
      expandedColumn.setCellValueFactory(new PropertyValueFactory<>("expanded"));
      xIndexColumn.setCellValueFactory(new PropertyValueFactory<>("xIndex"));
      yIndexColumn.setCellValueFactory(new PropertyValueFactory<>("yIndex"));

      solutionColumn.setPrefWidth(80);
      expandedColumn.setPrefWidth(80);
      xIndexColumn.setPrefWidth(80);
      yIndexColumn.setPrefWidth(80);

      List<TableColumn> defaultColumns = new ArrayList<>();
      defaultColumns.add(solutionColumn);
      defaultColumns.add(expandedColumn);
      defaultColumns.add(xIndexColumn);
      defaultColumns.add(yIndexColumn);

      return defaultColumns;
   }

   private static void clearAndAddDefaultColumns(TableView tableView, List<TableColumn> tableColumns)
   {
      tableView.getColumns().clear();
      tableView.getColumns().addAll(tableColumns);
   }

   public class ParentStepProperty
   {
      private final BodyPathLatticePoint parentNode;
      private final double height;

      public ParentStepProperty(AStarBodyPathIterationData iterationData)
      {
         this.parentNode = iterationData.getParentNode();
         this.height = iterationData.getParentNodeHeight();
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
         return Integer.toString(parentNode.getXIndex());
      }

      public String getYIndex()
      {
         return Integer.toString(parentNode.getYIndex());
      }
   }

   public class ChildStepProperty
   {
      private final BodyPathLatticePoint node;
      private final double height;
      private final boolean solution;
      private final boolean expanded;
      private final AStarBodyPathEdgeData edgeData;

      public ChildStepProperty(AStarBodyPathEdgeData edgeData, boolean expanded)
      {
         this.node = edgeData.getChildNode();
         this.expanded = expanded;
         this.height = edgeData.getChildSnapHeight();
         this.solution = edgeData.isSolutionEdge();
         this.edgeData = edgeData;
      }

      public String getXIndex()
      {
         return Integer.toString(node.getXIndex());
      }

      public String getYIndex()
      {
         return Integer.toString(node.getYIndex());
      }

      public double getHeight()
      {
         return height;
      }

      public boolean isSolution()
      {
         return solution;
      }

      public boolean isExpanded()
      {
         return expanded;
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
