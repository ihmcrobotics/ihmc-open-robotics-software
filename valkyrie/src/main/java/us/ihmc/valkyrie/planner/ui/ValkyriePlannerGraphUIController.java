package us.ihmc.valkyrie.planner.ui;

import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.event.Event;
import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.control.TableColumn;
import javafx.scene.control.TableColumn.SortType;
import javafx.scene.control.TableRow;
import javafx.scene.control.TableView;
import javafx.scene.control.cell.PropertyValueFactory;
import javafx.scene.input.ScrollEvent;
import javafx.scene.layout.Pane;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapperReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.messager.Messager;
import us.ihmc.pathPlanning.graph.structure.DirectedGraph;
import us.ihmc.pathPlanning.graph.structure.GraphEdge;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.valkyrie.planner.ValkyrieAStarFootstepPlanner;
import us.ihmc.valkyrie.planner.ValkyrieFootstepValidityChecker.StepRejectionReason;
import us.ihmc.valkyrie.planner.log.ValkyriePlannerEdgeData;
import us.ihmc.valkyrie.planner.log.ValkyriePlannerIterationData;

import java.text.DecimalFormat;
import java.util.HashSet;
import java.util.List;
import java.util.Stack;
import java.util.concurrent.atomic.AtomicReference;

public class ValkyriePlannerGraphUIController
{
   private ValkyrieAStarFootstepPlanner planner;
   private final ObservableList<ChildStepProperty> childTableItems = FXCollections.observableArrayList();
   private final ObservableList<ParentStepProperty> parentTableItems = FXCollections.observableArrayList();
   private TableColumnHolder parentColumnHolder;
   private TableColumnHolder childColumnHolder;
   private Messager messager;
   private List<FootstepNode> path;
   private final Stack<FootstepNode> parentStepStack = new Stack<>();
   private final AtomicReference<ChildStepProperty> selectedRow = new AtomicReference<>();

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

   void setPlanner(ValkyrieAStarFootstepPlanner planner)
   {
      this.planner = planner;
   }

   void setMessager(Messager messager)
   {
      this.messager = messager;
   }

   void setup()
   {
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
   }

   public void initialize()
   {
      debugChildStepTable.setRowFactory(tableValue ->
                                        {
                                           TableRow<ChildStepProperty> row = new TableRow<>();
                                           row.setOnMouseClicked(event ->
                                                                 {
                                                                    if (event.getClickCount() == 2 && (!row.isEmpty()))
                                                                    {
                                                                       ChildStepProperty rowData = row.getItem();
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

   public void reset()
   {
      parentStepStack.clear();
      selectedRow.set(null);

      FootstepNode endNode = planner.getEndNode();
      path = planner.getInternalPlanner().getGraph().getPathFromStart(endNode);
      FootstepNode startNode = path.get(0);
      parentStepStack.push(startNode);
      updateTable();
   }

   private void updateTable()
   {
      FootstepNode parentNode = parentStepStack.peek();
      DirectedGraph<FootstepNode> graph = planner.getInternalPlanner().getGraph();
      FootstepNodeSnapper snapper = planner.getSnapper();
      FootstepNode idealStep = planner.getIterationData().stream().filter(iterationData -> iterationData.getStanceNode().equals(parentNode)).findFirst().get().getIdealStep();

      FootstepNodeSnapData parentSnapData = snapper.getSnapData(parentNode);
      HashSet<GraphEdge<FootstepNode>> edges = graph.getOutgoingEdges().get(parentNode);

      if(edges == null)
      {
         parentStepStack.pop();
         return;
      }

      parentTableItems.clear();
      childTableItems.clear();

      for (GraphEdge<FootstepNode> edge : edges)
      {
         FootstepNodeSnapData childSnapData = snapper.getSnapData(edge.getEndNode());
         double edgeCost = graph.getCostFromStart(edge.getEndNode()) - graph.getCostFromStart(edge.getStartNode());
         double heuristicCost = planner.getHeuristics().compute(edge.getEndNode());
         ValkyriePlannerEdgeData edgeData = planner.getEdgeDataMap().get(edge);
         double totalCost = graph.getCostFromStart(edge.getStartNode()) + edgeCost + heuristicCost;
         boolean expanded = graph.getOutgoingEdges().containsKey(edge.getEndNode());
         boolean solution = path.get(parentStepStack.size()).equals(edge.getEndNode());
         ChildStepProperty stepProperty = new ChildStepProperty(edgeData, childSnapData, edgeCost, heuristicCost, totalCost, expanded, solution);
         childTableItems.add(stepProperty);
      }

      ParentStepProperty stepProperty = new ParentStepProperty(parentNode, parentSnapData, idealStep, snapper.snapFootstepNode(idealStep));
      parentTableItems.add(stepProperty);

      debugChildStepTable.getSortOrder().clear();
      debugChildStepTable.getSortOrder().add(childColumnHolder.solutionStep);
      childColumnHolder.solutionStep.setSortType(SortType.DESCENDING);
      debugChildStepTable.getSortOrder().add(childColumnHolder.totalCostColumn);
      childColumnHolder.totalCostColumn.setSortType(SortType.ASCENDING);
      debugChildStepTable.sort();

      messager.submitMessage(ValkyriePlannerMessagerAPI.parentDebugStep, Pair.of(stepProperty.transform, stepProperty.snapData.getCroppedFoothold()));
      messager.submitMessage(ValkyriePlannerMessagerAPI.idealDebugStep, stepProperty.idealStepTransform);

      debugChildStepTable.getSelectionModel().selectedItemProperty().addListener((observer, oldValue, newValue) ->
                                                                                 {
                                                                                    if (newValue != null)
                                                                                    {
                                                                                       ChildStepProperty property = (ChildStepProperty) newValue;
                                                                                       messager.submitMessage(ValkyriePlannerMessagerAPI.childDebugStep,
                                                                                                              Pair.of(property.transform,
                                                                                                                      property.snapData.getCroppedFoothold()));
                                                                                       selectedRow.set(property);
                                                                                    }
                                                                                 });

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
      if (parentStepStack.size() < 2)
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
      private final FootstepNode node;
      private final FootstepNodeSnapData snapData;
      private final RigidBodyTransform transform = new RigidBodyTransform();
      private final RigidBodyTransform idealStepTransform = new RigidBodyTransform();

      public ParentStepProperty(FootstepNode node, FootstepNodeSnapData snapData, FootstepNode idealStep, FootstepNodeSnapData idealStepSnapData)
      {
         this.node = node;
         this.snapData = snapData;
         FootstepNodeTools.getSnappedNodeTransform(node, snapData.getSnapTransform(), transform);

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
      private final ValkyriePlannerEdgeData edgeData;
      private final RigidBodyTransform transform = new RigidBodyTransform();
      private final FootstepNodeSnapData snapData;
      private final StepRejectionReason rejectionReason;
      private final double edgeCost;
      private final double heuristicCost;
      private final double totalCost;
      private final boolean expanded;
      private final double stepWidth;
      private final double stepLength;
      private final double stepHeight;
      private final double stepReach;
      private final double stepYaw;
      private final boolean solution;

      public ChildStepProperty(ValkyriePlannerEdgeData edgeData,
                               FootstepNodeSnapData childSnapData,
                               double edgeCost,
                               double heuristicCost,
                               double totalCost,
                               boolean expanded,
                               boolean solution)
      {
         this.edgeData = edgeData;
         this.snapData = childSnapData;
         this.edgeCost = edgeCost;
         this.heuristicCost = heuristicCost;
         this.totalCost = totalCost;
         this.expanded = expanded;
         this.solution = solution;
         this.rejectionReason = edgeData.getRejectionReason();

         FootstepNode candidateNode = edgeData.getCandidateNode();
         FootstepNode stanceNode = edgeData.getCandidateNode();

         if (childSnapData.getSnapTransform().containsNaN())
         {
            FootstepNodeTools.getSnappedNodeTransform(candidateNode, new RigidBodyTransform(), transform);

            stepWidth = Double.NaN;
            stepLength = Double.NaN;
            stepHeight = Double.NaN;
            stepReach = Double.NaN;
            stepYaw = Double.NaN;
         }
         else
         {
            FootstepNodeTools.getSnappedNodeTransform(candidateNode, childSnapData.getSnapTransform(), transform);

            stepWidth = edgeData.getStepWidth();
            stepLength = edgeData.getStepLength();
            stepHeight = edgeData.getStepHeight();
            stepReach = edgeData.getStepReach();
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
         return doubleFormat.format(stepWidth);
      }

      public String getLength()
      {
         return doubleFormat.format(stepLength);
      }

      public String getHeight()
      {
         return doubleFormat.format(stepHeight);
      }

      public String getReach()
      {
         return doubleFormat.format(stepReach);
      }

      public String getStepYaw()
      {
         return doubleFormat.format(stepYaw);
      }

      public String getEdgeCost()
      {
         return doubleFormat.format(edgeCost);
      }

      public String getHeuristicCost()
      {
         return doubleFormat.format(heuristicCost);
      }

      public String getTotalCost()
      {
         return doubleFormat.format(totalCost);
      }

      public String getExpanded()
      {
         return Boolean.toString(expanded);
      }

      public String getRejectionReason()
      {
         return rejectionReason == null ? "" : rejectionReason.toString();
      }

      public String getSolution()
      {
         return Boolean.toString(solution);
      }
   }
}
