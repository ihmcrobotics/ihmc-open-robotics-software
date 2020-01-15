package us.ihmc.valkyrie.planner.ui;

import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.event.Event;
import javafx.fxml.FXML;
import javafx.scene.control.*;
import javafx.scene.control.TableColumn.SortType;
import javafx.scene.control.cell.PropertyValueFactory;
import javafx.scene.input.KeyCode;
import javafx.scene.input.KeyEvent;
import javafx.scene.input.MouseButton;
import javafx.scene.input.ScrollEvent;
import javafx.scene.layout.Pane;
import javafx.util.Callback;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapperReadOnly;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.messager.Messager;
import us.ihmc.pathPlanning.graph.structure.DirectedGraph;
import us.ihmc.pathPlanning.graph.structure.GraphEdge;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.referenceFrames.TransformReferenceFrame;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
import us.ihmc.valkyrie.planner.ValkyrieAStarFootstepPlanner;
import us.ihmc.valkyrie.planner.ValkyrieFootstepValidityChecker.StepRejectionReason;

import java.text.DecimalFormat;
import java.util.*;
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
                                                                       parentStepStack.push(rowData.node);
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
                                                                     if (event.getClickCount() == 2 && (!row.isEmpty()))
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
      FootstepNodeSnapperReadOnly snapper = planner.getSnapper();
      Map<GraphEdge<FootstepNode>, StepRejectionReason> rejectionReasonMap = planner.getRejectionReasonMap();

      FootstepNodeSnapData parentSnapData = snapper.getSnapData(parentNode);
      HashSet<GraphEdge<FootstepNode>> startEdges = graph.getOutgoingEdges().get(parentNode);

      if(startEdges == null)
      {
         parentStepStack.pop();
         return;
      }

      parentTableItems.clear();
      childTableItems.clear();

      for (GraphEdge<FootstepNode> edge : startEdges)
      {
         StepRejectionReason rejectionReason = rejectionReasonMap.get(edge);
         FootstepNodeSnapData childSnapData = snapper.getSnapData(edge.getEndNode());
         double edgeCost = graph.getCostFromStart(edge.getEndNode()) - graph.getCostFromStart(edge.getStartNode());
         double heuristicCost = planner.getHeuristics().compute(edge.getEndNode());
         double totalCost = graph.getCostFromStart(edge.getStartNode()) + edgeCost + heuristicCost;
         boolean expanded = graph.getOutgoingEdges().containsKey(edge.getEndNode());
         boolean solution = path.contains(edge.getEndNode());
         ChildStepProperty stepProperty = new ChildStepProperty(edge.getEndNode(), parentNode, childSnapData, parentSnapData, rejectionReason, edgeCost, heuristicCost, totalCost, expanded, solution);
         childTableItems.add(stepProperty);
      }

      ParentStepProperty stepProperty = new ParentStepProperty(parentNode, parentSnapData);
      parentTableItems.add(stepProperty);

      debugChildStepTable.getSortOrder().clear();
//      debugChildStepTable.getSortOrder().add(childColumnHolder.totalCostColumn);
//      childColumnHolder.totalCostColumn.setSortType(SortType.ASCENDING);
//      debugChildStepTable.sort();

      messager.submitMessage(ValkyriePlannerMessagerAPI.parentDebugStep, Pair.of(stepProperty.transform, stepProperty.snapData.getCroppedFoothold()));

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
         parentStepStack.push(selectedRow.node);
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

      public ParentStepProperty(FootstepNode node, FootstepNodeSnapData snapData)
      {
         this.node = node;
         this.snapData = snapData;
         FootstepNodeTools.getSnappedNodeTransform(node, snapData.getSnapTransform(), transform);
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

   private final TransformReferenceFrame parentFootFrame = new TransformReferenceFrame("parentFootFrame", ReferenceFrame.getWorldFrame());
   private final TransformReferenceFrame childFootFrame = new TransformReferenceFrame("childFootFrame", ReferenceFrame.getWorldFrame());
   private final ZUpFrame parentFootZUpFrame = new ZUpFrame(ReferenceFrame.getWorldFrame(), parentFootFrame, "parentFootZUpFrame");
   private final FramePoint3D stepPosition = new FramePoint3D();

   public class ChildStepProperty
   {
      private final FootstepNode node;
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

      public ChildStepProperty(FootstepNode childNode,
                               FootstepNode parentNode,
                               FootstepNodeSnapData childSnapData,
                               FootstepNodeSnapData parentSnapData,
                               StepRejectionReason rejectionReason,
                               double edgeCost,
                               double heuristicCost,
                               double totalCost,
                               boolean expanded,
                               boolean solution)
      {
         this.node = childNode;
         this.snapData = childSnapData;
         this.rejectionReason = rejectionReason;
         this.edgeCost = edgeCost;
         this.heuristicCost = heuristicCost;
         this.totalCost = totalCost;
         this.expanded = expanded;
         this.solution = solution;

         if (childSnapData.getSnapTransform().containsNaN())
         {
            FootstepNodeTools.getSnappedNodeTransform(node, new RigidBodyTransform(), transform);

            stepWidth = Double.NaN;
            stepLength = Double.NaN;
            stepHeight = Double.NaN;
            stepReach = Double.NaN;
            stepYaw = Double.NaN;
         }
         else
         {
            FootstepNodeTools.getSnappedNodeTransform(node, childSnapData.getSnapTransform(), transform);

            parentFootFrame.setTransformAndUpdate(parentSnapData.getOrComputeSnappedNodeTransform(parentNode));
            childFootFrame.setTransformAndUpdate(childSnapData.getOrComputeSnappedNodeTransform(childNode));
            parentFootZUpFrame.update();
            stepPosition.setToZero(childFootFrame);
            stepPosition.changeFrame(parentFootZUpFrame);

            stepWidth = childNode.getRobotSide().negateIfRightSide(stepPosition.getY());
            stepLength = stepPosition.getX();
            stepReach = EuclidGeometryTools.pythagorasGetHypotenuse(Math.abs(stepPosition.getX()), Math.abs(stepWidth - planner.getParameters().getIdealFootstepWidth()));
            stepHeight = stepPosition.getZ();
            stepYaw = AngleTools.trimAngleMinusPiToPi(childNode.getRobotSide().negateIfLeftSide(childNode.getYaw() - parentNode.getYaw()));
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
