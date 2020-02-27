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
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.SimplePlanarRegionFootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.messager.Messager;
import us.ihmc.pathPlanning.graph.structure.GraphEdge;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.planner.log.ValkyriePlannerEdgeData;
import us.ihmc.valkyrie.planner.log.ValkyriePlannerIterationData;

import java.text.DecimalFormat;
import java.util.*;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.valkyrie.planner.ValkyrieAStarFootstepPlanner.createFootPolygons;

public class ValkyriePlannerDebuggerController
{
   private final ObservableList<ChildStepProperty> childTableItems = FXCollections.observableArrayList();
   private final ObservableList<ParentStepProperty> parentTableItems = FXCollections.observableArrayList();
   private TableColumnHolder parentColumnHolder;
   private TableColumnHolder childColumnHolder;
   private Messager messager;
   private List<FootstepNode> path = new ArrayList<>();
   private final Stack<FootstepNode> parentStepStack = new Stack<>();
   private final AtomicReference<ChildStepProperty> selectedRow = new AtomicReference<>();

   private List<ValkyriePlannerIterationData> iterationDataList;
   private Map<GraphEdge<FootstepNode>, ValkyriePlannerEdgeData> edgeDataMap;
   private FootstepNodeSnapper snapper;

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

   void setMessager(Messager messager)
   {
      this.messager = messager;
   }

   void setup()
   {
      snapper = new SimplePlanarRegionFootstepNodeSnapper(createFootPolygons(new ValkyrieRobotModel(RobotTarget.SCS)));

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

   public void reset(List<ValkyriePlannerIterationData> iterationDataList, Map<GraphEdge<FootstepNode>, ValkyriePlannerEdgeData> edgeDataMap, PlanarRegionsList planarRegionsList)
   {
      this.iterationDataList = iterationDataList;
      this.edgeDataMap = edgeDataMap;
      this.snapper.setPlanarRegions(planarRegionsList);

      parentStepStack.clear();
      selectedRow.set(null);

      path.clear();
      recursivelyBuildPath(iterationDataList.get(0), iterationDataList, edgeDataMap);

      FootstepNode startNode = path.get(0);
      parentStepStack.push(startNode);
      updateTable();
   }

   private void recursivelyBuildPath(ValkyriePlannerIterationData iterationData, List<ValkyriePlannerIterationData> iterationDataList, Map<GraphEdge<FootstepNode>, ValkyriePlannerEdgeData> edgeDataMap)
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
      Optional<ValkyriePlannerIterationData> iterationDataOptional = iterationDataList.stream().filter(data -> data.getStanceNode().equals(parentNode)).findFirst();

      ValkyriePlannerIterationData iterationData = iterationDataOptional.get();

      parentTableItems.clear();
      childTableItems.clear();

      for (int i = 0; i < iterationData.getChildNodes().size(); i++)
      {
         FootstepNode childNode = iterationData.getChildNodes().get(i);
         ValkyriePlannerEdgeData edgeData = edgeDataMap.get(new GraphEdge<>(iterationData.getStanceNode(), childNode));
         boolean expanded = iterationDataList.stream().anyMatch(data -> data.getStanceNode().equals(childNode));
         ChildStepProperty stepProperty = new ChildStepProperty(edgeData, expanded);
         childTableItems.add(stepProperty);
      }

      ParentStepProperty stepProperty = new ParentStepProperty(iterationData);
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
                                                                                                                      property.edgeData.getCandidateNodeSnapData().getCroppedFoothold()));
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
      private final FootstepNode stanceNode;
      private final FootstepNodeSnapData snapData;
      private final RigidBodyTransform transform = new RigidBodyTransform();
      private final RigidBodyTransform idealStepTransform = new RigidBodyTransform();

      public ParentStepProperty(ValkyriePlannerIterationData iterationData)
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
      private final ValkyriePlannerEdgeData edgeData;
      private final RigidBodyTransform transform = new RigidBodyTransform();
      private final boolean expanded;
      private final double stepYaw;

      public ChildStepProperty(ValkyriePlannerEdgeData edgeData,
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
