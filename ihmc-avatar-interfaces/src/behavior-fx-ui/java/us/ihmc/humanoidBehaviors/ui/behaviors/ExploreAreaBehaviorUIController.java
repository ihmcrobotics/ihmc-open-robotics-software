package us.ihmc.humanoidBehaviors.ui.behaviors;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Set;

import javafx.application.Platform;
import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.fxml.FXML;
import javafx.scene.Group;
import javafx.scene.SubScene;
import javafx.scene.control.CheckBox;
import javafx.scene.control.SpinnerValueFactory;
import javafx.scene.control.SpinnerValueFactory.DoubleSpinnerValueFactory;
import javafx.scene.control.SpinnerValueFactory.IntegerSpinnerValueFactory;
import javafx.scene.control.TableView;
import javafx.scene.control.TextField;
import javafx.scene.paint.Color;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidBehaviors.exploreArea.ExploreAreaBehavior;
import us.ihmc.humanoidBehaviors.exploreArea.ExploreAreaBehaviorParameters;
import us.ihmc.humanoidBehaviors.exploreArea.TemporaryConvexPolygon2DMessage;
import us.ihmc.humanoidBehaviors.exploreArea.TemporaryPlanarRegionMessage;
import us.ihmc.humanoidBehaviors.ui.graphics.BoundingBox3DGraphic;
import us.ihmc.humanoidBehaviors.ui.graphics.PositionGraphic;
import us.ihmc.javafx.parameter.JavaFXParameterTable;
import us.ihmc.javafx.parameter.JavaFXParameterTableEntry;
import us.ihmc.messager.Messager;
import us.ihmc.pathPlanning.visibilityGraphs.ui.graphics.PlanarRegionsGraphic;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.tools.property.DoubleStoredPropertyKey;
import us.ihmc.tools.property.IntegerStoredPropertyKey;
import us.ihmc.tools.property.StoredPropertyKey;

public class ExploreAreaBehaviorUIController extends Group
{
   private final ExploreAreaBehaviorParameters parameters = new ExploreAreaBehaviorParameters();

   @FXML
   private CheckBox exploreAreaCheckBox;
   @FXML
   private TextField stateTextField;
   @FXML
   private TableView parameterTable;

   private final ObservableList<JavaFXParameterTableEntry> parameterTableItems = FXCollections.observableArrayList();

   private Messager behaviorMessager;

   private PlanarRegionsGraphic planarRegionsGraphic;

   private ArrayList<PlanarRegion> planarRegions = new ArrayList<PlanarRegion>();

   private HashMap<Integer, RigidBodyTransform> transformMap = new HashMap<>();
   private HashMap<Integer, Integer> numberOfPolygonsMap = new HashMap<>();
   private HashMap<Integer, ArrayList<ConvexPolygon2D>> polygonsMap = new HashMap<>();

   public void init(SubScene sceneNode, Messager behaviorMessager, DRCRobotModel robotModel)
   {
      this.behaviorMessager = behaviorMessager;
      behaviorMessager.registerTopicListener(ExploreAreaBehavior.ExploreAreaBehaviorAPI.ObservationPosition,
                                             result -> Platform.runLater(() -> displayObservationPosition(result)));
      behaviorMessager.registerTopicListener(ExploreAreaBehavior.ExploreAreaBehaviorAPI.ExplorationBoundingBoxes,
                                             result -> Platform.runLater(() -> displayExplorationBoundingBoxes(result)));
      behaviorMessager.registerTopicListener(ExploreAreaBehavior.ExploreAreaBehaviorAPI.PotentialPointsToExplore,
                                             result -> Platform.runLater(() -> displayPotentialPointsToExplore(result)));
      behaviorMessager.registerTopicListener(ExploreAreaBehavior.ExploreAreaBehaviorAPI.PlanningToPosition,
                                             result -> Platform.runLater(() -> displayPlanningToPosition(result)));
      behaviorMessager.registerTopicListener(ExploreAreaBehavior.ExploreAreaBehaviorAPI.FoundBodyPathTo,
                                             result -> Platform.runLater(() -> displayFoundBodyPathTo(result)));
      behaviorMessager.registerTopicListener(ExploreAreaBehavior.ExploreAreaBehaviorAPI.ClearPlanarRegions,
                                             result -> Platform.runLater(() -> clearPlanarRegions(result)));
      behaviorMessager.registerTopicListener(ExploreAreaBehavior.ExploreAreaBehaviorAPI.AddPlanarRegionToMap,
                                             result -> Platform.runLater(() -> addPlanarRegionToMap(result)));

      behaviorMessager.registerTopicListener(ExploreAreaBehavior.ExploreAreaBehaviorAPI.AddPolygonToPlanarRegion,
                                             result -> Platform.runLater(() -> addPolygonToPlanarRegion(result)));

      behaviorMessager.registerTopicListener(ExploreAreaBehavior.ExploreAreaBehaviorAPI.DrawMap, result -> Platform.runLater(() -> drawMap(result)));
      behaviorMessager.registerTopicListener(ExploreAreaBehavior.ExploreAreaBehaviorAPI.CurrentState,
                                             state -> Platform.runLater(() -> stateTextField.setText(state.name())));

      JavaFXParameterTable javaFXParameterTable = new JavaFXParameterTable(parameterTable);

      for (StoredPropertyKey<?> parameterKey : parameters.keys.keys())
      {
         SpinnerValueFactory spinnerValueFactory = null;

         if (parameterKey.getType().equals(Double.class)) // TODO: Guess have to store these too? Optionally override?
         {
            spinnerValueFactory = new DoubleSpinnerValueFactory(-100.0, 100.0, parameters.get((DoubleStoredPropertyKey) parameterKey), 0.1);
         }
         else if (parameterKey.getType().equals(Integer.class))
         {
            spinnerValueFactory = new IntegerSpinnerValueFactory(-100, 100, parameters.get((IntegerStoredPropertyKey) parameterKey), 1);
         }

         JavaFXParameterTableEntry javaFXParameterTableEntry = new JavaFXParameterTableEntry<>(parameterKey.getTitleCasedName(),
                                                                                               () -> parameters.get(parameterKey),
                                                                                               newValue -> parameters.set(parameterKey, newValue),
                                                                                               observable ->
                                                                                               {
                                                                                               },
                                                                                               spinnerValueFactory);
         javaFXParameterTable.addEntry(javaFXParameterTableEntry);
      }

      javaFXParameterTable.updateEntries();

      //      TableColumn<AnchorPane, ExploreAreaBehaviorParameters> nameColumn = new TableColumn<>("Name");
      //      nameColumn.setCellValueFactory(param -> param.getTableView());

      planarRegionsGraphic = new PlanarRegionsGraphic(false);
   }

   @FXML
   public void exploreArea()
   {
      behaviorMessager.submitMessage(ExploreAreaBehavior.ExploreAreaBehaviorAPI.ExploreArea, exploreAreaCheckBox.isSelected());
   }

   @FXML
   public void saveButton()
   {
      parameters.save();
   }

   private BunchOfPointsDisplayer observationPointsDisplayer = new BunchOfPointsDisplayer(this);
   private BunchOfPointsDisplayer potentialPointsToExploreDisplayer = new BunchOfPointsDisplayer(this);
   private BunchOfPointsDisplayer foundBodyPathToPointsDisplayer = new BunchOfPointsDisplayer(this);
   private BunchOfPointsDisplayer planningToPointsDisplayer = new BunchOfPointsDisplayer(this);
   private BunchOfBoundingBoxesDisplayer boundingBoxesDisplayer = new BunchOfBoundingBoxesDisplayer(this);

   public void displayObservationPosition(Point3D observationPosition)
   {
      observationPointsDisplayer.displayPoint(observationPosition, Color.AZURE, 0.04);
   }

   private void displayExplorationBoundingBoxes(ArrayList<BoundingBox3D> boxes)
   {
      boundingBoxesDisplayer.clear();
      Color[] boundingBoxColors = new Color[] {Color.INDIANRED, Color.DARKSEAGREEN, Color.CADETBLUE};

      for (int i = 0; i < boxes.size(); i++)
      {
         Color color = boundingBoxColors[i % boundingBoxColors.length];
         boundingBoxesDisplayer.displayBoundingBox(boxes.get(i), color, 0.1);
      }
   }

   public void displayPotentialPointsToExplore(ArrayList<Point3D> potentialPointsToExplore)
   {
      potentialPointsToExploreDisplayer.clear();
      foundBodyPathToPointsDisplayer.clear();
      planningToPointsDisplayer.clear();
      potentialPointsToExploreDisplayer.displayPoints(potentialPointsToExplore, Color.BLACK, 0.01);
   }

   public void displayFoundBodyPathTo(Point3D foundBodyPathToPoint)
   {
      foundBodyPathToPointsDisplayer.displayPoint(foundBodyPathToPoint, Color.CORAL, 0.02);
   }

   public void displayPlanningToPosition(Point3D planningToPosition)
   {
      planningToPointsDisplayer.displayPoint(planningToPosition, Color.BLUEVIOLET, 0.03);
   }

   public void clearPlanarRegions(boolean input)
   {
      planarRegions.clear();
      polygonsMap.clear();
   }

   public void drawMap(boolean input)
   {
      getChildren().remove(planarRegionsGraphic);

      planarRegions.clear();

      Set<Integer> indices = transformMap.keySet();

      for (int index : indices)
      {
         RigidBodyTransform rigidBodyTransform = transformMap.get(index);
         Integer numberOfPolygons = numberOfPolygonsMap.get(index);
         ArrayList<ConvexPolygon2D> polygons = polygonsMap.get(index);

         PlanarRegion planarRegion = new PlanarRegion(rigidBodyTransform, polygons);
         planarRegion.setRegionId(index);
         planarRegions.add(planarRegion);
      }

      PlanarRegionsList planarRegionsList = new PlanarRegionsList(planarRegions);
      planarRegionsGraphic.generateMeshes(planarRegionsList);
      planarRegionsGraphic.update();
      getChildren().add(planarRegionsGraphic);
   }

   public void addPlanarRegionToMap(TemporaryPlanarRegionMessage planarRegionMessage)
   {
      transformMap.put(planarRegionMessage.index, planarRegionMessage.transformToWorld);
      numberOfPolygonsMap.put(planarRegionMessage.index, planarRegionMessage.numberOfPolygons);
   }

   public void addPolygonToPlanarRegion(TemporaryConvexPolygon2DMessage polygonMessage)
   {
      ConvexPolygon2D polygon = TemporaryConvexPolygon2DMessage.convertToConvexPolygon2D(polygonMessage);

      int index = polygonMessage.index;

      ArrayList<ConvexPolygon2D> polygons = polygonsMap.get(index);

      if (polygons == null)
      {
         polygons = new ArrayList<ConvexPolygon2D>();
         polygonsMap.put(index, polygons);
      }

      polygons.add(polygon);
   }

   private static class BunchOfPointsDisplayer
   {
      private final Group group;
      private final ArrayList<PositionGraphic> positionGraphics = new ArrayList<PositionGraphic>();

      public BunchOfPointsDisplayer(Group group)
      {
         this.group = group;
      }

      public void clear()
      {
         for (PositionGraphic graphic : positionGraphics)
         {
            group.getChildren().remove(graphic.getNode());
         }

         positionGraphics.clear();
      }

      public void displayPoints(ArrayList<Point3D> points, Color color, double diameter)
      {
         for (Point3D point : points)
         {
            displayPoint(point, color, diameter);
         }
      }

      public void displayPoint(Point3D point, Color color, double diameter)
      {
         PositionGraphic potentialPointToExploreGraphic = new PositionGraphic(color, diameter);
         potentialPointToExploreGraphic.setPosition(point);
         positionGraphics.add(potentialPointToExploreGraphic);
         group.getChildren().add(potentialPointToExploreGraphic.getNode());
      }
   }

   private static class BunchOfBoundingBoxesDisplayer
   {
      private final Group group;
      private final ArrayList<BoundingBox3DGraphic> boundingBoxGraphics = new ArrayList<BoundingBox3DGraphic>();

      public BunchOfBoundingBoxesDisplayer(Group group)
      {
         this.group = group;
      }

      public void clear()
      {
         for (BoundingBox3DGraphic graphic : boundingBoxGraphics)
         {
            group.getChildren().remove(graphic.getNode());
         }

         boundingBoxGraphics.clear();
      }

      public void displayBoundingBox(BoundingBox3D boundingBox, Color color, double lineWidth)
      {
         BoundingBox3DGraphic boundingBox3DGraphic = new BoundingBox3DGraphic(boundingBox, color, lineWidth);
         boundingBoxGraphics.add(boundingBox3DGraphic);
         group.getChildren().add(boundingBox3DGraphic.getNode());
      }
   }

}
