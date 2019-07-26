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
import javafx.scene.control.*;
import javafx.scene.control.SpinnerValueFactory.DoubleSpinnerValueFactory;
import javafx.scene.control.SpinnerValueFactory.IntegerSpinnerValueFactory;
import javafx.scene.paint.Color;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidBehaviors.exploreArea.ExploreAreaBehavior;
import us.ihmc.humanoidBehaviors.exploreArea.ExploreAreaBehaviorParameters;
import us.ihmc.humanoidBehaviors.exploreArea.TemporaryConvexPolygon2DMessage;
import us.ihmc.humanoidBehaviors.exploreArea.TemporaryPlanarRegionMessage;
import us.ihmc.javafx.parameter.JavaFXParameterTable;
import us.ihmc.javafx.parameter.JavaFXParameterTableEntry;
import us.ihmc.pathPlanning.visibilityGraphs.ui.graphics.PlanarRegionsGraphic;
import us.ihmc.humanoidBehaviors.ui.graphics.PositionGraphic;
import us.ihmc.messager.Messager;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.tools.property.DoubleStoredPropertyKey;
import us.ihmc.tools.property.IntegerStoredPropertyKey;
import us.ihmc.tools.property.StoredPropertyKey;

public class ExploreAreaBehaviorUIController extends Group
{
   private final ExploreAreaBehaviorParameters parameters = new ExploreAreaBehaviorParameters();

   @FXML private CheckBox exploreAreaCheckBox;
   @FXML private TextField stateTextField;
   @FXML private TableView parameterTable;

   private final ObservableList<JavaFXParameterTableEntry> parameterTableItems = FXCollections.observableArrayList();

   //   @FXML
   //   private Button singleSupportButton;

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
      behaviorMessager.registerTopicListener(ExploreAreaBehavior.ExploreAreaBehaviorAPI.ClearPlanarRegions,
                                             result -> Platform.runLater(() -> clearPlanarRegions(result)));
      behaviorMessager.registerTopicListener(ExploreAreaBehavior.ExploreAreaBehaviorAPI.AddPlanarRegionToMap,
                                             result -> Platform.runLater(() -> addPlanarRegionToMap(result)));

      behaviorMessager.registerTopicListener(ExploreAreaBehavior.ExploreAreaBehaviorAPI.AddPolygonToPlanarRegion,
                                             result -> Platform.runLater(() -> addPolygonToPlanarRegion(result)));

      behaviorMessager.registerTopicListener(ExploreAreaBehavior.ExploreAreaBehaviorAPI.DrawMap, result -> Platform.runLater(() -> drawMap(result)));
      behaviorMessager.registerTopicListener(ExploreAreaBehavior.ExploreAreaBehaviorAPI.CurrentState, state -> Platform.runLater(() -> stateTextField.setText(state.name())));

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
                                                                                               observable -> { },
                                                                                               spinnerValueFactory);
         javaFXParameterTable.addEntry(javaFXParameterTableEntry);
      }

      javaFXParameterTable.updateEntries();


//      TableColumn<AnchorPane, ExploreAreaBehaviorParameters> nameColumn = new TableColumn<>("Name");
//      nameColumn.setCellValueFactory(param -> param.getTableView());


      planarRegionsGraphic = new PlanarRegionsGraphic(false);
   }

//   class

   @FXML public void exploreArea()
   {
      behaviorMessager.submitMessage(ExploreAreaBehavior.ExploreAreaBehaviorAPI.ExploreArea, exploreAreaCheckBox.isSelected());
   }

   public void displayObservationPosition(Point3D observationPosition)
   {
      PositionGraphic observationPositionGraphic = new PositionGraphic(Color.AZURE, 0.04);
      observationPositionGraphic.setPosition(observationPosition);
      getChildren().add(observationPositionGraphic.getNode());
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
}
