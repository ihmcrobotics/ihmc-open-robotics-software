package us.ihmc.humanoidBehaviors.ui.behaviors;

import javafx.application.Platform;
import javafx.fxml.FXML;
import javafx.scene.SubScene;
import javafx.scene.control.CheckBox;
import javafx.scene.control.TableView;
import javafx.scene.control.TextField;
import javafx.scene.layout.Pane;
import javafx.scene.paint.Color;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidBehaviors.exploreArea.ExploreAreaBehavior;
import us.ihmc.humanoidBehaviors.exploreArea.ExploreAreaBehaviorAPI;
import us.ihmc.humanoidBehaviors.exploreArea.ExploreAreaBehaviorParameters;
import us.ihmc.humanoidBehaviors.exploreArea.TemporaryConvexPolygon2DMessage;
import us.ihmc.humanoidBehaviors.exploreArea.TemporaryPlanarRegionMessage;
import us.ihmc.humanoidBehaviors.ui.BehaviorUIDefinition;
import us.ihmc.humanoidBehaviors.ui.BehaviorUIInterface;
import us.ihmc.humanoidBehaviors.ui.graphics.BodyPathPlanGraphic;
import us.ihmc.humanoidBehaviors.ui.graphics.live.LivePlanarRegionsGraphic;
import us.ihmc.javafx.parameter.JavaFXStoredPropertyTable;
import us.ihmc.messager.Messager;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.ROS2NodeInterface;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Set;

import static us.ihmc.humanoidBehaviors.exploreArea.ExploreAreaBehavior.ExploreAreaBehaviorState.LookAndStep;
import static us.ihmc.humanoidBehaviors.ui.graphics.JavaFXGraphicPrimitives.createBoundingBox3D;
import static us.ihmc.humanoidBehaviors.ui.graphics.JavaFXGraphicPrimitives.createSphere3D;

public class ExploreAreaBehaviorUI extends BehaviorUIInterface
{
   public static final BehaviorUIDefinition DEFINITION = new BehaviorUIDefinition(ExploreAreaBehavior.DEFINITION, ExploreAreaBehaviorUI::new);

   @FXML private CheckBox exploreAreaCheckBox;
   @FXML private TextField stateTextField;
   @FXML private TableView parameterTable;

   private final LivePlanarRegionsGraphic planarRegionsGraphic = new LivePlanarRegionsGraphic(false);
   private final GraphicGroup observationPointsGraphicGroup = new GraphicGroup(get3DGroup());
   private final GraphicGroup potentialPointsToExploreGraphicGroup = new GraphicGroup(get3DGroup());
   private final GraphicGroup foundBodyPathToPointsGraphicGroup = new GraphicGroup(get3DGroup());
   private final GraphicGroup planningToPointsGraphicGroup = new GraphicGroup(get3DGroup());
   private final GraphicGroup boundingBoxGraphics = new GraphicGroup(get3DGroup());
   private final LookAndStepVisualizationGroup lookAndStepVisualizationGroup;

   private final ExploreAreaBehaviorParameters parameters = new ExploreAreaBehaviorParameters();
   private final ArrayList<PlanarRegion> planarRegions = new ArrayList<>();
   private final HashMap<Integer, RigidBodyTransform> transformMap = new HashMap<>();
   private final HashMap<Integer, Integer> numberOfPolygonsMap = new HashMap<>();
   private final HashMap<Integer, ArrayList<ConvexPolygon2D>> polygonsMap = new HashMap<>();

   private ExploreAreaBehavior.ExploreAreaBehaviorState currentState;

   public ExploreAreaBehaviorUI(SubScene sceneNode, Pane visualizationPane, ROS2NodeInterface ros2Node, Messager behaviorMessager, DRCRobotModel robotModel)
   {
      super(sceneNode, visualizationPane, ros2Node, behaviorMessager, robotModel);

      lookAndStepVisualizationGroup = new LookAndStepVisualizationGroup(ros2Node, behaviorMessager);
      get3DGroup().getChildren().add(lookAndStepVisualizationGroup);

      behaviorMessager.registerTopicListener(ExploreAreaBehaviorAPI.ObservationPosition,
                                             result -> Platform.runLater(() -> displayObservationPosition(result)));
      behaviorMessager.registerTopicListener(ExploreAreaBehaviorAPI.ExplorationBoundingBoxes,
                                             result -> Platform.runLater(() -> displayExplorationBoundingBoxes(result)));
      behaviorMessager.registerTopicListener(ExploreAreaBehaviorAPI.PotentialPointsToExplore,
                                             result -> Platform.runLater(() -> displayPotentialPointsToExplore(result)));
      behaviorMessager.registerTopicListener(ExploreAreaBehaviorAPI.PlanningToPosition,
                                             result -> Platform.runLater(() -> displayPlanningToPosition(result)));
      behaviorMessager.registerTopicListener(ExploreAreaBehaviorAPI.FoundBodyPath,
                                             result -> Platform.runLater(() -> displayFoundBodyPathTo(result)));
      behaviorMessager.registerTopicListener(ExploreAreaBehaviorAPI.ClearPlanarRegions,
                                             result -> Platform.runLater(() -> clearPlanarRegions(result)));
      behaviorMessager.registerTopicListener(ExploreAreaBehaviorAPI.AddPlanarRegionToMap,
                                             result -> Platform.runLater(() -> addPlanarRegionToMap(result)));
      behaviorMessager.registerTopicListener(ExploreAreaBehaviorAPI.AddPolygonToPlanarRegion,
                                             result -> Platform.runLater(() -> addPolygonToPlanarRegion(result)));
      behaviorMessager.registerTopicListener(ExploreAreaBehaviorAPI.DrawMap,
                                             result -> Platform.runLater(() -> drawMap(result)));
      behaviorMessager.registerTopicListener(ExploreAreaBehaviorAPI.CurrentState,
                                             state -> Platform.runLater(() ->
                                             {
                                                this.currentState = state;
                                                stateTextField.setText(state.name());
                                                lookAndStepVisualizationGroup.setEnabled(state == LookAndStep);
                                             }));

      JavaFXStoredPropertyTable javaFXStoredPropertyTable = new JavaFXStoredPropertyTable(parameterTable);
      javaFXStoredPropertyTable.setup(parameters, ExploreAreaBehaviorParameters.keys, this::publishParameters);
   }

   @Override
   public void setEnabled(boolean enabled)
   {
      planarRegionsGraphic.setEnabled(enabled);

      observationPointsGraphicGroup.setEnabled(enabled);
      potentialPointsToExploreGraphicGroup.setEnabled(enabled);
      foundBodyPathToPointsGraphicGroup.setEnabled(enabled);
      planningToPointsGraphicGroup.setEnabled(enabled);
      boundingBoxGraphics.setEnabled(enabled);
      lookAndStepVisualizationGroup.setEnabled(enabled && currentState == LookAndStep);

      if (enabled)
      {
         Platform.runLater(() -> get3DGroup().getChildren().add(planarRegionsGraphic));
      }
      else
      {
         Platform.runLater(() -> get3DGroup().getChildren().remove(planarRegionsGraphic));
      }
   }

   private void publishParameters()
   {
      getBehaviorMessager().submitMessage(ExploreAreaBehaviorAPI.Parameters, parameters.getAllAsStrings());
   }

   @FXML
   public void exploreArea()
   {
      getBehaviorMessager().submitMessage(ExploreAreaBehaviorAPI.ExploreArea, exploreAreaCheckBox.isSelected());
   }

   @FXML
   public void randomPoseUpdate()
   {
      getBehaviorMessager().submitMessage(ExploreAreaBehaviorAPI.RandomPoseUpdate, true);
   }

   @FXML
   public void doSlamButtonClicked()
   {
      getBehaviorMessager().submitMessage(ExploreAreaBehaviorAPI.DoSlam, true);
   }

   @FXML
   public void clearMapButtonClicked()
   {
      getBehaviorMessager().submitMessage(ExploreAreaBehaviorAPI.ClearMap, true);
   }

   @FXML
   public void saveButton()
   {
      parameters.save();
   }

   public void displayObservationPosition(Point3D observationPosition)
   {
      observationPointsGraphicGroup.add(createSphere3D(observationPosition, Color.AZURE, 0.04));
   }

   private void displayExplorationBoundingBoxes(ArrayList<BoundingBox3D> boxes)
   {
      boundingBoxGraphics.removeAll();
      Color[] boundingBoxColors = new Color[] {Color.INDIANRED, Color.DARKSEAGREEN, Color.CADETBLUE};

      for (int i = 0; i < boxes.size(); i++)
      {
         Color color = boundingBoxColors[i % boundingBoxColors.length];
         boundingBoxGraphics.add(createBoundingBox3D(boxes.get(i), color, 0.1));
      }
   }

   public void displayPotentialPointsToExplore(ArrayList<Point3D> potentialPointsToExplore)
   {
      potentialPointsToExploreGraphicGroup.removeAll();
      planningToPointsGraphicGroup.removeAll();
      for (Point3D potentialPointToExplore : potentialPointsToExplore)
      {
         potentialPointsToExploreGraphicGroup.add(createSphere3D(potentialPointToExplore, Color.BLACK, 0.01));
      }
   }

   public void displayFoundBodyPathTo(List<Pose3D> foundBodyPathToPoint)
   {
      BodyPathPlanGraphic bodyPathPlanGraphic = new BodyPathPlanGraphic();
      bodyPathPlanGraphic.generateMeshesAsynchronously(foundBodyPathToPoint);
      foundBodyPathToPointsGraphicGroup.add(bodyPathPlanGraphic);
   }

   public void displayPlanningToPosition(Point3D planningToPosition)
   {
      planningToPointsGraphicGroup.removeAll();
      planningToPointsGraphicGroup.add(createSphere3D(planningToPosition, Color.BLUEVIOLET, 0.1));
   }

   public void clearPlanarRegions(boolean input)
   {
      planarRegionsGraphic.clear();

      transformMap.clear();
      numberOfPolygonsMap.clear();
      planarRegions.clear();
      polygonsMap.clear();
   }

   public void drawMap(boolean input)
   {
      planarRegionsGraphic.clear();
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
      planarRegionsGraphic.acceptPlanarRegions(planarRegionsList);
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
         polygons = new ArrayList<>();
         polygonsMap.put(index, polygons);
      }

      polygons.add(polygon);
   }

   @Override
   public void destroy()
   {

   }
}
