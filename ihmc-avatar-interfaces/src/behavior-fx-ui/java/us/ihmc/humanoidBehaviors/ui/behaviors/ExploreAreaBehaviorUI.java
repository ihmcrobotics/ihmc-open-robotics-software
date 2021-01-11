package us.ihmc.humanoidBehaviors.ui.behaviors;

import com.sun.javafx.scene.CameraHelper;
import javafx.application.Platform;
import javafx.event.EventHandler;
import javafx.fxml.FXML;
import javafx.scene.SubScene;
import javafx.scene.control.CheckBox;
import javafx.scene.control.TableView;
import javafx.scene.control.TextField;
import javafx.scene.input.MouseButton;
import javafx.scene.input.MouseEvent;
import javafx.scene.layout.Pane;
import javafx.scene.paint.Color;
import javafx.scene.shape.Sphere;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.humanoidBehaviors.exploreArea.ExploreAreaBehavior;
import us.ihmc.humanoidBehaviors.exploreArea.ExploreAreaBehaviorAPI;
import us.ihmc.humanoidBehaviors.exploreArea.ExploreAreaBehaviorParameters;
import us.ihmc.humanoidBehaviors.exploreArea.TemporaryConvexPolygon2DMessage;
import us.ihmc.humanoidBehaviors.exploreArea.TemporaryPlanarRegionMessage;
import us.ihmc.humanoidBehaviors.ui.BehaviorUIDefinition;
import us.ihmc.humanoidBehaviors.ui.BehaviorUIInterface;
import us.ihmc.humanoidBehaviors.ui.graphics.JavaFXGraphicPrimitives;
import us.ihmc.humanoidBehaviors.ui.graphics.live.JavaFXLivePlanarRegionsGraphic;
import us.ihmc.javaFXVisualizers.JavaFXGraphicTools;
import us.ihmc.javafx.parameter.JavaFXStoredPropertyTable;
import us.ihmc.log.LogTools;
import us.ihmc.messager.Messager;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.ROS2NodeInterface;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Set;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.humanoidBehaviors.exploreArea.ExploreAreaBehavior.ExploreAreaBehaviorState.LookAndStep;
import static us.ihmc.humanoidBehaviors.ui.graphics.JavaFXGraphicPrimitives.*;

public class ExploreAreaBehaviorUI extends BehaviorUIInterface
{
   public static final BehaviorUIDefinition DEFINITION = new BehaviorUIDefinition(ExploreAreaBehavior.DEFINITION, ExploreAreaBehaviorUI::new);

   @FXML private CheckBox exploreAreaCheckBox;
   @FXML private TextField stateTextField;
   @FXML private TableView parameterTable;

   private final JavaFXLivePlanarRegionsGraphic planarRegionsGraphic = new JavaFXLivePlanarRegionsGraphic(false);
   private final GraphicGroup observationPointsGraphicGroup = new GraphicGroup(get3DGroup());
   private final GraphicGroup potentialPointsToExploreGraphicGroup = new GraphicGroup(get3DGroup());
   private final GraphicGroup foundBodyPathToPointsGraphicGroup = new GraphicGroup(get3DGroup());
   private final GraphicGroup planningToPointsGraphicGroup = new GraphicGroup(get3DGroup());
   private final GraphicGroup boundingBoxGraphics = new GraphicGroup(get3DGroup());
   private final LookAndStepVisualizationGroup lookAndStepVisualizationGroup;
   private final GraphicGroup pointToLookAtGroup = new GraphicGroup(get3DGroup());

   private final Sphere pointToLookAt = createSphere3D(new Point3D(100.0, 100.0, 0.0), Color.RED, 0.15);
   private final SubScene sceneNode;
   private final EventHandler<MouseEvent> mouseMoved = this::mouseMoved;
   private final EventHandler<MouseEvent> mouseClicked = this::mouseClicked;
   private final AtomicReference<Point3D> mouseMovedMeshIntersection = new AtomicReference<>();
   private final AtomicReference<Point3D> mouseClickedMeshIntersection = new AtomicReference<>();

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
      pointToLookAtGroup.add(pointToLookAt);

      behaviorMessager.registerTopicListener(ExploreAreaBehaviorAPI.ObservationPosition,
                                             position -> Platform.runLater(() -> displayObservationPosition(position)));
      behaviorMessager.registerTopicListener(ExploreAreaBehaviorAPI.ExplorationBoundingBoxes,
                                             boxes -> Platform.runLater(() -> displayExplorationBoundingBoxes(boxes)));
      behaviorMessager.registerTopicListener(ExploreAreaBehaviorAPI.PotentialPointsToExplore,
                                             points -> Platform.runLater(() -> displayPotentialPointsToExplore(points)));
      behaviorMessager.registerTopicListener(ExploreAreaBehaviorAPI.WalkingToPose,
                                             pose -> Platform.runLater(() -> displayPlanningToPose(pose)));
      behaviorMessager.registerTopicListener(ExploreAreaBehaviorAPI.FoundBodyPath,
                                             bodyPath -> Platform.runLater(() -> displayFoundBodyPathTo(bodyPath)));
      behaviorMessager.registerTopicListener(ExploreAreaBehaviorAPI.ClearPlanarRegions,
                                             unused -> Platform.runLater(this::clearPlanarRegions));
      behaviorMessager.registerTopicListener(ExploreAreaBehaviorAPI.AddPlanarRegionToMap,
                                             region -> Platform.runLater(() -> addPlanarRegionToMap(region)));
      behaviorMessager.registerTopicListener(ExploreAreaBehaviorAPI.AddPolygonToPlanarRegion,
                                             polygon -> Platform.runLater(() -> addPolygonToPlanarRegion(polygon)));
      behaviorMessager.registerTopicListener(ExploreAreaBehaviorAPI.DrawMap,
                                             unused -> Platform.runLater(this::drawMap));
      behaviorMessager.registerTopicListener(ExploreAreaBehaviorAPI.CurrentState,
                                             state -> Platform.runLater(() ->
                                             {
                                                this.currentState = state;
                                                stateTextField.setText(state.name());
                                                lookAndStepVisualizationGroup.setEnabled(state == LookAndStep);
                                             }));
      behaviorMessager.registerTopicListener(ExploreAreaBehaviorAPI.EnvironmentGapToLookAt, point -> Platform.runLater(() -> setPointToLookAt(point)));

      sceneNode.addEventHandler(MouseEvent.MOUSE_MOVED, mouseMoved);
      sceneNode.addEventHandler(MouseEvent.MOUSE_CLICKED, mouseClicked);

      JavaFXStoredPropertyTable javaFXStoredPropertyTable = new JavaFXStoredPropertyTable(parameterTable);
      javaFXStoredPropertyTable.setup(parameters, ExploreAreaBehaviorParameters.keys, this::publishParameters);
      this.sceneNode = sceneNode;
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
      pointToLookAtGroup.setEnabled(enabled);

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
         boundingBoxGraphics.add(createBoundingBox3D(boxes.get(i), color, 0.02));
      }
   }

   private void setPointToLookAt(Point2D pointToLookAt)
   {
      JavaFXGraphicTools.setNodePosition(this.pointToLookAt, new Point3D(pointToLookAt.getX(), pointToLookAt.getY(), 0.4));
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
      foundBodyPathToPointsGraphicGroup.add(JavaFXGraphicPrimitives.createPath(foundBodyPathToPoint, Color.LIMEGREEN));
   }

   public void displayPlanningToPose(Pose3D planningToPosition)
   {
      planningToPointsGraphicGroup.removeAll();
      planningToPointsGraphicGroup.add(createSphereWithArrow3D(planningToPosition, Color.BLUEVIOLET, 0.1));
   }

   public void clearPlanarRegions()
   {
      planarRegionsGraphic.clear();

      transformMap.clear();
      numberOfPolygonsMap.clear();
      planarRegions.clear();
      polygonsMap.clear();
   }

   public void drawMap()
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

   private void mouseMoved(MouseEvent event)
   {
      Point3D intersection = calculateMouseIntersection(event);
      if (intersection != null)
      {
         mouseMovedMeshIntersection.set(intersection);
      }
   }

   private void mouseClicked(MouseEvent event)
   {
      if (event.getButton() == MouseButton.SECONDARY)
      {
         LogTools.info("Mouse right clicked");
         if (mouseMovedMeshIntersection.get() != null)
         {
            System.out.println(mouseMovedMeshIntersection.get());
            getBehaviorMessager().submitMessage(ExploreAreaBehaviorAPI.UserRequestedPointToLookAt, mouseMovedMeshIntersection.get());
         }

         event.consume();
      }
   }

   private Point3D calculateMouseIntersection(MouseEvent event)
   {
      Point3D point1 = new Point3D();
      point1.setX(sceneNode.getCamera().getLocalToSceneTransform().getTx());
      point1.setY(sceneNode.getCamera().getLocalToSceneTransform().getTy());
      point1.setZ(sceneNode.getCamera().getLocalToSceneTransform().getTz());

      Point3D point2 = new Point3D();
      javafx.geometry.Point3D pointOnProjectionPlane = CameraHelper.pickProjectPlane(sceneNode.getCamera(), event.getSceneX(), event.getSceneY());
      point2.setX(pointOnProjectionPlane.getX());
      point2.setY(pointOnProjectionPlane.getY());
      point2.setZ(pointOnProjectionPlane.getZ());

      Line3D line = new Line3D(point1, point2);

      Point3DReadOnly pickPoint = new Point3D(0.0, 0.0, 0.0);
      Vector3DReadOnly planeNormal = new Vector3D(0.0, 0.0, 1.0);
      Point3D pickDirection = EuclidGeometryTools.intersectionBetweenLine3DAndPlane3D(pickPoint, planeNormal, line.getPoint(), line.getDirection());

      return pickDirection;
   }

   @Override
   public void destroy()
   {

   }
}
