package us.ihmc.humanoidBehaviors.ui.behaviors;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Set;

import javafx.application.Platform;
import javafx.fxml.FXML;
import javafx.scene.Group;
import javafx.scene.SubScene;
import javafx.scene.control.CheckBox;
import javafx.scene.paint.Color;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidBehaviors.exploreArea.ExploreAreaBehavior;
import us.ihmc.humanoidBehaviors.exploreArea.TemporaryConvexPolygon2DMessage;
import us.ihmc.humanoidBehaviors.exploreArea.TemporaryPlanarRegionMessage;
import us.ihmc.humanoidBehaviors.ui.graphics.PlanarRegionsGraphic;
import us.ihmc.humanoidBehaviors.ui.graphics.PositionGraphic;
import us.ihmc.messager.Messager;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class ExploreAreaBehaviorUIController extends Group
{
   @FXML
   private CheckBox exploreAreaCheckBox;

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

      planarRegionsGraphic = new PlanarRegionsGraphic();
   }

   @FXML
   public void exploreArea()
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
