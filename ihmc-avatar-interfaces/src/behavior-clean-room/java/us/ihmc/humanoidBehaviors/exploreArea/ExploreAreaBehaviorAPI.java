package us.ihmc.humanoidBehaviors.exploreArea;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.messager.MessagerAPIFactory;

import java.util.ArrayList;
import java.util.List;

public class ExploreAreaBehaviorAPI
{
   private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
   private static final MessagerAPIFactory.Category RootCategory = apiFactory.createRootCategory("ExploreAreaBehavior");
   private static final MessagerAPIFactory.CategoryTheme ExploreAreaTheme = apiFactory.createCategoryTheme("ExploreArea");

   public static final MessagerAPIFactory.Topic<Boolean> ExploreArea = topic("ExploreArea");
   public static final MessagerAPIFactory.Topic<Boolean> RandomPoseUpdate = topic("RandomPoseUpdate");
   public static final MessagerAPIFactory.Topic<Boolean> DoSlam = topic("DoSlam");
   public static final MessagerAPIFactory.Topic<Boolean> ClearMap = topic("ClearMap");
   public static final MessagerAPIFactory.Topic<PlanarRegionsListMessage> ConcatenatedMap = topic("ConcatenatedMap");
   public static final MessagerAPIFactory.Topic<Point3D> ObservationPosition = topic("ObservationPosition");
   public static final MessagerAPIFactory.Topic<ArrayList<BoundingBox3D>> ExplorationBoundingBoxes = topic("ExplorationBoundingBoxes");
   public static final MessagerAPIFactory.Topic<ArrayList<Point3D>> PotentialPointsToExplore = topic("PotentialPointsToExplore");
   public static final MessagerAPIFactory.Topic<List<Pose3D>> FoundBodyPath = topic("FoundBodyPath");
   public static final MessagerAPIFactory.Topic<Pose3D> WalkingToPose = topic("PlanningToPosition");
   public static final MessagerAPIFactory.Topic<Object> DrawMap = topic("DrawMap");
   public static final MessagerAPIFactory.Topic<Object> ClearPlanarRegions = topic("ClearPlanarRegions");
   public static final MessagerAPIFactory.Topic<TemporaryPlanarRegionMessage> AddPlanarRegionToMap = topic("AddPlanarRegionToMap");
   public static final MessagerAPIFactory.Topic<TemporaryConvexPolygon2DMessage> AddPolygonToPlanarRegion = topic("AddPolygonToPlanarRegion");
   public static final MessagerAPIFactory.Topic<List<String>> Parameters = topic("Parameters");
   public static final MessagerAPIFactory.Topic<ExploreAreaBehavior.ExploreAreaBehaviorState> CurrentState = topic("CurrentState");
   public static final MessagerAPIFactory.Topic<Point2D> EnvironmentGapToLookAt = topic("EnvironmentGapToLookAt");
   public static final MessagerAPIFactory.Topic<Point3D> UserRequestedPointToLookAt = topic("UserRequestedPointToLookAt");

   private static final <T> MessagerAPIFactory.Topic<T> topic(String name)
   {
      return RootCategory.child(ExploreAreaTheme).topic(apiFactory.createTypedTopicTheme(name));
   }

   public static final MessagerAPIFactory.MessagerAPI create()
   {
      return apiFactory.getAPIAndCloseFactory();
   }
}
