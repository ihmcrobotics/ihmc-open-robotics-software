package us.ihmc.pathPlanning.visibilityGraphs.ui.messager;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.Category;
import us.ihmc.messager.MessagerAPIFactory.CategoryTheme;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityMapWithNavigableRegion;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityMapHolder;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersReadOnly;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.List;

public class UIVisibilityGraphsTopics
{
   private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
   private static final Category Root = apiFactory.createRootCategory("VizGraphs");
   private static final CategoryTheme VisibilityGraphs = apiFactory.createCategoryTheme("VisibilityGraphs");

   public static final Topic<Boolean> GlobalReset = topic("GlobalReset");

   public static final Topic<PlanarRegionsList> PlanarRegionData = topic("PlanarRegionData");
   public static final Topic<Boolean> RandomizePlanarRegionIDRequest = topic("RandomizePlanarRegionIDRequest");

   public static final Topic<Boolean> StartEditModeEnabled = topic("StartEditModeEnabled");
   public static final Topic<Boolean> GoalEditModeEnabled = topic("GoalEditModeEnabled");
   public static final Topic<Boolean> ShowStartPosition = topic("ShowStartPosition");
   public static final Topic<Boolean> ShowGoalPosition = topic("ShowGoalPosition");
   public static final Topic<Point3D> StartPosition = topic("StartPosition");
   public static final Topic<Point3D> GoalPosition = topic("GoalPosition");

   public static final Topic<List<Point3DReadOnly>> BodyPathData = topic("BodyPathData");
   
   public static final Topic<Boolean> ShowNavigableRegionVisibilityMaps = topic("ShowNavigableRegionVisibilityMaps");
   public static final Topic<Boolean> ShowInterRegionVisibilityMap = topic("ShowInterRegionVisibilityMap");
   public static final Topic<Boolean> ShowStartVisibilityMap = topic("ShowStartVisibilityMap");
   public static final Topic<Boolean> ShowGoalVisibilityMap = topic("ShowGoalVisibilityMap");

   public static final Topic<List<VisibilityMapWithNavigableRegion>> NavigableRegionData = topic("NavigableRegionData");
   public static final Topic<List<VisibilityMapWithNavigableRegion>> NavigableRegionVisibilityMap = topic("NavigableRegionVisibilityMap");
   public static final Topic<VisibilityMapHolder> InterRegionVisibilityMap = topic("InterRegionVisibilityMap");
   public static final Topic<VisibilityMapHolder> StartVisibilityMap = topic("StartVisibilityMap");
   public static final Topic<VisibilityMapHolder> GoalVisibilityMap = topic("GoalVisibilityMap");

   public static final Topic<Boolean> VisibilityGraphsComputePath = topic("VisibilityGraphsComputePath");
   public static final Topic<Boolean> VisibilityGraphsComputePathWithOcclusions = topic("VisibilityGraphsComputePathWithOcclusions");
   public static final Topic<VisibilityGraphsParametersReadOnly> VisibilityGraphsParameters = topic("VisibilityGraphsParameters");
   public static final Topic<Boolean> ShowBodyPath = topic("ShowBodyPath");
   public static final Topic<Boolean> ShowPlanarRegions = topic("ShowPlanarRegions");
   public static final Topic<Boolean> ShowClusterRawPoints = topic("ShowClusterRawPoints");
   public static final Topic<Boolean> ShowClusterPreferredNavigableExtrusions = topic("ShowClusterPreferredNavigableExtrusions");
   public static final Topic<Boolean> ShowClusterPreferredNonNavigableExtrusions = topic("ShowClusterPreferredNonNavigableExtrusions");
   public static final Topic<Boolean> ShowClusterNavigableExtrusions = topic("ShowClusterNavigableExtrusions");
   public static final Topic<Boolean> ShowClusterNonNavigableExtrusions = topic("ShowClusterNonNavigableExtrusions");
   public static final Topic<Boolean> exportUnitTestDataFile = topic("exportUnitTestDataFile");
   public static final Topic<String> exportUnitTestPath = topic("exportUnitTestPath");

   // Topics for the test visualizer
   // TODO Need to figure out a better way to declare these guys, in part to be able to separate the main UI topics from the test topics.
   public static final Topic<Boolean> ReloadDatasetRequest = topic("ReloadDatasetRequest");
   public static final Topic<Boolean> PreviousDatasetRequest = topic("PreviousDatasetRequest");
   public static final Topic<String> CurrentDatasetPath = topic("CurrentDatasetPath");
   public static final Topic<Boolean> NextDatasetRequest = topic("NextDatasetRequest");
   public static final Topic<List<String>> AllDatasetPaths = topic("AllDatasetPaths");
   public static final Topic<Boolean> EnableWalkerAnimation = topic("EnableWalkerAnimation");
   public static final Topic<Point3D> WalkerPosition = topic("WalkerPosition");
   public static final Topic<Quaternion> WalkerOrientation = topic("WalkerOrientation");
   public static final Topic<Double> WalkerOffsetHeight = topic("WalkerOffsetHeight");
   public static final Topic<Vector3D> WalkerSize = topic("WalkerSize");
   public static final Topic<Vector3D> WalkerBoxSize = topic("WalkerBoxSize");
   public static final Topic<List<Point3D>> WalkerCollisionLocations = topic("WalkerCollisionLocations");
   public static final Topic<Boolean> StopWalker = topic("StopWalker");
   public static final Topic<PlanarRegionsList> ShadowPlanarRegionData = topic("ShadowPlanarRegionData");

   public static final MessagerAPI API = apiFactory.getAPIAndCloseFactory();

   private static final <T> Topic<T> topic(String name)
   {
      return Root.child(VisibilityGraphs).topic(apiFactory.createTypedTopicTheme(name));
   }
}
