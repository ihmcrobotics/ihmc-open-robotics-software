package us.ihmc.pathPlanning.visibilityGraphs.ui.messager;

import java.util.List;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.Category;
import us.ihmc.messager.MessagerAPIFactory.CategoryTheme;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.messager.MessagerAPIFactory.TopicTheme;
import us.ihmc.messager.MessagerAPIFactory.TypedTopicTheme;
import us.ihmc.pathPlanning.visibilityGraphs.dataStructure.VisibilityMapWithNavigableRegion;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.VisibilityGraphsParametersReadOnly;
import us.ihmc.pathPlanning.visibilityGraphs.interfaces.VisibilityMapHolder;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class UIVisibilityGraphsTopics
{
   private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();

   private static final CategoryTheme VisibilityGraphs = apiFactory.createCategoryTheme("VisibilityGraphs");
   private static final CategoryTheme Map = apiFactory.createCategoryTheme("Map");
   private static final CategoryTheme BodyPath = apiFactory.createCategoryTheme("BodyPath");
   private static final CategoryTheme Cluster = apiFactory.createCategoryTheme("Cluster");
   private static final CategoryTheme RawPoints = apiFactory.createCategoryTheme("RawPoints");
   private static final CategoryTheme InterRegion = apiFactory.createCategoryTheme("InterRegion");
   private static final CategoryTheme NavigableRegion = apiFactory.createCategoryTheme("NavigableRegion");
   private static final CategoryTheme NavigableExtrusions = apiFactory.createCategoryTheme("NavigableExtrusions");
   private static final CategoryTheme NonNavigableExtrusions = apiFactory.createCategoryTheme("NonNavigableExtrusions");
   private static final CategoryTheme PlanarRegion = apiFactory.createCategoryTheme("PlanarRegion");
   private static final CategoryTheme Start = apiFactory.createCategoryTheme("Start");
   private static final CategoryTheme Goal = apiFactory.createCategoryTheme("Goal");
   private static final CategoryTheme EditMode = apiFactory.createCategoryTheme("EditMode");
   private static final CategoryTheme UnitTest = apiFactory.createCategoryTheme("UnitTest");
   private static final CategoryTheme Dataset = apiFactory.createCategoryTheme("Dataset");
   private static final CategoryTheme Previous = apiFactory.createCategoryTheme("Next");
   private static final CategoryTheme Next = apiFactory.createCategoryTheme("Previous");
   private static final CategoryTheme Reload = apiFactory.createCategoryTheme("Reload");
   private static final CategoryTheme All = apiFactory.createCategoryTheme("All");
   private static final CategoryTheme Walker = apiFactory.createCategoryTheme("Walker");
   private static final CategoryTheme Offset = apiFactory.createCategoryTheme("Offset");
   private static final CategoryTheme Size = apiFactory.createCategoryTheme("Size");
   private static final CategoryTheme BoxSize = apiFactory.createCategoryTheme("Box");
   private static final CategoryTheme Collision = apiFactory.createCategoryTheme("Collision");
   private static final CategoryTheme Animation = apiFactory.createCategoryTheme("Animation");
   private static final CategoryTheme Shadow = apiFactory.createCategoryTheme("Shadow");
   private static final CategoryTheme RandomizeID = apiFactory.createCategoryTheme("RandomizeID");

   private static final TopicTheme Parameters = apiFactory.createTopicTheme("Parameters");

   private static final TypedTopicTheme<Boolean> Enable = apiFactory.createTypedTopicTheme("Enable");
   private static final TypedTopicTheme<Boolean> Show = apiFactory.createTypedTopicTheme("Show");
   private static final TypedTopicTheme<Boolean> Reset = apiFactory.createTypedTopicTheme("Reset");
   private static final TypedTopicTheme<Boolean> Request = apiFactory.createTypedTopicTheme("Request");
   private static final TypedTopicTheme<Boolean> Stop = apiFactory.createTypedTopicTheme("Stop");
   private static final TypedTopicTheme<Boolean> ComputePath = apiFactory.createTypedTopicTheme("ComputePath");
   private static final TypedTopicTheme<Boolean> ComputePathWithOcclusions = apiFactory.createTypedTopicTheme("ComputePathWithOcclusions");
   private static final TypedTopicTheme<Point3D> Position = apiFactory.createTypedTopicTheme("Position");
   private static final TypedTopicTheme<Quaternion> Orientation = apiFactory.createTypedTopicTheme("Orientation");
   private static final TypedTopicTheme<Boolean> Export = apiFactory.createTypedTopicTheme("Export");
   private static final TypedTopicTheme<String> Path = apiFactory.createTypedTopicTheme("Path");
   private static final TypedTopicTheme<List<String>> Paths = apiFactory.createTypedTopicTheme("Paths");

   private static final TopicTheme Data = apiFactory.createTopicTheme("Data");

   private static final Category Root = apiFactory.createRootCategory(apiFactory.createCategoryTheme("VizGraphs"));
   
   public static final Topic<Boolean> GlobalReset = Root.topic(Reset);

   public static final Topic<PlanarRegionsList> PlanarRegionData = Root.child(PlanarRegion).topic(Data);
   public static final Topic<Boolean> RandomizePlanarRegionIDRequest = Root.child(PlanarRegion).child(RandomizeID).topic(Request);

   public static final Topic<Boolean> StartEditModeEnabled = Root.child(Start).child(EditMode).topic(Enable);
   public static final Topic<Boolean> GoalEditModeEnabled = Root.child(Goal).child(EditMode).topic(Enable);
   public static final Topic<Boolean> ShowStartPosition = Root.child(Start).topic(Show);
   public static final Topic<Boolean> ShowGoalPosition = Root.child(Goal).topic(Show);
   public static final Topic<Point3D> StartPosition = Root.child(Start).topic(Position);
   public static final Topic<Point3D> GoalPosition = Root.child(Goal).topic(Position);

   public static final Topic<List<Point3DReadOnly>> BodyPathData = Root.child(VisibilityGraphs).child(BodyPath).topic(Data);
   
   public static final Topic<Boolean> ShowNavigableRegionVisibilityMaps = Root.child(VisibilityGraphs).child(NavigableRegion).child(Map).topic(Show);
   public static final Topic<Boolean> ShowInterRegionVisibilityMap = Root.child(VisibilityGraphs).child(InterRegion).child(Map).topic(Show);
   public static final Topic<Boolean> ShowStartVisibilityMap = Root.child(VisibilityGraphs).child(Start).child(Map).topic(Show);
   public static final Topic<Boolean> ShowGoalVisibilityMap = Root.child(VisibilityGraphs).child(Goal).child(Map).topic(Show);

   public static final Topic<List<VisibilityMapWithNavigableRegion>> NavigableRegionData = Root.child(VisibilityGraphs).child(NavigableRegion).topic(Data);
   public static final Topic<List<VisibilityMapWithNavigableRegion>> NavigableRegionVisibilityMap = Root.child(VisibilityGraphs).child(NavigableRegion).child(Map).topic(Data);
   public static final Topic<VisibilityMapHolder> InterRegionVisibilityMap = Root.child(VisibilityGraphs).child(InterRegion).child(Map).topic(Data);
   public static final Topic<VisibilityMapHolder> StartVisibilityMap = Root.child(VisibilityGraphs).child(Start).child(Map).topic(Data);
   public static final Topic<VisibilityMapHolder> GoalVisibilityMap = Root.child(VisibilityGraphs).child(Goal).child(Map).topic(Data);

   public static final Topic<Boolean> VisibilityGraphsComputePath = Root.child(VisibilityGraphs).topic(ComputePath);
   public static final Topic<Boolean> VisibilityGraphsComputePathWithOcclusions = Root.child(VisibilityGraphs).topic(ComputePathWithOcclusions);
   public static final Topic<VisibilityGraphsParametersReadOnly> VisibilityGraphsParameters = Root.child(VisibilityGraphs).topic(Parameters);
   public static final Topic<Boolean> ShowBodyPath = Root.child(VisibilityGraphs).child(BodyPath).topic(Show);
   public static final Topic<Boolean> ShowPlanarRegions = Root.child(PlanarRegion).topic(Show);
   public static final Topic<Boolean> ShowClusterRawPoints = Root.child(VisibilityGraphs).child(Cluster).child(RawPoints).topic(Show);
   public static final Topic<Boolean> ShowClusterNavigableExtrusions = Root.child(VisibilityGraphs).child(Cluster).child(NavigableExtrusions).topic(Show);
   public static final Topic<Boolean> ShowClusterNonNavigableExtrusions = Root.child(VisibilityGraphs).child(Cluster).child(NonNavigableExtrusions).topic(Show);
   public static final Topic<Boolean> exportUnitTestDataFile = Root.child(UnitTest).topic(Export);
   public static final Topic<String> exportUnitTestPath = Root.child(UnitTest).topic(Path);

   // Topics for the test visualizer
   // TODO Need to figure out a better way to declare these guys, in part to be able to separate the main UI topics from the test topics.
   public static final Topic<Boolean> ReloadDatasetRequest = Root.child(UnitTest).child(Dataset).child(Reload).topic(Request);
   public static final Topic<Boolean> PreviousDatasetRequest = Root.child(UnitTest).child(Dataset).child(Previous).topic(Request);
   public static final Topic<String> CurrentDatasetPath = Root.child(UnitTest).child(Dataset).topic(Path);
   public static final Topic<Boolean> NextDatasetRequest = Root.child(UnitTest).child(Dataset).child(Next).topic(Request);
   public static final Topic<List<String>> AllDatasetPaths = Root.child(UnitTest).child(Dataset).child(All).topic(Paths);
   public static final Topic<Boolean> EnableWalkerAnimation = Root.child(UnitTest).child(Walker).child(Animation).topic(Enable);
   public static final Topic<Point3D> WalkerPosition = Root.child(UnitTest).child(Walker).topic(Position);
   public static final Topic<Quaternion> WalkerOrientation = Root.child(UnitTest).child(Walker).topic(Orientation);
   public static final Topic<Double> WalkerOffsetHeight = Root.child(UnitTest).child(Walker).child(Offset).topic(Data);
   public static final Topic<Vector3D> WalkerSize = Root.child(UnitTest).child(Walker).child(Size).topic(Data);
   public static final Topic<Vector3D> WalkerBoxSize = Root.child(UnitTest).child(Walker).child(BoxSize).child(Size).topic(Data);
   public static final Topic<List<Point3D>> WalkerCollisionLocations = Root.child(UnitTest).child(Walker).child(Collision).topic(Data);
   public static final Topic<Boolean> StopWalker = Root.child(UnitTest).child(Walker).topic(Stop);
   public static final Topic<PlanarRegionsList> ShadowPlanarRegionData = Root.child(PlanarRegion).child(Shadow).topic(Data);

   public static final MessagerAPI API = apiFactory.getAPIAndCloseFactory();
}
