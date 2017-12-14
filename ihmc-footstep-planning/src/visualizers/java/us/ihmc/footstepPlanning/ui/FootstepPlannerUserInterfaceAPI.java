package us.ihmc.footstepPlanning.ui;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.robotEnvironmentAwareness.communication.APIFactory;
import us.ihmc.robotEnvironmentAwareness.communication.APIFactory.*;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class FootstepPlannerUserInterfaceAPI
{
   private static final APIFactory apiFactory = new APIFactory();

   private static final CategoryTheme PlanarRegion = apiFactory.createCategoryTheme("PlanarRegion");
   private static final CategoryTheme Start = apiFactory.createCategoryTheme("Start");
   private static final CategoryTheme Goal = apiFactory.createCategoryTheme("Goal");
   private static final CategoryTheme PositionTheme = apiFactory.createCategoryTheme("PositionTheme");
   private static final CategoryTheme OrientationTheme = apiFactory.createCategoryTheme("OrientationTheme");
   private static final CategoryTheme EditMode = apiFactory.createCategoryTheme("EditMode");
   private static final CategoryTheme FootstepPlan = apiFactory.createCategoryTheme("FootstepPlan");

   private static final TopicTheme Parameters = apiFactory.createTopicTheme("Parameters");

   private static final TypedTopicTheme<Boolean> Show = apiFactory.createTypedTopicTheme("Show");
   private static final TypedTopicTheme<Boolean> Enable = apiFactory.createTypedTopicTheme("Enable");
   private static final TypedTopicTheme<Boolean> Reset = apiFactory.createTypedTopicTheme("Reset");
   private static final TypedTopicTheme<Point3D> Position = apiFactory.createTypedTopicTheme("Position");
   private static final TypedTopicTheme<Double> Orientation = apiFactory.createTypedTopicTheme("Orientation");
   private static final TypedTopicTheme<Boolean> ComputePath = apiFactory.createTypedTopicTheme("ComputePath");
   private static final TypedTopicTheme<FootstepPlannerType> FootstepPlannerType = apiFactory.createTypedTopicTheme("FootstepPlannerType");
   private static final TypedTopicTheme<FootstepPlannerParameters> FootstepPlannerParameters = apiFactory.createTypedTopicTheme("FootstepPlannerParameters");

   private static final TopicTheme Data = apiFactory.createTopicTheme("Data");

   private static final Category Root = apiFactory.getRootCategory(apiFactory.createCategoryTheme("FootstepPlanning"));

   public static final Topic<PlanarRegionsList> PlanarRegionDataTopic = Root.child(PlanarRegion).topic(Data);
   public static final Topic<Boolean> ShowPlanarRegionsTopic = Root.child(PlanarRegion).topic(Show);

   public static final Topic<Boolean> ComputePathTopic = Root.child(FootstepPlan).topic(ComputePath);
   public static final Topic<FootstepPlannerParameters> PlannerParametersTopic = Root.child(FootstepPlan).topic(FootstepPlannerParameters);
   public static final Topic<FootstepPlannerType> PlannerTypeTopic = Root.child(PlanarRegion).topic(Data);

   public static final Topic<Boolean> StartPositionEditModeEnabledTopic = Root.child(Start).child(EditMode).child(PositionTheme).topic(Enable);
   public static final Topic<Boolean> GoalPositionEditModeEnabledTopic = Root.child(Goal).child(EditMode).child(PositionTheme).topic(Enable);

   public static final Topic<Boolean> StartOrientationEditModeEnabledTopic = Root.child(Start).child(EditMode).child(OrientationTheme).topic(Enable);
   public static final Topic<Boolean> GoalOrientationEditModeEnabledTopic = Root.child(Goal).child(EditMode).child(OrientationTheme).topic(Enable);

   public static final Topic<Point3D> StartPositionTopic = Root.child(Start).topic(Position);
   public static final Topic<Point3D> GoalPositionTopic = Root.child(Goal).topic(Position);

   public static final Topic<Double> StartOrientationTopic = Root.child(Start).topic(Orientation);
   public static final Topic<Double> GoalOrientationTopic = Root.child(Goal).topic(Orientation);

   public static final Topic<Boolean> GlobalResetTopic = Root.topic(Reset);

   public static final API API = apiFactory.getAPIAndCloseFactory();
}