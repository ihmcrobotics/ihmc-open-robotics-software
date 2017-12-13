package us.ihmc.footstepPlanning.ui;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.FootstepPlannerType;
import us.ihmc.robotEnvironmentAwareness.communication.APIFactory;
import us.ihmc.robotEnvironmentAwareness.communication.APIFactory.*;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class FootstepPlannerUserInterfaceAPI
{
   private static final APIFactory apiFactory = new APIFactory();

   private static final CategoryTheme PlanarRegion = apiFactory.createCategoryTheme("PlanarRegion");
   private static final CategoryTheme Start = apiFactory.createCategoryTheme("Start");
   private static final CategoryTheme Goal = apiFactory.createCategoryTheme("Goal");
   private static final CategoryTheme EditMode = apiFactory.createCategoryTheme("EditMode");
   private static final CategoryTheme FootstepPlan = apiFactory.createCategoryTheme("FootstepPlan");

   private static final TopicTheme Parameters = apiFactory.createTopicTheme("Parameters");

   private static final TypedTopicTheme<Boolean> Enable = apiFactory.createTypedTopicTheme("Enable");
   private static final TypedTopicTheme<Boolean> Reset = apiFactory.createTypedTopicTheme("Reset");
   private static final TypedTopicTheme<Point3D> Position = apiFactory.createTypedTopicTheme("Position");
   private static final TypedTopicTheme<Boolean> ComputePath = apiFactory.createTypedTopicTheme("ComputePath");
   private static final TypedTopicTheme<FootstepPlannerType> FootstepPlannerType = apiFactory.createTypedTopicTheme("FootstepPlannerType");

   private static final TopicTheme Data = apiFactory.createTopicTheme("Data");

   private static final Category Root = apiFactory.getRootCategory(apiFactory.createCategoryTheme("FootstepPlanning"));

   public static final Topic<PlanarRegionsList> PlanarRegionData = Root.child(PlanarRegion).topic(Data);
   public static final Topic<FootstepPlannerType> SelectedPlannerType = Root.child(PlanarRegion).topic(Data);

   public static final Topic<Boolean> StartEditModeEnabled = Root.child(Start).child(EditMode).topic(Enable);
   public static final Topic<Boolean> GoalEditModeEnabled = Root.child(Goal).child(EditMode).topic(Enable);
   public static final Topic<Point3D> StartPosition = Root.child(Start).topic(Position);
   public static final Topic<Point3D> GoalPosition = Root.child(Goal).topic(Position);

   public static final Topic<Boolean> GlobalReset = Root.topic(Reset);

   public static final API API = apiFactory.getAPIAndCloseFactory();
}
