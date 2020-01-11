package us.ihmc.valkyrie.planner.ui;

import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.Category;
import us.ihmc.messager.MessagerAPIFactory.CategoryTheme;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.pathPlanning.DataSetName;

public class ValkyriePlannerMessagerAPI
{
   private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
   private static final Category Root = apiFactory.createRootCategory("ValkyriePlannerMessagerAPI");
   private static final CategoryTheme FootstepPlanner = apiFactory.createCategoryTheme("FootstepPlanner");

   public static final Topic<Boolean> doPlanning = topic("DoPlanning");
   public static final Topic<Boolean> haltPlanning = topic("HaltPlanning");
   public static final Topic<Boolean> sendPlanningResult = topic("SendPlanningResult");
   public static final Topic<Boolean> stopWalking = topic("StopWalking");
   public static final Topic<Boolean> placeGoal = topic("PlaceGoal");
   public static final Topic<Boolean> addWaypoint = topic("AddWaypoint");
   public static final Topic<Boolean> clearWaypoints = topic("ClearWaypoints");
   public static final Topic<DataSetName> dataSetSelected = topic("DataSetSelected");

   public static final Topic<Boolean> showRobot = topic("ShowRobot");
   public static final Topic<Boolean> showSolutionSteps = topic("ShowSolutionSteps");
   public static final Topic<Boolean> showDebugSteps = topic("ShowDebugSteps");

   public static final Topic<Pair<RigidBodyTransform, ConvexPolygon2D>> parentDebugStep = topic("ParentDebugStep");
   public static final Topic<Pair<RigidBodyTransform, ConvexPolygon2D>> childDebugStep = topic("ChildDebugStep");
   public static final Topic<RigidBodyTransform> idealDebugStep = topic("IdealDebugStep");

   public static final MessagerAPI API = apiFactory.getAPIAndCloseFactory();

   private static final <T> Topic<T> topic(String name)
   {
      return Root.child(FootstepPlanner).topic(apiFactory.createTypedTopicTheme(name));
   }
}
