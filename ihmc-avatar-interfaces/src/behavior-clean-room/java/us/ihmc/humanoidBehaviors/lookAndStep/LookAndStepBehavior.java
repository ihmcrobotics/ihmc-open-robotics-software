package us.ihmc.humanoidBehaviors.lookAndStep;

import controller_msgs.msg.dds.WalkingStatusMessage;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.RemoteREAInterface;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.exceptions.NotARotationMatrixException;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.graphSearch.collision.FootstepNodeBodyCollisionDetector;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapData;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.SimplePlanarRegionFootstepNodeSnapper;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNode;
import us.ihmc.footstepPlanning.graphSearch.graph.FootstepNodeTools;
import us.ihmc.footstepPlanning.graphSearch.heuristics.DistanceAndYawBasedHeuristics;
import us.ihmc.footstepPlanning.graphSearch.nodeChecking.*;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.FootstepNodeExpansion;
import us.ihmc.footstepPlanning.graphSearch.nodeExpansion.ParameterBasedNodeExpansion;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.graphSearch.planners.AStarFootstepPlanner;
import us.ihmc.footstepPlanning.graphSearch.stepCost.EuclideanDistanceAndYawBasedCost;
import us.ihmc.footstepPlanning.graphSearch.stepCost.FootstepCost;
import us.ihmc.humanoidBehaviors.BehaviorDefinition;
import us.ihmc.humanoidBehaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.tools.BehaviorHelper;
import us.ihmc.humanoidBehaviors.tools.HumanoidRobotState;
import us.ihmc.humanoidBehaviors.tools.RemoteHumanoidRobotInterface;
import us.ihmc.humanoidRobotics.footstep.SimpleFootstep;
import us.ihmc.log.LogTools;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.Category;
import us.ihmc.messager.MessagerAPIFactory.CategoryTheme;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.thread.PausablePeriodicThread;
import us.ihmc.tools.thread.TypedNotification;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehavior.LookAndStepBehaviorAPI.*;

public class LookAndStepBehavior implements BehaviorInterface
{
   public static final BehaviorDefinition DEFINITION = new BehaviorDefinition("Look and Step", LookAndStepBehavior::new, create());

   private final LookAndStepBehaviorParameters parameters = new LookAndStepBehaviorParameters();

   private final BehaviorHelper helper;
   private final SideDependentList<ConvexPolygon2D> footPolygons;
   private final Notification takeStep;
   private final PausablePeriodicThread mainThread;
   private final RemoteREAInterface rea;

   private final FootstepPlannerParametersBasics footstepPlannerParameters;
   private final RemoteHumanoidRobotInterface robot;

   public LookAndStepBehavior(BehaviorHelper helper)
   {
      this.helper = helper;
      footPolygons = helper.createFootPolygons();
      rea = helper.getOrCreateREAInterface();
      robot = helper.getOrCreateRobotInterface();
      takeStep = helper.createUINotification(TakeStep);
      helper.createUICallback(Parameters, parameters::setAllFromStrings);
      mainThread = helper.createPausablePeriodicThread(getClass(), 0.1, this::lookAndStep);
      footstepPlannerParameters = helper.getRobotModel().getFootstepPlannerParameters();
   }

   @Override
   public void setEnabled(boolean enabled)
   {
      LogTools.info("Look and step behavior selected = {}", enabled);

      mainThread.setRunning(enabled);
      helper.setCommunicationCallbacksEnabled(enabled);
   }

   private void lookAndStep()
   {
      HumanoidRobotState latestHumanoidRobotState = robot.pollHumanoidRobotState();
      PlanarRegionsList latestPlanarRegionList = rea.getLatestPlanarRegionList();

      FramePose3D goalPoseBetweenFeet = new FramePose3D();
      goalPoseBetweenFeet.setToZero(latestHumanoidRobotState.getMidFeetZUpFrame());
      goalPoseBetweenFeet.changeFrame(ReferenceFrame.getWorldFrame());
      double midFeetZ = goalPoseBetweenFeet.getZ();

      goalPoseBetweenFeet.setToZero(latestHumanoidRobotState.getPelvisFrame());
      goalPoseBetweenFeet.appendTranslation(parameters.get(LookAndStepBehaviorParameters.stepLength), 0.0, 0.0);
      goalPoseBetweenFeet.changeFrame(ReferenceFrame.getWorldFrame());
      goalPoseBetweenFeet.setZ(midFeetZ);

      RobotSide initialStanceFootSide = null;
      FramePose3D initialStanceFootPose = null;
      FramePose3D leftSolePose = new FramePose3D();
      leftSolePose.setToZero(latestHumanoidRobotState.getSoleZUpFrame(RobotSide.LEFT));
      leftSolePose.changeFrame(ReferenceFrame.getWorldFrame());
      FramePose3D rightSolePose = new FramePose3D();
      rightSolePose.setToZero(latestHumanoidRobotState.getSoleZUpFrame(RobotSide.RIGHT));
      rightSolePose.changeFrame(ReferenceFrame.getWorldFrame());

      if (leftSolePose.getPosition().distance(goalPoseBetweenFeet.getPosition()) <= rightSolePose.getPosition().distance(goalPoseBetweenFeet.getPosition()))
      {
         initialStanceFootSide = RobotSide.LEFT;
         initialStanceFootPose = leftSolePose;
      }
      else
      {
         initialStanceFootSide = RobotSide.RIGHT;
         initialStanceFootPose = rightSolePose;
      }


      footstepPlannerParameters.setReturnBestEffortPlan(true);
      footstepPlannerParameters.setMaximumStepYaw(1.5);

      FootstepNodeSnapper snapper = new SimplePlanarRegionFootstepNodeSnapper(footPolygons);
      FootstepNodeChecker snapBasedNodeChecker = new SnapBasedNodeChecker(footstepPlannerParameters, footPolygons, snapper);
      FootstepNodeBodyCollisionDetector collisionDetector = new FootstepNodeBodyCollisionDetector(footstepPlannerParameters);
      BodyCollisionNodeChecker bodyCollisionNodeChecker = new BodyCollisionNodeChecker(collisionDetector, footstepPlannerParameters, snapper);
      PlanarRegionBaseOfCliffAvoider cliffAvoider = new PlanarRegionBaseOfCliffAvoider(footstepPlannerParameters, snapper, footPolygons);
      FootstepNodeChecker nodeChecker = new FootstepNodeCheckerOfCheckers(Arrays.asList(snapBasedNodeChecker, bodyCollisionNodeChecker, cliffAvoider));
      //      FootstepNodeChecker nodeChecker = new FootstepNodeCheckerOfCheckers(Arrays.asList(snapBasedNodeChecker));
      FootstepNodeExpansion nodeExpansion = new ParameterBasedNodeExpansion(footstepPlannerParameters);
      FootstepCost stepCostCalculator = new EuclideanDistanceAndYawBasedCost(footstepPlannerParameters);
      DistanceAndYawBasedHeuristics heuristics = new DistanceAndYawBasedHeuristics(snapper,
                                                                                   footstepPlannerParameters.getAStarHeuristicsWeight(),
                                                                                   footstepPlannerParameters);

      YoVariableRegistry registry = new YoVariableRegistry("footstepPlannerRegistry");
      AStarFootstepPlanner planner = new AStarFootstepPlanner(footstepPlannerParameters,
                                                              nodeChecker,
                                                              heuristics,
                                                              nodeExpansion,
                                                              stepCostCalculator,
                                                              snapper,
                                                              registry);



//      planner.setPlanningHorizonLength(100.0); // ??
      FootstepPlannerGoal footstepPlannerGoal = new FootstepPlannerGoal();
      footstepPlannerGoal.setFootstepPlannerGoalType(FootstepPlannerGoalType.POSE_BETWEEN_FEET);
      footstepPlannerGoal.setGoalPoseBetweenFeet(goalPoseBetweenFeet);
      planner.setPlanarRegions(latestPlanarRegionList);
      planner.setInitialStanceFoot(initialStanceFootPose, initialStanceFootSide);
      planner.setGoal(footstepPlannerGoal);
      planner.setBestEffortTimeout(2.0); // TODO tune

      FootstepPlanningResult result = planner.plan(); // TODO time and store or display
      FootstepPlan footstepPlan = planner.getPlan();

      FootstepPlan shortenedFootstepPlan = new FootstepPlan();
      if (footstepPlan.getNumberOfSteps() > 0)
      {
         shortenedFootstepPlan.addFootstep(footstepPlan.getFootstep(0));
      }

      // send footstep plan to UI
      helper.publishToUI(FootstepPlanForUI, FootstepDataMessageConverter.reduceFootstepPlanForUIMessager(footstepPlan));

      if (takeStep.poll())
      {
         // take step

         LogTools.info("Requesting walk");
         TypedNotification<WalkingStatusMessage> walkingStatusNotification = robot.requestWalk(FootstepDataMessageConverter.createFootstepDataListFromPlan(
               shortenedFootstepPlan,
               1.0,
               0.5,
               ExecutionMode.OVERRIDE), robot.pollHumanoidRobotState(), true, latestPlanarRegionList);

         walkingStatusNotification.blockingPoll();
      }
   }

   public static class LookAndStepBehaviorAPI
   {
      private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
      private static final Category RootCategory = apiFactory.createRootCategory("LookAndStepBehavior");
      private static final CategoryTheme LookAndStepTheme = apiFactory.createCategoryTheme("LookAndStep");

      public static final Topic<Object> TakeStep = topic("TakeStep");
      public static final Topic<ArrayList<Pair<RobotSide, Pose3D>>> FootstepPlanForUI = topic("FootstepPlan");
//      public static final Topic<PlanarRegionsList> MapRegionsForUI = topic("MapRegionsForUI");
      public static final Topic<List<String>> Parameters = topic("Parameters");

      private static final <T> Topic<T> topic(String name)
      {
         return RootCategory.child(LookAndStepTheme).topic(apiFactory.createTypedTopicTheme(name));
      }

      public static final MessagerAPIFactory.MessagerAPI create()
      {
         return apiFactory.getAPIAndCloseFactory();
      }
   }
}
