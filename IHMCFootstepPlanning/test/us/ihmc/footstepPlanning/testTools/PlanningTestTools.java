package us.ihmc.footstepPlanning.testTools;

import static org.junit.Assert.assertTrue;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.footstepPlanning.AnytimeFootstepPlanner;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlanner;
import us.ihmc.footstepPlanning.FootstepPlannerGoal;
import us.ihmc.footstepPlanning.FootstepPlannerGoalType;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.footstepPlanning.SimpleFootstep;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPolygon;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.tools.thread.ThreadTools;

public class PlanningTestTools
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final AppearanceDefinition[] appearances = { YoAppearance.White(), YoAppearance.Grey(), YoAppearance.DarkGray() };

   public static ConvexPolygon2d createDefaultFootPolygon()
   {
      double footLength = 0.2;
      double footWidth = 0.1;

      ConvexPolygon2d footPolygon = new ConvexPolygon2d();
      footPolygon.addVertex(footLength / 2.0, footWidth / 2.0);
      footPolygon.addVertex(footLength / 2.0, -footWidth / 2.0);
      footPolygon.addVertex(-footLength / 2.0, footWidth / 2.0);
      footPolygon.addVertex(-footLength / 2.0, -footWidth / 2.0);
      footPolygon.update();

      return footPolygon;
   }

   public static SideDependentList<ConvexPolygon2d> createDefaultFootPolygons()
   {
      SideDependentList<ConvexPolygon2d> footPolygons = new SideDependentList<>();
      for (RobotSide side : RobotSide.values)
         footPolygons.put(side, PlanningTestTools.createDefaultFootPolygon());
      return footPolygons;
   }

   public static void visualizeAndSleep(PlanarRegionsList planarRegionsList, FootstepPlan footseps, FramePose goalPose)
   {
      visualizeAndSleep(planarRegionsList, footseps, goalPose, null, null);
   }

   public static void visualizeAndSleep(PlanarRegionsList planarRegionsList, FootstepPlan footseps)
   {
      visualizeAndSleep(planarRegionsList, footseps, null, null, null);
   }

   public static void visualizeAndSleep(PlanarRegionsList planarRegionsList, FootstepPlan footseps, FramePose goalPose, YoVariableRegistry registry, YoGraphicsListRegistry graphicsListRegistry)
   {
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("Dummy"));
      if (registry != null)
         scs.addYoVariableRegistry(registry);
      if (graphicsListRegistry != null)
         scs.addYoGraphicsListRegistry(graphicsListRegistry, true);

      Graphics3DObject graphics3DObject = new Graphics3DObject();
      graphics3DObject.addCoordinateSystem(0.3);
      if (planarRegionsList != null)
         graphics3DObject.addPlanarRegionsList(planarRegionsList, appearances);
      scs.addStaticLinkGraphics(graphics3DObject);

      YoVariableRegistry vizRegistry = new YoVariableRegistry("FootstepPlanningResult");
      YoGraphicsListRegistry vizGraphicsListRegistry = new YoGraphicsListRegistry();

      if (goalPose != null)
         addGoalViz(goalPose, vizRegistry, vizGraphicsListRegistry);

      if (footseps != null)
      {
         YoFrameConvexPolygon2d yoDefaultFootPolygon = new YoFrameConvexPolygon2d("DefaultFootPolygon", worldFrame, 4, vizRegistry);
         yoDefaultFootPolygon.setConvexPolygon2d(createDefaultFootPolygon());

         int numberOfSteps = footseps.getNumberOfSteps();

         for (int i = 0; i < numberOfSteps; i++)
         {
            SimpleFootstep footstep = footseps.getFootstep(i);
            FramePose footstepPose = new FramePose();
            footstep.getSoleFramePose(footstepPose);

            AppearanceDefinition appearance = footstep.getRobotSide() == RobotSide.RIGHT ? YoAppearance.Green() : YoAppearance.Red();
            YoFramePose yoFootstepPose = new YoFramePose("footPose" + i, worldFrame, vizRegistry);
            yoFootstepPose.set(footstepPose);

            if (!footstep.hasFoothold())
            {
               YoGraphicPolygon footstepViz = new YoGraphicPolygon("footstep" + i, yoDefaultFootPolygon, yoFootstepPose, 1.0, appearance);
               vizGraphicsListRegistry.registerYoGraphic("viz", footstepViz);
            }
            else
            {
               YoGraphicPolygon fullFootstepViz = new YoGraphicPolygon("fullFootstep" + i, yoDefaultFootPolygon, yoFootstepPose, 1.0, YoAppearance.Glass(0.7));
               vizGraphicsListRegistry.registerYoGraphic("viz", fullFootstepViz);

               ConvexPolygon2d foothold = new ConvexPolygon2d();
               footstep.getFoothold(foothold);
               ConvexPolygonTools.limitVerticesConservative(foothold, 4);
               YoFrameConvexPolygon2d yoFoothold = new YoFrameConvexPolygon2d("Foothold" + i, worldFrame, 4, vizRegistry);
               yoFoothold.setConvexPolygon2d(foothold);
               YoGraphicPolygon footstepViz = new YoGraphicPolygon("footstep" + i, yoFoothold, yoFootstepPose, 1.0, appearance);
               vizGraphicsListRegistry.registerYoGraphic("viz", footstepViz);
            }
         }
      }

      scs.addYoVariableRegistry(vizRegistry);
      scs.addYoGraphicsListRegistry(vizGraphicsListRegistry, true);
      scs.startOnAThread();
      ThreadTools.sleepForever();
   }

   public static void addGoalViz(FramePose goalPose, YoVariableRegistry registry, YoGraphicsListRegistry graphicsListRegistry)
   {
      YoFramePoint yoGoal = new YoFramePoint("GoalPosition", worldFrame, registry);
      yoGoal.set(goalPose.getFramePointCopy());
      graphicsListRegistry.registerYoGraphic("viz", new YoGraphicPosition("GoalViz", yoGoal, 0.05, YoAppearance.White()));
      PoseReferenceFrame goalFrame = new PoseReferenceFrame("GoalFrame", goalPose);
      FrameVector goalOrientation = new FrameVector(goalFrame, 0.5, 0.0, 0.0);
      goalOrientation.changeFrame(worldFrame);
      YoFrameVector yoGoalOrientation = new YoFrameVector("GoalVector", worldFrame, registry);
      yoGoalOrientation.set(goalOrientation);
      graphicsListRegistry.registerYoGraphic("vizOrientation", new YoGraphicVector("GoalOrientationViz", yoGoal, yoGoalOrientation, 1.0, YoAppearance.White()));
   }

   public static FootstepPlan runPlanner(FootstepPlanner planner, FramePose initialStanceFootPose, RobotSide initialStanceSide, FramePose goalPose, PlanarRegionsList planarRegionsList)
   {
      return runPlanner(planner, initialStanceFootPose, initialStanceSide, goalPose, planarRegionsList, true);
   }

   public static FootstepPlan runPlanner(FootstepPlanner planner, FramePose initialStanceFootPose, RobotSide initialStanceSide, FramePose goalPose, PlanarRegionsList planarRegionsList, boolean assertPlannerReturnedResult)
   {
      FootstepPlannerGoal goal = new FootstepPlannerGoal();
      goal.setFootstepPlannerGoalType(FootstepPlannerGoalType.POSE_BETWEEN_FEET);
      goal.setGoalPoseBetweenFeet(goalPose);
      goal.setXYGoal(new Point2D(goalPose.getX(), goalPose.getY()), 0.5);

      return runPlanner(planner, initialStanceFootPose, initialStanceSide, goal, planarRegionsList, assertPlannerReturnedResult);
   }

   public static FootstepPlan runPlanner(FootstepPlanner planner, FramePose initialStanceFootPose, RobotSide initialStanceSide, FootstepPlannerGoal goal, PlanarRegionsList planarRegionsList, boolean assertPlannerReturnedResult)
   {
      planner.setInitialStanceFoot(initialStanceFootPose, initialStanceSide);
      planner.setGoal(goal);
      planner.setPlanarRegions(planarRegionsList);

      ExecutionTimer timer = new ExecutionTimer("Timer", 0.0, new YoVariableRegistry("Timer"));
      timer.startMeasurement();
      FootstepPlanningResult result = planner.plan();
      timer.stopMeasurement();
      PrintTools.info("Planning took " + timer.getCurrentTime().getDoubleValue() + "s");

      FootstepPlan footstepPlan = planner.getPlan();
      if (assertPlannerReturnedResult) assertTrue("Planner was not able to provide valid result.", result.validForExecution());
      return footstepPlan;
   }

   public static void configureAnytimePlannerRunnable(final AnytimeFootstepPlanner planner, FramePose initialStanceFootPose, RobotSide initialStanceSide, FramePose goalPose, PlanarRegionsList planarRegionsList)
   {
      FootstepPlannerGoal goal = new FootstepPlannerGoal();
      goal.setFootstepPlannerGoalType(FootstepPlannerGoalType.POSE_BETWEEN_FEET);
      goal.setGoalPoseBetweenFeet(goalPose);

      planner.setInitialStanceFoot(initialStanceFootPose, initialStanceSide);
      planner.setGoal(goal);
      planner.setPlanarRegions(planarRegionsList);
   }

   public static boolean isGoalNextToLastStep(FramePose goalPose, FootstepPlan footstepPlan)
   {
      return isGoalNextToLastStep(goalPose, footstepPlan, 0.5);
   }

   public static boolean isGoalNextToLastStep(FramePose goalPose, FootstepPlan footstepPlan, double epsilon)
   {
      int steps = footstepPlan.getNumberOfSteps();
      if (steps < 1)
         throw new RuntimeException("Did not get enough footsteps to check if goal is within feet.");

      SimpleFootstep footstep = footstepPlan.getFootstep(steps - 1);
      FramePose stepPose = new FramePose();
      footstep.getSoleFramePose(stepPose);
      RobotSide stepSide = footstep.getRobotSide();

      double midFeetOffset = stepSide.negateIfLeftSide(0.125);
      Vector3D goalOffset = new Vector3D(0.0, midFeetOffset , 0.0);
      RigidBodyTransform soleToWorld = new RigidBodyTransform();
      stepPose.getRigidBodyTransform(soleToWorld);
      soleToWorld.transform(goalOffset);

      FramePose achievedGoal = new FramePose(stepPose);
      Point3D goalPosition = new Point3D();
      achievedGoal.getPosition(goalPosition);
      goalPosition.add(goalOffset);
      achievedGoal.setPosition(goalPosition);

      if (achievedGoal.epsilonEquals(goalPose, epsilon))
         return true;
      else
         return false;
   }
}
