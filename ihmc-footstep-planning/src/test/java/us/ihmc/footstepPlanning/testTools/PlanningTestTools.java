package us.ihmc.footstepPlanning.testTools;

import static org.junit.Assert.assertTrue;

import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
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
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.pathPlanning.bodyPathPlanner.BodyPathPlanner;
import us.ihmc.robotics.geometry.ConvexPolygonTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class PlanningTestTools
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final AppearanceDefinition[] appearances = {YoAppearance.LightGray(), YoAppearance.Grey(), YoAppearance.DarkGray()};

   private static final double footLength = 0.2;
   private static final double footWidth = 0.1;

   public static ConvexPolygon2D createFootPolygon(double footLength, double footWidth)
   {
      ConvexPolygon2D footPolygon = new ConvexPolygon2D();
      footPolygon.addVertex(footLength / 2.0, footWidth / 2.0);
      footPolygon.addVertex(footLength / 2.0, -footWidth / 2.0);
      footPolygon.addVertex(-footLength / 2.0, footWidth / 2.0);
      footPolygon.addVertex(-footLength / 2.0, -footWidth / 2.0);
      footPolygon.update();

      return footPolygon;
   }

   public static ConvexPolygon2D createDefaultFootPolygon()
   {
      return createFootPolygon(footLength, footWidth);
   }

   public static SideDependentList<ConvexPolygon2D> createDefaultFootPolygons()
   {
      return createFootPolygons(footLength, footWidth);
   }

   public static SideDependentList<ConvexPolygon2D> createFootPolygons(double footLength, double footWidth)
   {
      SideDependentList<ConvexPolygon2D> footPolygons = new SideDependentList<>();
      for (RobotSide side : RobotSide.values)
         footPolygons.put(side, PlanningTestTools.createFootPolygon(footLength, footWidth));
      return footPolygons;
   }

   public static void visualizeAndSleep(PlanarRegionsList planarRegionsList, FootstepPlan footseps, FramePose3D goalPose, BodyPathPlanner bodyPath)
   {
      visualizeAndSleep(planarRegionsList, footseps, goalPose, bodyPath, null, null);
   }

   public static void visualizeAndSleep(PlanarRegionsList planarRegionsList, FootstepPlan footseps, FramePose3D goalPose)
   {
      visualizeAndSleep(planarRegionsList, footseps, goalPose, null, null, null);
   }

   public static void visualizeAndSleep(PlanarRegionsList planarRegionsList, FootstepPlan footseps)
   {
      visualizeAndSleep(planarRegionsList, footseps, null, null, null, null);
   }

   public static void visualizeAndSleep(PlanarRegionsList planarRegionsList, FootstepPlan footseps, FramePose3D goalPose, BodyPathPlanner bodyPath,
                                        YoVariableRegistry registry, YoGraphicsListRegistry graphicsListRegistry)
   {
      SimulationConstructionSet scs = new SimulationConstructionSet(new Robot("Dummy"));
      if (registry != null)
         scs.addYoVariableRegistry(registry);
      if (graphicsListRegistry != null)
         scs.addYoGraphicsListRegistry(graphicsListRegistry, true);

      Graphics3DObject graphics3DObject = new Graphics3DObject();
//      graphics3DObject.addCoordinateSystem(0.3);
      if (planarRegionsList != null)
      {
         graphics3DObject.addPlanarRegionsList(planarRegionsList, appearances);
         scs.setGroundVisible(false);
      }
      scs.addStaticLinkGraphics(graphics3DObject);
      scs.setCameraPosition(-4.0, -4.0, 6.0);
      scs.setCameraFix(0.5, 0.0, 0.1);

      YoVariableRegistry vizRegistry = new YoVariableRegistry("FootstepPlanningResult");
      YoGraphicsListRegistry vizGraphicsListRegistry = new YoGraphicsListRegistry();

      if (goalPose != null)
         addGoalViz(goalPose, vizRegistry, vizGraphicsListRegistry);

      if (bodyPath != null)
      {
         int markers = 100;
         for (int i = 0; i < markers; i++)
         {
            double alpha = (double) i / (double) (markers - 1);
            Pose2D pose = new Pose2D();
            bodyPath.getPointAlongPath(alpha, pose);
            YoFramePoint yoPoint = new YoFramePoint("BodyPathPoint" + i, worldFrame, vizRegistry);
            yoPoint.set(pose.getPosition());
            YoGraphicPosition pointVis = new YoGraphicPosition("BodyPathPoint" + i, yoPoint, 0.025, YoAppearance.Blue());
            vizGraphicsListRegistry.registerYoGraphic("viz", pointVis);
         }
      }

      if (footseps != null)
      {
         YoFrameConvexPolygon2d yoDefaultFootPolygon = new YoFrameConvexPolygon2d("DefaultFootPolygon", worldFrame, 4, vizRegistry);
         yoDefaultFootPolygon.setConvexPolygon2d(createDefaultFootPolygon());

         int numberOfSteps = footseps.getNumberOfSteps();

         for (int i = 0; i < numberOfSteps; i++)
         {
            SimpleFootstep footstep = footseps.getFootstep(i);
            FramePose3D footstepPose = new FramePose3D();
            footstep.getSoleFramePose(footstepPose);

            AppearanceDefinition appearance = footstep.getRobotSide() == RobotSide.RIGHT ? YoAppearance.Green() : YoAppearance.Red();
            YoFramePose yoFootstepPose = new YoFramePose("footPose" + i, worldFrame, vizRegistry);
            yoFootstepPose.set(footstepPose);
            yoFootstepPose.setZ(yoFootstepPose.getZ() + (footstep.getRobotSide() == RobotSide.RIGHT ? 0.001 : 0.0));

            if (!footstep.hasFoothold())
            {
               YoGraphicPolygon footstepViz = new YoGraphicPolygon("footstep" + i, yoDefaultFootPolygon, yoFootstepPose, 1.0, appearance);
               vizGraphicsListRegistry.registerYoGraphic("viz", footstepViz);
            }
            else
            {
               YoGraphicPolygon fullFootstepViz = new YoGraphicPolygon("fullFootstep" + i, yoDefaultFootPolygon, yoFootstepPose, 1.0, YoAppearance.Glass(0.7));
               vizGraphicsListRegistry.registerYoGraphic("viz", fullFootstepViz);

               ConvexPolygon2D foothold = new ConvexPolygon2D();
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

   public static void addGoalViz(FramePose3D goalPose, YoVariableRegistry registry, YoGraphicsListRegistry graphicsListRegistry)
   {
      YoFramePoint yoGoal = new YoFramePoint("GoalPosition", worldFrame, registry);
      yoGoal.set(goalPose.getPosition());
      graphicsListRegistry.registerYoGraphic("viz", new YoGraphicPosition("GoalViz", yoGoal, 0.05, YoAppearance.Yellow()));
      YoFramePoint yoStart = new YoFramePoint("StartPosition", worldFrame, registry);
      graphicsListRegistry.registerYoGraphic("viz", new YoGraphicPosition("StartViz", yoStart, 0.05, YoAppearance.Blue()));
      PoseReferenceFrame goalFrame = new PoseReferenceFrame("GoalFrame", goalPose);
      FrameVector3D goalOrientation = new FrameVector3D(goalFrame, 0.5, 0.0, 0.0);
      goalOrientation.changeFrame(worldFrame);
      YoFrameVector yoGoalOrientation = new YoFrameVector("GoalVector", worldFrame, registry);
      yoGoalOrientation.set(goalOrientation);
//      graphicsListRegistry.registerYoGraphic("vizOrientation", new YoGraphicVector("GoalOrientationViz", yoGoal, yoGoalOrientation, 1.0, YoAppearance.White()));
   }

   public static FootstepPlan runPlanner(FootstepPlanner planner, FramePose3D initialStanceFootPose, RobotSide initialStanceSide, FramePose3D goalPose,
                                         PlanarRegionsList planarRegionsList)
   {
      return runPlanner(planner, initialStanceFootPose, initialStanceSide, goalPose, planarRegionsList, true);
   }

   public static FootstepPlan runPlanner(FootstepPlanner planner, FramePose3D initialStanceFootPose, RobotSide initialStanceSide, FramePose3D goalPose,
                                         PlanarRegionsList planarRegionsList, boolean assertPlannerReturnedResult)
   {
      FootstepPlannerGoal goal = new FootstepPlannerGoal();
      goal.setFootstepPlannerGoalType(FootstepPlannerGoalType.POSE_BETWEEN_FEET);
      goal.setGoalPoseBetweenFeet(goalPose);
      goal.setXYGoal(new Point2D(goalPose.getX(), goalPose.getY()), 0.5);

      return runPlanner(planner, initialStanceFootPose, initialStanceSide, goal, planarRegionsList, assertPlannerReturnedResult);
   }

   public static FootstepPlan runPlanner(FootstepPlanner planner, FramePose3D initialStanceFootPose, RobotSide initialStanceSide, FootstepPlannerGoal goal,
                                         PlanarRegionsList planarRegionsList, boolean assertPlannerReturnedResult)
   {
      planner.setPlanarRegions(planarRegionsList);
      planner.setInitialStanceFoot(initialStanceFootPose, initialStanceSide);
      planner.setGoal(goal);

      ExecutionTimer timer = new ExecutionTimer("Timer", 0.0, new YoVariableRegistry("Timer"));
      timer.startMeasurement();
      FootstepPlanningResult result = planner.plan();
      timer.stopMeasurement();
      PrintTools.info("Planning took " + timer.getCurrentTime().getDoubleValue() + "s");

      FootstepPlan footstepPlan = planner.getPlan();
      if (assertPlannerReturnedResult)
         assertTrue("Planner was not able to provide valid result. Result: " + result, result.validForExecution());
      return footstepPlan;
   }

   public static void configureAnytimePlannerRunnable(final AnytimeFootstepPlanner planner, FramePose3D initialStanceFootPose, RobotSide initialStanceSide,
                                                      FramePose3D goalPose, PlanarRegionsList planarRegionsList)
   {
      FootstepPlannerGoal goal = new FootstepPlannerGoal();
      goal.setFootstepPlannerGoalType(FootstepPlannerGoalType.POSE_BETWEEN_FEET);
      goal.setGoalPoseBetweenFeet(goalPose);

      planner.setInitialStanceFoot(initialStanceFootPose, initialStanceSide);
      planner.setGoal(goal);
      planner.setPlanarRegions(planarRegionsList);
   }

   public static boolean isGoalNextToLastStep(FramePose3D goalPose, FootstepPlan footstepPlan)
   {
      return isGoalNextToLastStep(goalPose, footstepPlan, 0.5);
   }

   public static boolean isGoalNextToLastStep(FramePose3D goalPose, FootstepPlan footstepPlan, double epsilon)
   {
      int steps = footstepPlan.getNumberOfSteps();
      if (steps < 1)
         throw new RuntimeException("Did not get enough footsteps to check if goal is within feet.");

      SimpleFootstep footstep = footstepPlan.getFootstep(steps - 1);
      FramePose3D stepPose = new FramePose3D();
      footstep.getSoleFramePose(stepPose);
      RobotSide stepSide = footstep.getRobotSide();

      double midFeetOffset = stepSide.negateIfLeftSide(0.125);
      Vector3D goalOffset = new Vector3D(0.0, midFeetOffset, 0.0);
      RigidBodyTransform soleToWorld = new RigidBodyTransform();
      stepPose.get(soleToWorld);
      soleToWorld.transform(goalOffset);

      FramePose3D achievedGoal = new FramePose3D(stepPose);
      Point3D goalPosition = new Point3D(achievedGoal.getPosition());
      goalPosition.add(goalOffset);
      achievedGoal.setPosition(goalPosition);

      if (achievedGoal.epsilonEquals(goalPose, epsilon))
         return true;
      else
         return false;
   }
}
