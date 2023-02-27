package us.ihmc.quadrupedFootstepPlanning.pawPlanning.turnWalkTurn;

import java.util.List;

import ihmc_common_msgs.msg.dds.GroundPlaneMessage;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.pathPlanning.bodyPathPlanner.BodyPathPlanHolder;
import us.ihmc.pathPlanning.bodyPathPlanner.WaypointDefinedBodyPathPlanHolder;
import us.ihmc.pathPlanning.visibilityGraphs.tools.BodyPathPlan;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedOrientedStep;
import us.ihmc.quadrupedBasics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedFootstepPlanning.pathPlanning.WaypointsForPawStepPlanner;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.BodyPathAndPawPlanner;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.PawStepPlan;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.PawStepPlanner;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.PawStepPlannerGoal;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.PawStepPlannerStart;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.PawStepPlanningResult;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsBasics;
import us.ihmc.quadrupedPlanning.footstepChooser.PlanarGroundPointFootSnapper;
import us.ihmc.quadrupedPlanning.footstepChooser.PlanarRegionBasedPointFootSnapper;
import us.ihmc.quadrupedPlanning.footstepChooser.PointFootSnapperParameters;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.time.TimeIntervalTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public abstract class QuadrupedPathWithTurnWalkTurnPlanner implements BodyPathAndPawPlanner
{
   protected final YoRegistry registry = new YoRegistry(getClass().getSimpleName());



   private final BodyPathPlanHolder bodyPathPlanner = new WaypointDefinedBodyPathPlanHolder();
   private final WaypointsForPawStepPlanner waypointPathPlanner;

   private final QuadrupedTurnWalkTurnPathPlanner quadBodyPathPlanner;
   private final QuadrupedXGaitStepCalculator stepCalculator;
   private final PlanarGroundPointFootSnapper groundPlaneSnapper;
   private final PlanarRegionBasedPointFootSnapper planarRegionSnapper;


   public QuadrupedPathWithTurnWalkTurnPlanner(WaypointsForPawStepPlanner waypointPathPlanner, QuadrupedXGaitSettingsBasics xGaitSettings,
                                               YoDouble timestamp, PointFootSnapperParameters pointFootSnapperParameters,
                                               QuadrupedReferenceFrames referenceFrames, YoGraphicsListRegistry graphicsListRegistry,
                                               YoRegistry parentRegistry)
   {
      this.waypointPathPlanner = waypointPathPlanner;
      YoDouble firstStepDelay = new YoDouble("firstStepDelay", registry);
      firstStepDelay.set(0.5);

      quadBodyPathPlanner = new QuadrupedTurnWalkTurnPathPlanner(new DefaultTurnWalkTurnPathParameters(), bodyPathPlanner, registry);
      stepCalculator = new QuadrupedXGaitStepCalculator(xGaitSettings, timestamp, referenceFrames, firstStepDelay, graphicsListRegistry, registry);

      groundPlaneSnapper = new PlanarGroundPointFootSnapper(referenceFrames);
      planarRegionSnapper = new PlanarRegionBasedPointFootSnapper(pointFootSnapperParameters);
      planarRegionSnapper.setFallbackSnapper(groundPlaneSnapper);
      stepCalculator.setStepSnapper(planarRegionSnapper);

      parentRegistry.addChild(registry);
   }

   @Override
   public void setTimeout(double timeout)
   {
   }

   @Override
   public void cancelPlanning()
   {
   }


   @Override
   public WaypointsForPawStepPlanner getWaypointPathPlanner()
   {
      return waypointPathPlanner;
   }

   @Override
   public PawStepPlanner getPawStepPlanner()
   {
      return this;
   }

   @Override
  public double getPlanningDuration()
   {
      return 0.0;
   }

   @Override
   public void setGroundPlane(GroundPlaneMessage message)
   {
      groundPlaneSnapper.submitGroundPlane(message);
   }

   @Override
   public void setPlanarRegionsList(PlanarRegionsList planarRegionsList)
   {
      planarRegionSnapper.setPlanarRegionsList(planarRegionsList);
      waypointPathPlanner.setPlanarRegionsList(planarRegionsList);
   }

   @Override
   public void setStart(PawStepPlannerStart start)
   {
      waypointPathPlanner.setInitialBodyPose(start.getTargetPose());
   }

   @Override
   public void setGoal(PawStepPlannerGoal goal)
   {
      waypointPathPlanner.setGoal(goal);
   }


   @Override
   public PawStepPlanningResult planPath()
   {
      PawStepPlanningResult result = waypointPathPlanner.planWaypoints();

      bodyPathPlanner.setPoseWaypoints(waypointPathPlanner.getWaypoints());

      return result;
   }

   @Override
   public PawStepPlanningResult plan()
   {
      BodyPathPlan bodyPathPlan = bodyPathPlanner.getPlan();
      bodyPathPlan.setStartPose(waypointPathPlanner.getInitialBodyPose().getPosition().getX(), waypointPathPlanner.getInitialBodyPose().getPosition().getY(), waypointPathPlanner
            .getInitialBodyPose().getYaw());
      bodyPathPlan.setGoalPose(waypointPathPlanner.getGoalBodyPose().getPosition().getX(), waypointPathPlanner.getGoalBodyPose().getPosition().getY(), waypointPathPlanner
            .getGoalBodyPose().getYaw());

      quadBodyPathPlanner.computePlan();

      stepCalculator.setBodyPathPlan(quadBodyPathPlanner.getPlan());
      stepCalculator.setGoalPose(waypointPathPlanner.getGoalBodyPose());
      stepCalculator.onEntry();

      return PawStepPlanningResult.OPTIMAL_SOLUTION;
   }

   @Override
   public BodyPathPlan getPathPlan()
   {
      return bodyPathPlanner.getPlan();
   }

   @Override
   public PawStepPlan getPlan()
   {
      PawStepPlan plan = new PawStepPlan();
      List<QuadrupedTimedOrientedStep> steps = stepCalculator.getSteps();
      TimeIntervalTools.sortByStartTime(steps);
      double startTime = steps.get(0).getTimeInterval().getStartTime();
      steps.forEach(step -> step.getTimeInterval().shiftInterval(-startTime));
      steps.forEach(plan::addPawStep);
      return plan;
   }
}
