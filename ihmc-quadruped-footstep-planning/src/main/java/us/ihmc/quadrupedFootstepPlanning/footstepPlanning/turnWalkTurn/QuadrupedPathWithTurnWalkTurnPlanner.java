package us.ihmc.quadrupedFootstepPlanning.footstepPlanning.turnWalkTurn;

import controller_msgs.msg.dds.QuadrupedGroundPlaneMessage;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.pathPlanning.bodyPathPlanner.BodyPathPlanner;
import us.ihmc.pathPlanning.bodyPathPlanner.WaypointDefinedBodyPathPlanner;
import us.ihmc.pathPlanning.visibilityGraphs.tools.BodyPathPlan;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedOrientedStep;
import us.ihmc.quadrupedBasics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.*;
import us.ihmc.quadrupedFootstepPlanning.pathPlanning.WaypointsForQuadrupedFootstepPlanner;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsBasics;
import us.ihmc.quadrupedPlanning.footstepChooser.PlanarGroundPointFootSnapper;
import us.ihmc.quadrupedPlanning.footstepChooser.PlanarRegionBasedPointFootSnapper;
import us.ihmc.quadrupedPlanning.footstepChooser.PointFootSnapperParameters;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.time.TimeIntervalTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.List;

public abstract class QuadrupedPathWithTurnWalkTurnPlanner implements QuadrupedBodyPathAndFootstepPlanner
{
   protected final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());



   private final BodyPathPlanner bodyPathPlanner = new WaypointDefinedBodyPathPlanner();
   private final WaypointsForQuadrupedFootstepPlanner waypointPathPlanner;

   private final QuadrupedTurnWalkTurnPathPlanner quadBodyPathPlanner;
   private final QuadrupedXGaitStepCalculator stepCalculator;
   private final PlanarGroundPointFootSnapper groundPlaneSnapper;
   private final PlanarRegionBasedPointFootSnapper planarRegionSnapper;


   public QuadrupedPathWithTurnWalkTurnPlanner(WaypointsForQuadrupedFootstepPlanner waypointPathPlanner, QuadrupedXGaitSettingsBasics xGaitSettings,
                                               YoDouble timestamp, PointFootSnapperParameters pointFootSnapperParameters,
                                               QuadrupedReferenceFrames referenceFrames, YoGraphicsListRegistry graphicsListRegistry,
                                               YoVariableRegistry parentRegistry)
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
  public double getPlanningDuration()
   {
      return 0.0;
   }

   @Override
   public void setGroundPlane(QuadrupedGroundPlaneMessage message)
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
   public void setStart(QuadrupedFootstepPlannerStart start)
   {
      waypointPathPlanner.setInitialBodyPose(start.getTargetPose());
   }

   @Override
   public void setGoal(QuadrupedFootstepPlannerGoal goal)
   {
      waypointPathPlanner.setGoal(goal);
   }


   @Override
   public FootstepPlanningResult planPath()
   {
      FootstepPlanningResult result = waypointPathPlanner.planWaypoints();

      bodyPathPlanner.setWaypoints(waypointPathPlanner.getWaypoints());
      bodyPathPlanner.compute();

      return result;
   }

   @Override
   public FootstepPlanningResult plan()
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

      return FootstepPlanningResult.OPTIMAL_SOLUTION;
   }

   @Override
   public BodyPathPlan getPathPlan()
   {
      return bodyPathPlanner.getPlan();
   }

   @Override
   public FootstepPlan getPlan()
   {
      FootstepPlan plan = new FootstepPlan();
      List<QuadrupedTimedOrientedStep> steps = stepCalculator.getSteps();
      TimeIntervalTools.sortByStartTime(steps);
      double startTime = steps.get(0).getTimeInterval().getStartTime();
      steps.forEach(step -> step.getTimeInterval().shiftInterval(-startTime));
      steps.forEach(plan::addFootstep);
      return plan;
   }
}
