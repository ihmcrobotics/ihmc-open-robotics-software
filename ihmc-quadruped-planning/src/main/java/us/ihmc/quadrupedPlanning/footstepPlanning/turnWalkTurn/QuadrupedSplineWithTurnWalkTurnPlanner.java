package us.ihmc.quadrupedPlanning.footstepPlanning.turnWalkTurn;

import controller_msgs.msg.dds.QuadrupedGroundPlaneMessage;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.pathPlanning.bodyPathPlanner.BodyPathPlanner;
import us.ihmc.pathPlanning.bodyPathPlanner.WaypointDefinedBodyPathPlanner;
import us.ihmc.pathPlanning.visibilityGraphs.tools.BodyPathPlan;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedBasics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedPlanning.QuadrupedFootstepPlannerGoal;
import us.ihmc.quadrupedPlanning.YoQuadrupedXGaitSettings;
import us.ihmc.quadrupedPlanning.bodyPath.QuadrupedWaypointBasedBodyPathProvider;
import us.ihmc.quadrupedPlanning.footstepChooser.PlanarGroundPointFootSnapper;
import us.ihmc.quadrupedPlanning.footstepChooser.PlanarRegionBasedPointFootSnapper;
import us.ihmc.quadrupedPlanning.footstepChooser.PointFootSnapperParameters;
import us.ihmc.quadrupedPlanning.footstepPlanning.QuadrupedBodyPathAndFootstepPlanner;
import us.ihmc.quadrupedPlanning.pathPlanning.SplinePathPlanner;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.List;

public class QuadrupedSplineWithTurnWalkTurnPlanner implements QuadrupedBodyPathAndFootstepPlanner
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final BodyPathPlanner bodyPathPlanner = new WaypointDefinedBodyPathPlanner();
   private final SplinePathPlanner waypointPlanner = new SplinePathPlanner(registry);

   private final QuadrupedTurnWalkTurnPathPlanner quadBodyPathPlanner;
   private final QuadrupedWaypointBasedBodyPathProvider waypointBasedPath;
   private final QuadrupedXGaitStepCalculator stepCalculator;
   private final PlanarGroundPointFootSnapper groundPlaneSnapper;
   private final PlanarRegionBasedPointFootSnapper planarRegionSnapper;


   public QuadrupedSplineWithTurnWalkTurnPlanner(YoQuadrupedXGaitSettings xGaitSettings, YoDouble timestamp, PointFootSnapperParameters pointFootSnapperParameters,
                                                 QuadrupedReferenceFrames referenceFrames, YoGraphicsListRegistry graphicsListRegistry,
                                                 YoVariableRegistry parentRegistry)
   {
      YoDouble firstStepDelay = new YoDouble("firstStepDelay", registry);
      firstStepDelay.set(0.5);

      quadBodyPathPlanner = new QuadrupedTurnWalkTurnPathPlanner(new DefaultTurnWalkTurnPathParameters(), bodyPathPlanner, registry);
      waypointBasedPath = new QuadrupedWaypointBasedBodyPathProvider(referenceFrames, timestamp, graphicsListRegistry, registry);
      stepCalculator = new QuadrupedXGaitStepCalculator(xGaitSettings, timestamp, waypointBasedPath, firstStepDelay, registry);

      groundPlaneSnapper = new PlanarGroundPointFootSnapper(referenceFrames);
      planarRegionSnapper = new PlanarRegionBasedPointFootSnapper(pointFootSnapperParameters);
      planarRegionSnapper.setFallbackSnapper(groundPlaneSnapper);
      stepCalculator.setStepSnapper(planarRegionSnapper);

      parentRegistry.addChild(registry);
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
      waypointPlanner.setPlanarRegionsList(planarRegionsList);
   }

   @Override
   public void setInitialBodyPose(FramePose3DReadOnly bodyPose)
   {
      waypointPlanner.setInitialBodyPose(bodyPose);
   }

   @Override
   public void setGoal(QuadrupedFootstepPlannerGoal goal)
   {
      waypointPlanner.setGoal(goal);
   }

   @Override
   public void initialize()
   {
   }


   @Override
   public void planPath()
   {
      waypointPlanner.planWaypoints();

      bodyPathPlanner.setWaypoints(waypointPlanner.getWaypoints());
      bodyPathPlanner.compute();
   }

   @Override
   public void plan()
   {
      BodyPathPlan bodyPathPlan = bodyPathPlanner.getPlan();
      bodyPathPlan.setStartPose(waypointPlanner.getInitialBodyPose().getPosition().getX(), waypointPlanner.getInitialBodyPose().getPosition().getY(), waypointPlanner.getInitialBodyPose().getYaw());
      bodyPathPlan.setGoalPose(waypointPlanner.getGoalBodyPose().getPosition().getX(), waypointPlanner.getGoalBodyPose().getPosition().getY(), waypointPlanner.getGoalBodyPose().getYaw());

      quadBodyPathPlanner.computePlan();

      waypointBasedPath.setBodyPathPlan(quadBodyPathPlanner.getPlan());

      stepCalculator.setGoalPose(waypointPlanner.getGoalBodyPose());
      stepCalculator.onEntry();
   }

   @Override
   public void update()
   {
//      stepPlanner.updateOnline();
   }

   @Override
   public BodyPathPlan getPathPlan()
   {
      return bodyPathPlanner.getPlan();
   }

   @Override
   public List<? extends QuadrupedTimedStep> getSteps()
   {
      return stepCalculator.getSteps();
   }
}
