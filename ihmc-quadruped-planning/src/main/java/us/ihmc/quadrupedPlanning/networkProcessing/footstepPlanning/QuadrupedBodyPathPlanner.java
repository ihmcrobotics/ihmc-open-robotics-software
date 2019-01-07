package us.ihmc.quadrupedPlanning.velocityPlanning;

import us.ihmc.commons.Conversions;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.pathPlanning.bodyPathPlanner.WaypointDefinedBodyPathPlanner;
import us.ihmc.quadrupedBasics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedPlanning.QuadrupedFootstepPlannerGoal;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedPlanning.YoQuadrupedXGaitSettings;
import us.ihmc.quadrupedPlanning.bodyPath.QuadrupedWaypointBasedBodyPathProvider;
import us.ihmc.quadrupedPlanning.footstepChooser.PlanarGroundPointFootSnapper;
import us.ihmc.quadrupedPlanning.footstepChooser.PlanarRegionBasedPointFootSnapper;
import us.ihmc.quadrupedPlanning.pathPlanning.SplinePathPlanner;
import us.ihmc.quadrupedPlanning.stepStream.QuadrupedXGaitStepStream;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.concurrent.atomic.AtomicLong;

public class QuadrupedBodyPathPlanner
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final SplinePathPlanner waypointPlanner = new SplinePathPlanner(registry);
   protected final WaypointDefinedBodyPathPlanner bodyPathPlanner = new WaypointDefinedBodyPathPlanner();
   private final QuadrupedConstantAccelerationBodyPathPlanner quadBodyPathPlanner;
   private final QuadrupedWaypointBasedBodyPathProvider waypointBasedPath;
   private final QuadrupedXGaitStepStream stepStream;
   private final PlanarGroundPointFootSnapper groundPlaneSnapper;
   private final PlanarRegionBasedPointFootSnapper planarRegionSnapper;

   private final YoDouble timestamp = new YoDouble("timestamp", registry);
   private final YoQuadrupedXGaitSettings xGaitSettings;

   private final YoDouble firstStepDelay = new YoDouble("firstStepDelay", registry);

   private final AtomicLong timestampNanos = new AtomicLong();


   public QuadrupedBodyPathPlanner(QuadrupedXGaitSettingsReadOnly defaultXGaitSettings, QuadrupedReferenceFrames referenceFrames, YoGraphicsListRegistry graphicsListRegistry,
                                   YoVariableRegistry parentRegistry)
   {
      xGaitSettings = new YoQuadrupedXGaitSettings(defaultXGaitSettings, registry);
      firstStepDelay.set(0.5);

      quadBodyPathPlanner = new QuadrupedConstantAccelerationBodyPathPlanner(registry);
      waypointBasedPath = new QuadrupedWaypointBasedBodyPathProvider(referenceFrames, timestamp, graphicsListRegistry, registry);
      stepStream = new QuadrupedXGaitStepStream(xGaitSettings, timestamp, waypointBasedPath, firstStepDelay, registry);

      groundPlaneSnapper = new PlanarGroundPointFootSnapper(referenceFrames);
      planarRegionSnapper = new PlanarRegionBasedPointFootSnapper(null);//pointFootSnapperParameters);
      planarRegionSnapper.setFallbackSnapper(groundPlaneSnapper);
      stepStream.setStepSnapper(planarRegionSnapper);

      parentRegistry.addChild(registry);
   }

   public void setInitialBodyPose(FramePose3DReadOnly bodyPose)
   {
      waypointPlanner.setInitialBodyPose(bodyPose);
   }

   public void setGoal(QuadrupedFootstepPlannerGoal goal)
   {
      waypointPlanner.setGoal(goal);
   }

   public void setPlanarRegions(PlanarRegionsList planarRegionsList)
   {
      waypointPlanner.setPlanarRegionsList(planarRegionsList);
   }

   public void compute()
   {
      waypointPlanner.planWaypoints();

      bodyPathPlanner.setWaypoints(waypointPlanner.getWaypoints());
      bodyPathPlanner.compute();

      quadBodyPathPlanner.setBodyPathWaypoints(bodyPathPlanner.getPlan());
      quadBodyPathPlanner.computePlan();

      waypointBasedPath.setBodyPathPlan(quadBodyPathPlanner.getPlan());
   }

   public void update()
   {
      timestamp.set(Conversions.nanosecondsToSeconds(timestampNanos.get()));

      stepStream.process();
   }

}
