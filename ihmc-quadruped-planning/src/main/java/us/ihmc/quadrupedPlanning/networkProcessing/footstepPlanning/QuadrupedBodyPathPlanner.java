package us.ihmc.quadrupedPlanning.networkProcessing.footstepPlanning;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.QuadrupedFootstepStatusMessage;
import controller_msgs.msg.dds.QuadrupedGroundPlaneMessage;
import controller_msgs.msg.dds.QuadrupedXGaitSettingsPacket;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.pathPlanning.bodyPathPlanner.WaypointDefinedBodyPathPlanner;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedBasics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedPlanning.QuadrupedFootstepPlannerGoal;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.quadrupedPlanning.YoQuadrupedXGaitSettings;
import us.ihmc.quadrupedPlanning.bodyPath.QuadrupedWaypointBasedBodyPathProvider;
import us.ihmc.quadrupedPlanning.footstepChooser.PlanarGroundPointFootSnapper;
import us.ihmc.quadrupedPlanning.footstepChooser.PlanarRegionBasedPointFootSnapper;
import us.ihmc.quadrupedPlanning.footstepChooser.PointFootSnapperParameters;
import us.ihmc.quadrupedPlanning.pathPlanning.SplinePathPlanner;
import us.ihmc.quadrupedPlanning.stepStream.QuadrupedXGaitStepStream;
import us.ihmc.quadrupedPlanning.velocityPlanning.DefaultConstantAccelerationBodyPathParameters;
import us.ihmc.quadrupedPlanning.velocityPlanning.QuadrupedConstantAccelerationBodyPathPlanner;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.List;
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


   public QuadrupedBodyPathPlanner(QuadrupedXGaitSettingsReadOnly defaultXGaitSettings, PointFootSnapperParameters pointFootSnapperParameters,
                                   QuadrupedReferenceFrames referenceFrames, YoGraphicsListRegistry graphicsListRegistry,
                                   YoVariableRegistry parentRegistry)
   {
      xGaitSettings = new YoQuadrupedXGaitSettings(defaultXGaitSettings, registry);
      firstStepDelay.set(0.5);

      quadBodyPathPlanner = new QuadrupedConstantAccelerationBodyPathPlanner(new DefaultConstantAccelerationBodyPathParameters(), registry);
      waypointBasedPath = new QuadrupedWaypointBasedBodyPathProvider(referenceFrames, timestamp, graphicsListRegistry, registry);
      stepStream = new QuadrupedXGaitStepStream(xGaitSettings, timestamp, waypointBasedPath, firstStepDelay, registry);

      groundPlaneSnapper = new PlanarGroundPointFootSnapper(referenceFrames);
      planarRegionSnapper = new PlanarRegionBasedPointFootSnapper(pointFootSnapperParameters);
      planarRegionSnapper.setFallbackSnapper(groundPlaneSnapper);
      stepStream.setStepSnapper(planarRegionSnapper);

      parentRegistry.addChild(registry);
   }

   public void processPlanarRegionsListMessage(PlanarRegionsListMessage message)
   {
      PlanarRegionsList planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsList(message);
      setPlanarRegions(planarRegionsList);
   }

   public void processGroundPlaneMessage(QuadrupedGroundPlaneMessage message)
   {
      groundPlaneSnapper.submitGroundPlane(message);
   }

   public void processXGaitSettingsPacket(QuadrupedXGaitSettingsPacket packet)
   {
      xGaitSettings.set(packet);
   }

   public void processTimestamp(long timestampInNanos)
   {
      this.timestampNanos.set(timestampInNanos);
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
      planarRegionSnapper.setPlanarRegionsList(planarRegionsList);
      waypointPlanner.setPlanarRegionsList(planarRegionsList);
   }

   public void initialize()
   {
      timestamp.set(Conversions.nanosecondsToSeconds(timestampNanos.get()));
//      paused.set(false);
      stepStream.onEntry();
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

   public List<? extends QuadrupedTimedStep> getSteps()
   {
      return stepStream.getSteps();
   }

}
