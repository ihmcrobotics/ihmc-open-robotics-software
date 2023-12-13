package us.ihmc.behaviors.lookAndStep;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import perception_msgs.msg.dds.PlanarRegionsListMessage;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.tools.Timer;
import us.ihmc.tools.TimerSnapshotWithExpiration;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.behaviors.tools.interfaces.ROS2TypedPublisherInterface;
import us.ihmc.behaviors.tools.interfaces.StatusLogger;
import us.ihmc.idl.IDLSequence;
import us.ihmc.robotics.geometry.ConvexPolygonScaler;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.util.ArrayList;
import java.util.List;

/**
 * @deprecated This class isn't currently used.
 */
public class LookAndStepSupportRegionsPublisher
{
   private ResettableExceptionHandlingExecutorService executor;
   private BehaviorTaskSuppressor suppressor;
   private StatusLogger statusLogger;
   private LookAndStepBehaviorParametersReadOnly lookAndStepBehaviorParameters;
   private ROS2TypedPublisherInterface ros2Publisher;
   private final TypedInput<CapturabilityBasedStatus> capturabilityBasedStatusInput = new TypedInput<>();
   private final Timer capturabilityBasedStatusExpirationTimer = new Timer();
   protected TimerSnapshotWithExpiration capturabilityBasedStatusReceptionTimerSnapshot;
   private final Timer planarRegionsExpirationTimer = new Timer();
   private final TypedInput<PlanarRegionsList> planarRegionsInput = new TypedInput<>();
   protected TimerSnapshotWithExpiration planarRegionReceptionTimerSnapshot;
   protected PlanarRegionsList planarRegions;
   protected CapturabilityBasedStatus capturabilityBasedStatus;

   public void initialize(StatusLogger statusLogger,
                          LookAndStepBehaviorParametersReadOnly lookAndStepBehaviorParameters,
                          ROS2TypedPublisherInterface ros2Publisher)
   {
      this.statusLogger = statusLogger;
      this.lookAndStepBehaviorParameters = lookAndStepBehaviorParameters;
      this.ros2Publisher = ros2Publisher;

      executor = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);

      suppressor = new BehaviorTaskSuppressor(statusLogger, "Support regions publisher");
      suppressor.addCondition(() -> "Regions expired. haveReceivedAny: " + planarRegionReceptionTimerSnapshot.hasBeenSet()
                                    + " timeSinceLastUpdate: " + planarRegionReceptionTimerSnapshot.getTimePassedSinceReset(),
                              () -> planarRegionReceptionTimerSnapshot.isExpired());
      suppressor.addCondition(() -> "No regions. "
                                    + (planarRegions == null ? null : (" isEmpty: " + planarRegions.isEmpty())),
                              () -> !(planarRegions != null && !planarRegions.isEmpty()));
      suppressor.addCondition(() -> "Capturability based status expired. haveReceivedAny: " + capturabilityBasedStatusExpirationTimer.hasBeenSet()
                                    + " timeSinceLastUpdate: " + capturabilityBasedStatusReceptionTimerSnapshot.getTimePassedSinceReset(),
                              () -> capturabilityBasedStatusReceptionTimerSnapshot.isExpired());
      suppressor.addCondition(() -> "No capturability based status. ", () -> capturabilityBasedStatus == null);
   }

   public void queuePublish()
   {
      executor.clearQueueAndExecute(this::evaluateAndRun);
   }

   public void acceptCapturabilityBasedStatus(CapturabilityBasedStatus capturabilityBasedStatus)
   {
      capturabilityBasedStatusInput.set(capturabilityBasedStatus);
   }

   public void acceptPlanarRegions(PlanarRegionsListMessage planarRegionsListMessage)
   {
      acceptPlanarRegions(PlanarRegionMessageConverter.convertToPlanarRegionsList(planarRegionsListMessage));
   }

   public void acceptPlanarRegions(PlanarRegionsList planarRegionsList)
   {
      planarRegionsInput.set(planarRegionsList);
      planarRegionsExpirationTimer.reset();
   }

   private void evaluateAndRun()
   {
      planarRegions = planarRegionsInput.getLatest();
      planarRegionReceptionTimerSnapshot = planarRegionsExpirationTimer.createSnapshot(lookAndStepBehaviorParameters.getPlanarRegionsExpiration());
      capturabilityBasedStatus = capturabilityBasedStatusInput.getLatest();
      capturabilityBasedStatusReceptionTimerSnapshot
            = capturabilityBasedStatusExpirationTimer.createSnapshot(lookAndStepBehaviorParameters.getPlanarRegionsExpiration());

      if (suppressor.evaulateShouldAccept())
      {
         performTask();
      }
   }

   /**
    * Under development
    * Ideas:
    * - If feet are squared up, maybe assume space between is filled?
    */
   private void performTask()
   {
      // merge support regions in if < 2 steps taken
      //      if (numberOfCompletedFootsteps < 2)
      IDLSequence.Object<Point3D> leftPolygon = capturabilityBasedStatus.getLeftFootSupportPolygon3d();
      IDLSequence.Object<Point3D> rightPolygon = capturabilityBasedStatus.getRightFootSupportPolygon3d();

      List<Point3D> allPoints = new ArrayList<>();

      // if any point has no region under it
      int numberOfPointsWithNoRegion = 0;
      for (Point3D point3D : leftPolygon)
      {
         if (PlanarRegionTools.projectPointToPlanesVertically(point3D, planarRegions) == null)
         {
            ++numberOfPointsWithNoRegion;
         }
      }

      if (numberOfPointsWithNoRegion > 0)
      {
         statusLogger.info("{} left foot vertices with no region", numberOfPointsWithNoRegion);

         allPoints.addAll(leftPolygon);
      }

      numberOfPointsWithNoRegion = 0;
      for (Point3D point3D : rightPolygon)
      {
         if (PlanarRegionTools.projectPointToPlanesVertically(point3D, planarRegions) == null)
         {
            ++numberOfPointsWithNoRegion;
         }
      }

      if (numberOfPointsWithNoRegion > 0)
      {
         statusLogger.info("{} right foot vertices with no region", numberOfPointsWithNoRegion);

         allPoints.addAll(rightPolygon);
      }

      if (allPoints.size() >= 3)
      {
         // find place using first 3 points
         Plane3D plane = new Plane3D(allPoints.get(0), allPoints.get(1), allPoints.get(2));
         if (plane.getNormal().getZ() < 0)
         {
            plane.getNormal().negate();
         }

         // find transform of plane
         Quaternion fromZUp = new Quaternion();
         EuclidGeometryTools.orientation3DFromFirstToSecondVector3D(Axis3D.Z, plane.getNormal(), fromZUp);

         RigidBodyTransform transform = new RigidBodyTransform(fromZUp, plane.getPoint());

         PoseReferenceFrame planeFrame = new PoseReferenceFrame("PlaneFrame", ReferenceFrame.getWorldFrame());
         planeFrame.setPoseAndUpdate(plane.getPoint(), fromZUp);

         ConvexPolygon2D polygon2D = new ConvexPolygon2D();
         for (Point3D planarPoint3D : allPoints)
         {
            FramePoint3D framePoint3D = new FramePoint3D(ReferenceFrame.getWorldFrame(), planarPoint3D);
            framePoint3D.changeFrame(planeFrame);
            polygon2D.addVertex(framePoint3D);
         }
         polygon2D.update();

         ConvexPolygonScaler convexPolygonScaler = new ConvexPolygonScaler();
         ConvexPolygon2D grownPolygon = new ConvexPolygon2D();
         convexPolygonScaler.scaleConvexPolygon(polygon2D, -0.07, grownPolygon);

         PlanarRegion supportRegion = new PlanarRegion(transform, grownPolygon);
         planarRegions.addPlanarRegion(supportRegion);

         PlanarRegionsListMessage message = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(supportRegion);
         ros2Publisher.publishROS2(PerceptionAPI.REA_SUPPORT_REGIONS_INPUT, message);
         statusLogger.info("Published region with {} vertices", supportRegion.getConvexPolygon(0).getNumberOfVertices());
      }
      else
      {
         statusLogger.error("Did not add support region");
      }
   }
}
