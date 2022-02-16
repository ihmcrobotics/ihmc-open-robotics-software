package us.ihmc.behaviors.lookAndStep;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.networkProcessor.supportingPlanarRegionPublisher.BipedalSupportPlanarRegionCalculator;
import us.ihmc.behaviors.tools.footstepPlanner.MinimalFootstep;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.Timer;
import us.ihmc.tools.TimerSnapshotWithExpiration;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.function.Consumer;
import java.util.stream.Collectors;

public class LookAndStepPlanarRegionsManager
{
   private final double minimumTranslationToAppend = 0.2;
   private final double minimumRotationToAppend = Math.toRadians(30);
   private final double maxDistanceToRemember = 2.0;

   private final LookAndStepBehaviorParametersReadOnly lookAndStepParameters;
   private final BipedalSupportPlanarRegionCalculator bipedalSupportPlanarRegionCalculator;
   private final ROS2SyncedRobotModel syncedRobot;

   private final Timer planarRegionsExpirationTimer = new Timer();
   private final Timer capturabilityBasedStatusExpirationTimer = new Timer();
   private final Timer robotConfigurationDataExpirationTimer = new Timer();

   private final TypedInput<PlanarRegionsList> planarRegionsInput = new TypedInput<>();
   private final TypedInput<CapturabilityBasedStatus> capturabilityBasedStatusInput = new TypedInput<>();
   private final TypedInput<RobotConfigurationData> robotConfigurationDataInput = new TypedInput<>();

   protected PlanarRegionsHistory planarRegionsHistory = new PlanarRegionsHistory();

   public LookAndStepPlanarRegionsManager(LookAndStepBehaviorParametersReadOnly lookAndStepParameters,
                                          DRCRobotModel robotModel,
                                          ROS2SyncedRobotModel syncedRobot)
   {
      this.lookAndStepParameters = lookAndStepParameters;
      bipedalSupportPlanarRegionCalculator = new BipedalSupportPlanarRegionCalculator(robotModel);
      this.syncedRobot = syncedRobot;
   }

   public void acceptPlanarRegions(PlanarRegionsList planarRegionsList)
   {
      planarRegionsInput.set(planarRegionsList);
      planarRegionsExpirationTimer.reset();
   }

   public void acceptCapturabilityBasedStatus(CapturabilityBasedStatus capturabilityBasedStatus)
   {
      capturabilityBasedStatusInput.set(capturabilityBasedStatus);
      capturabilityBasedStatusExpirationTimer.reset();
   }

   public void acceptRobotConfigurationData(RobotConfigurationData robotConfigurationData)
   {
      robotConfigurationDataInput.set(robotConfigurationData);
      robotConfigurationDataExpirationTimer.reset();
   }

   public Collection<PlanarRegionsList> getPlanarRegionsHistory()
   {
      return planarRegionsHistory.getPlanarRegions();
   }

   public PlanarRegionsList getReceivedPlanarRegions()
   {
      return planarRegions;
   }

   public void addCallback(Consumer<PlanarRegionsList> callback)
   {
      planarRegionsInput.addCallback(callback);
   }

   public void clear()
   {
      planarRegionsHistory.clear();
   }

   // time snapshot data
   private PlanarRegionsList planarRegions;
   private CapturabilityBasedStatus capturabilityBasedStatus;
   private RobotConfigurationData robotConfigurationData;

   private TimerSnapshotWithExpiration capturabilityBasedStatusReceptionTimerSnapshot;
   private TimerSnapshotWithExpiration robotConfigurationDataReceptionTimerSnapshot;

   public void updateSnapshot()
   {
      planarRegions = planarRegionsInput.getLatest();
      capturabilityBasedStatus = capturabilityBasedStatusInput.getLatest();
      robotConfigurationData = robotConfigurationDataInput.getLatest();

      capturabilityBasedStatusReceptionTimerSnapshot = capturabilityBasedStatusExpirationTimer.createSnapshot(lookAndStepParameters.getPlanarRegionsExpiration());
      robotConfigurationDataReceptionTimerSnapshot = robotConfigurationDataExpirationTimer.createSnapshot(lookAndStepParameters.getPlanarRegionsExpiration());
   }

   public String computeRegionsToPlanWith(SideDependentList<MinimalFootstep> startFootPoses)
   {
      FramePose3D sensorPose = new FramePose3D(syncedRobot.getReferenceFrames().getPelvisZUpFrame());
      sensorPose.changeFrame(ReferenceFrame.getWorldFrame());

      boolean isDataValid = capturabilityBasedStatusReceptionTimerSnapshot.isRunning() && robotConfigurationDataReceptionTimerSnapshot.isRunning();

      if (planarRegionsHistory.isEmpty() && lookAndStepParameters.getPlanarRegionsHistorySize() > 0 && lookAndStepParameters.getUseInitialSupportRegions()
          && isDataValid)
      {
         bipedalSupportPlanarRegionCalculator.calculateSupportRegions(lookAndStepParameters.getSupportRegionScaleFactor(),
                                                                      capturabilityBasedStatus,
                                                                      robotConfigurationData);
         planarRegionsHistory.addLast(sensorPose, bipedalSupportPlanarRegionCalculator.getSupportRegionsAsList(), minimumTranslationToAppend, minimumRotationToAppend);
      }

      String status;
      if (lookAndStepParameters.getAssumeFlatGround())
      {
         status = "Assuming Flat Ground.";
         planarRegionsHistory.addLast(sensorPose,
                                      constructFlatGroundCircleRegion(computeMidFeetPose(), lookAndStepParameters.getAssumedFlatGroundCircleRadius()),
                                      minimumTranslationToAppend,
                                      minimumRotationToAppend);

      }
      else if (lookAndStepParameters.getDetectFlatGround())
      {
         FramePose3DReadOnly midFeetPose = computeMidFeetPose();

         List<PlanarRegion> largeEnoughRegions = planarRegions.getPlanarRegionsAsList()
                                                              .stream()
                                                              .filter(region -> PlanarRegionTools.computePlanarRegionArea(region)
                                                                                > lookAndStepParameters.getDetectFlatGroundMinRegionAreaToConsider())
                                                              .collect(Collectors.toList());

         // are the feet coplanar
         if (!areFeetCoplanar(startFootPoses))
         {
            status = "Flat ground not detected.";
            planarRegionsHistory.addLast(sensorPose, planarRegions, minimumTranslationToAppend, minimumRotationToAppend);
         }
         else
         {
            // feet are coplanar, so just pick the left one.
            Point3DReadOnly leftFootPosition = startFootPoses.get(RobotSide.LEFT).getSolePoseInWorld().getPosition();
            Vector3DReadOnly footNormal = getFootNormal(startFootPoses.get(RobotSide.LEFT));

            List<PlanarRegion> largeNonCoplanarRegions = largeEnoughRegions.stream().filter(region ->
            {
               boolean areHeightsTheSame = EuclidCoreTools.epsilonEquals(leftFootPosition.getZ(),
                                                                         region.getPlaneZGivenXY(leftFootPosition.getX(), leftFootPosition.getY()),
                                                                         lookAndStepParameters.getDetectFlatGroundZTolerance());
               if (!areHeightsTheSame)
                  return true;

               return region.getNormal().angle(footNormal) > lookAndStepParameters.getDetectFlatGroundOrientationTolerance();
            }).collect(Collectors.toList());

            double closestNonCoplanarDistance = lookAndStepParameters.getAssumedFlatGroundCircleRadius();
            for (int i = 0; i < largeNonCoplanarRegions.size(); i++)
            {
               PlanarRegion planarRegion = largeNonCoplanarRegions.get(i);
               double distanceToRegionFromMidstance = planarRegion.distanceToPointByProjectionOntoXYPlane(midFeetPose.getPosition().getX(),
                                                                                                          midFeetPose.getPosition().getY());
               closestNonCoplanarDistance = Math.min(closestNonCoplanarDistance, distanceToRegionFromMidstance);
            }

            if (closestNonCoplanarDistance >= lookAndStepParameters.getDetectFlatGroundMinRadius())
            {
               status = "Flat ground detected.";
               planarRegionsHistory.addLast(sensorPose,
                                            constructFlatGroundCircleRegion(midFeetPose, closestNonCoplanarDistance),
                                            minimumTranslationToAppend,
                                            minimumRotationToAppend);
            }
            else
            {
               status = "Flat ground not detected.";
               planarRegionsHistory.addLast(sensorPose, planarRegions, minimumTranslationToAppend, minimumRotationToAppend);
            }
         }
      }
      else
      {
         status = "Not looking for flat ground.";
         planarRegionsHistory.addLast(sensorPose, planarRegions, minimumTranslationToAppend, minimumRotationToAppend);
      }

      return status;
   }

   public void dequeueToSize()
   {
      planarRegionsHistory.dequeueToSize(lookAndStepParameters.getPlanarRegionsHistorySize(), maxDistanceToRemember);
   }

   private static PlanarRegionsList constructFlatGroundCircleRegion(FramePose3DReadOnly midFeetPose, double radius)
   {
      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      midFeetPose.get(transformToWorld);
      ArrayList<ConvexPolygon2D> polygons = new ArrayList<>();
      ConvexPolygon2D convexPolygon2D = new ConvexPolygon2D();
      for (int i = 0; i < 40; i++)
      {
         double angle = (i / 40.0) * 2.0 * Math.PI;
         convexPolygon2D.addVertex(radius * Math.cos(angle), radius * Math.sin(angle));
      }
      convexPolygon2D.update();
      polygons.add(convexPolygon2D);
      PlanarRegion circleRegion = new PlanarRegion(transformToWorld, polygons);
      PlanarRegionsList planarRegionsList = new PlanarRegionsList();
      planarRegionsList.addPlanarRegion(circleRegion);
      return planarRegionsList;
   }

   private FramePose3DReadOnly computeMidFeetPose()
   {
      FramePose3D midFeetPose = new FramePose3D(syncedRobot.getReferenceFrames().getMidFootZUpGroundFrame());
      midFeetPose.changeFrame(ReferenceFrame.getWorldFrame());

      return midFeetPose;
   }

   private boolean areFeetCoplanar(SideDependentList<MinimalFootstep> startFootPoses)
   {
      double leftZ = startFootPoses.get(RobotSide.LEFT).getSolePoseInWorld().getPosition().getZ();
      double rightZ = startFootPoses.get(RobotSide.RIGHT).getSolePoseInWorld().getPosition().getZ();
      if (!EuclidCoreTools.epsilonEquals(leftZ, rightZ, lookAndStepParameters.getDetectFlatGroundZTolerance()))
         return false;

      Vector3DReadOnly leftZUp = getFootNormal(startFootPoses.get(RobotSide.LEFT));
      Vector3DReadOnly rightZUp = getFootNormal(startFootPoses.get(RobotSide.RIGHT));
      return leftZUp.angle(rightZUp) < lookAndStepParameters.getDetectFlatGroundOrientationTolerance();
   }

   private Vector3DReadOnly getFootNormal(MinimalFootstep foot)
   {
      QuaternionReadOnly orientation = foot.getSolePoseInWorld().getOrientation();
      Vector3D zUp = new Vector3D(Axis3D.Z);
      orientation.transform(zUp);
      return zUp;
   }

   private static class PlanarRegionsHistory
   {
      private final ArrayDeque<PlanarRegionsList> planarRegionsQueue = new ArrayDeque<>();
      private final ArrayDeque<FramePose3DReadOnly> sensorPoseQueue = new ArrayDeque<>();

      public boolean isEmpty()
      {
         return planarRegionsQueue.isEmpty();
      }

      public void removeFirst()
      {
         planarRegionsQueue.removeFirst();
         sensorPoseQueue.removeFirst();
      }

      public void addLast(FramePose3DReadOnly sensorPose, PlanarRegionsList planarRegionsList, double minimumTranslationToAppend,
                          double minimumRotationToAppend)
      {
         if (!isEmpty())
         {
            FramePose3DReadOnly lastSensorPose = sensorPoseQueue.getLast();

            boolean hasSensorMovedFarEnough = sensorPose.getPositionDistance(lastSensorPose) > minimumTranslationToAppend  ||
                                              sensorPose.getOrientationDistance(lastSensorPose) > minimumRotationToAppend;
            if (!hasSensorMovedFarEnough)
            {
               // remove the last ones, we're going to overwrite them
               planarRegionsQueue.removeLast();
               sensorPoseQueue.removeLast();
            }
         }


         planarRegionsQueue.add(planarRegionsList);
         sensorPoseQueue.add(sensorPose);
      }

      public void dequeueToSize(int size, double maxDistance)
      {
         while (sensorPoseQueue.getFirst().getPositionDistance(sensorPoseQueue.getLast()) > maxDistance)
         {
            sensorPoseQueue.removeFirst();
            planarRegionsQueue.removeFirst();
         }

         while (planarRegionsQueue.size() > size)
         {
            sensorPoseQueue.removeFirst();
            planarRegionsQueue.removeFirst();
         }
      }

      public void clear()
      {
         planarRegionsQueue.clear();
         sensorPoseQueue.clear();
      }

      public Collection<PlanarRegionsList> getPlanarRegions()
      {
         return planarRegionsQueue;
      }
   }
}
