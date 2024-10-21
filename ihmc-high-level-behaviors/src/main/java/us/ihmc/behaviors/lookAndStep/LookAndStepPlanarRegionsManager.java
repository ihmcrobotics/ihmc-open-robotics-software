package us.ihmc.behaviors.lookAndStep;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.networkProcessor.supportingPlanarRegionPublisher.BipedalSupportPlanarRegionCalculator;
import us.ihmc.behaviors.tools.MinimalFootstep;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.robotEnvironmentAwareness.planarRegion.slam.PlanarRegionSLAM;
import us.ihmc.robotEnvironmentAwareness.planarRegion.slam.PlanarRegionSLAMParameters;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.Timer;
import us.ihmc.tools.TimerSnapshotWithExpiration;

import java.util.*;
import java.util.function.Consumer;
import java.util.stream.Collectors;

public class LookAndStepPlanarRegionsManager
{
   private final double minimumTranslationToAppend = 0.2;
   private final double minimumRotationToAppend = Math.toRadians(30);
   private final double maxDistanceToRemember = 2.0;
   private final boolean useSLAMToCombineRegions = true;

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
   private final List<PlanarRegionsList> planarRegionsForPlanning = new ArrayList<>();

   private final PlanarRegionSLAM planarRegionSLAM = new PlanarRegionSLAM();
   private final PlanarRegionSLAMParameters parameters = new PlanarRegionSLAMParameters();
   private final FramePose3D sensorPose = new FramePose3D();

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

      if (lookAndStepParameters.getAssumeFlatGround())
      {
         addPlanarRegionsToHistory(planarRegionsList);
         dequeueToSize();
      }
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
      return planarRegionsForPlanning;
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
      boolean isDataValid = capturabilityBasedStatusReceptionTimerSnapshot.isRunning() && robotConfigurationDataReceptionTimerSnapshot.isRunning();

      List<PlanarRegionsList> regionsToUseFromHistory = new ArrayList<>();
      List<PlanarRegionsList> regionsCreatedLocally = new ArrayList<>();

      if (planarRegionsHistory.isEmpty() && lookAndStepParameters.getPlanarRegionsHistorySize() > 0 && lookAndStepParameters.getUseInitialSupportRegions()
          && isDataValid)
      {
         bipedalSupportPlanarRegionCalculator.calculateSupportRegions(lookAndStepParameters.getSupportRegionScaleFactor(),
                                                                      capturabilityBasedStatus,
                                                                      robotConfigurationData);
         regionsCreatedLocally.add(bipedalSupportPlanarRegionCalculator.getSupportRegionsAsList());
      }

      String status;
      if (lookAndStepParameters.getAssumeFlatGround())
      {
         status = "Assuming Flat Ground.";
         regionsCreatedLocally.add(constructFlatGroundCircleRegion(computeMidFeetPose(), lookAndStepParameters.getAssumedFlatGroundCircleRadius()));
      }
      else if (lookAndStepParameters.getDetectFlatGround())
      {
         addPlanarRegionsToHistory(planarRegions);
         regionsToUseFromHistory.addAll(planarRegionsHistory.getPlanarRegions());

         FramePose3DReadOnly midFeetPose = computeMidFeetPose();

         List<PlanarRegion> largeEnoughRegions = new ArrayList<>();
         for (PlanarRegionsList regions : regionsToUseFromHistory)
         {
            largeEnoughRegions.addAll(regions.getPlanarRegionsAsList()
                                             .stream()
                                             .filter(region -> PlanarRegionTools.computePlanarRegionArea(region)
                                                               > lookAndStepParameters.getDetectFlatGroundMinRegionAreaToConsider())
                                             .toList());
         }

         // are the feet coplanar
         if (!areFeetCoplanar(startFootPoses))
         {
            status = "Flat ground not detected.";
         }
         else
         {
            // feet are coplanar, so just pick the left one.
            Point3DReadOnly leftFootPosition = startFootPoses.get(RobotSide.LEFT).getSolePoseInWorld().getPosition();
            Vector3DReadOnly footNormal = getFootNormal(startFootPoses.get(RobotSide.LEFT));

            List<PlanarRegion> largeNonCoplanarRegions = new ArrayList<>();
            List<PlanarRegion> largeCoplanarRegions = new ArrayList<>();
            for (PlanarRegion largeEnoughRegion : largeEnoughRegions)
            {
               boolean isAtFootHeight = EuclidCoreTools.epsilonEquals(leftFootPosition.getZ(),
                                                                         largeEnoughRegion.getPlaneZGivenXY(leftFootPosition.getX(), leftFootPosition.getY()),
                                                                         lookAndStepParameters.getDetectFlatGroundZTolerance());
               UnitVector3DReadOnly largeEnoughRegionNormal = largeEnoughRegion.getNormal();
               double angleDifference = largeEnoughRegionNormal.angle(footNormal);
               boolean isParallelWithFootPlane = angleDifference < lookAndStepParameters.getDetectFlatGroundOrientationTolerance();

               boolean isCoplanar = isAtFootHeight && isParallelWithFootPlane;

               if (isCoplanar)
               {
                  largeCoplanarRegions.add(largeEnoughRegion);
               }
               else
               {
                  largeNonCoplanarRegions.add(largeEnoughRegion);
               }
            }

            if (!largeCoplanarRegions.isEmpty())
            {
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
                  regionsCreatedLocally.add(constructFlatGroundCircleRegion(midFeetPose, 0.9 * closestNonCoplanarDistance));
               }
               else
               {
                  status = "Flat ground not detected. Closest non-coplanar distance < min radius.";
               }
            }
            else
            {
               status = "Flat ground not detected. No large coplanar regions.";
            }
         }
      }
      else
      {
         status = "Not looking for flat ground.";
         addPlanarRegionsToHistory(planarRegions);
         regionsToUseFromHistory.addAll(planarRegionsHistory.getPlanarRegions());
      }

      if (useSLAMToCombineRegions)
      {
         planarRegionsForPlanning.clear();
         if (regionsToUseFromHistory.isEmpty())
         {
            planarRegionsForPlanning.addAll(regionsCreatedLocally);
         }
         else
         {
            PlanarRegionsList map = regionsToUseFromHistory.remove(0);
            while (!regionsToUseFromHistory.isEmpty())
            {
               map = PlanarRegionSLAM.slam(map, regionsToUseFromHistory.remove(0), parameters).getMergedMap();
            }
            while (!regionsCreatedLocally.isEmpty())
            {
               PlanarRegionsList regionsCreatedLocallyRemoved = regionsCreatedLocally.remove(0);
               regionsCreatedLocallyRemoved.removePlanarRegionsWithNaN();
               map = PlanarRegionSLAM.slam(map, regionsCreatedLocallyRemoved, parameters).getMergedMap();
            }
            planarRegionsForPlanning.add(map);
         }
      }
      else
      {
         planarRegionsForPlanning.clear();
         planarRegionsForPlanning.addAll(regionsToUseFromHistory);
         planarRegionsForPlanning.addAll(regionsCreatedLocally);
      }

      for (PlanarRegionsList planarRegionsListForPlanning : planarRegionsForPlanning)
      {
         removeCloseRegionsToExcludeThoseFromTheBody(planarRegionsListForPlanning);
      }

      return status;
   }

   private void removeCloseRegionsToExcludeThoseFromTheBody(PlanarRegionsList planarRegionsList)
   {
      // filter the planar regions from colliding with the body
      sensorPose.setToZero(syncedRobot.getReferenceFrames().getSteppingCameraFrame());
      sensorPose.changeFrame(ReferenceFrame.getWorldFrame());

      for (PlanarRegion planarRegion : planarRegionsList.getPlanarRegionsAsList())
      {
         double distance = PlanarRegionTools.distanceToPlanarRegion(sensorPose.getPosition(), planarRegion);
         if (distance < 0.6)
         {
            planarRegionsList.queuePlanarRegionForRemoval(planarRegion);
         }
      }

      planarRegionsList.removeQueuedPlanarRegions();
   }

   private void addPlanarRegionsToHistory(PlanarRegionsList planarRegions)
   {
      FramePose3D sensorPose = new FramePose3D(syncedRobot.getReferenceFrames().getPelvisZUpFrame());
      sensorPose.changeFrame(ReferenceFrame.getWorldFrame());
      planarRegionsHistory.addLast(sensorPose, planarRegions, minimumTranslationToAppend, minimumRotationToAppend);
      dequeueToSize();
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
      private final List<FramePose3DReadOnly> sensorList = new ArrayList<>();
      private final HashMap<FramePose3DReadOnly, PlanarRegionsList> regionsFromSensors = new HashMap<>();

      private final FramePose3D lastPose = new FramePose3D();

      public boolean isEmpty()
      {
         return regionsFromSensors.isEmpty();
      }

      public void addLast(FramePose3DReadOnly sensorPose,
                          PlanarRegionsList planarRegionsList,
                          double minimumTranslationToAppend,
                          double minimumRotationToAppend)
      {
         lastPose.set(sensorPose);

         if (!isEmpty())
         {

            List<FramePose3DReadOnly> posesToRemove = regionsFromSensors.keySet().parallelStream()
                                                       .filter(pose -> pose.getPositionDistance(sensorPose) < minimumTranslationToAppend
                                                                          && pose.getOrientationDistance(sensorPose) < minimumRotationToAppend).collect(
                        Collectors.toList());

            if (!posesToRemove.isEmpty())
            {
               for (int i = 0; i < posesToRemove.size(); i++)
               {
                  regionsFromSensors.remove(posesToRemove.get(i));
                  sensorList.remove(posesToRemove.get(i));
               }
            }
         }

         sensorList.add(sensorPose);
         regionsFromSensors.put(sensorPose, planarRegionsList);
      }

      public void dequeueToSize(int size, double maxDistance)
      {
         List<FramePose3DReadOnly> samplesTooFarAway = regionsFromSensors.keySet().stream().filter(pose -> pose.getPositionDistance(lastPose) > maxDistance).collect(
               Collectors.toList());
         samplesTooFarAway.forEach(regionsFromSensors::remove);
         samplesTooFarAway.forEach(sensorList::remove);

         List<FramePose3DReadOnly> sortedRegions = regionsFromSensors.keySet().stream().sorted((poseA, poseB) ->
                                                     {
                                                        if (poseA.getPositionDistance(lastPose) < poseB.getPositionDistance(lastPose))
                                                           return -1;
                                                        else
                                                           return 1;
                                                     }).collect(Collectors.toList());

         int numberOfRegions = sortedRegions.size();
         while (numberOfRegions > size)
         {
            FramePose3DReadOnly removedPose = sortedRegions.remove(numberOfRegions - 1);
            sensorList.remove(removedPose);
            regionsFromSensors.remove(removedPose);
            numberOfRegions--;
         }
      }

      public void clear()
      {
         sensorList.clear();
         regionsFromSensors.clear();
      }

      public Collection<PlanarRegionsList> getPlanarRegions()
      {
         return regionsFromSensors.values();
      }

      public List<PlanarRegionsList> getOrderedRegionList()
      {
         List<PlanarRegionsList> regions = new ArrayList<>();
         sensorList.forEach(sensor -> regions.add(regionsFromSensors.get(sensor)));
         return regions;
      }
   }
}
