package us.ihmc.avatar.continuousLocomotion;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.avatar.networkProcessor.supportingPlanarRegionPublisher.BipedalSupportPlanarRegionCalculator;
import us.ihmc.behaviors.lookAndStep.LookAndStepBehaviorParametersReadOnly;
import us.ihmc.behaviors.tools.footstepPlanner.MinimalFootstep;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.communication.controllerAPI.ConcurrentMessageInputBuffer;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.PlanarRegionsListCommand;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotEnvironmentAwareness.planarRegion.slam.PlanarRegionSLAMParameters;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoEnum;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.List;
import java.util.stream.Collectors;

public class AvatarTerrainIdentifier
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final double minimumTranslationToAppend = 0.2;
   private final double minimumRotationToAppend = Math.toRadians(30);
   private final double maxDistanceToRemember = 2.0;
   private final boolean useSLAMToCombineRegions = true;

   private final HumanoidReferenceFrames referenceFrames;

   private final LookAndStepBehaviorParametersReadOnly lookAndStepParameters;
   private final BipedalSupportPlanarRegionCalculator bipedalSupportPlanarRegionCalculator;

   //   private final Timer planarRegionsExpirationTimer = new Timer();
   //   private final Timer capturabilityBasedStatusExpirationTimer = new Timer();
   //   private final Timer robotConfigurationDataExpirationTimer = new Timer();

   private final ConcurrentMessageInputBuffer messageListenerManager;
//   private final TypedInput<PlanarRegionsList> planarRegionsInput = new TypedInput<>();
//   private final TypedInput<CapturabilityBasedStatus> capturabilityBasedStatusInput = new TypedInput<>();
//   private final TypedInput<RobotConfigurationData> robotConfigurationDataInput = new TypedInput<>();

   private PlanarRegionsList planarRegions;
   private final CapturabilityBasedStatus capturabilityBasedStatus = new CapturabilityBasedStatus();
   private final RobotConfigurationData robotConfigurationData = new RobotConfigurationData();

   protected PlanarRegionsHistory planarRegionsHistory = new PlanarRegionsHistory();
   private final List<PlanarRegionsList> planarRegionsForPlanning = new ArrayList<>();

   private final PlanarRegionSLAMParameters parameters = new PlanarRegionSLAMParameters();
   private final FramePose3D sensorPose = new FramePose3D();

   private enum detectedTerrain{ROUGH, FLAT} //TODO rename
   private final YoEnum<detectedTerrain> yoDetectedTerrain;
   private String status;

   public AvatarTerrainIdentifier(FullHumanoidRobotModel robotModel,
                                  HumanoidReferenceFrames referenceFrames,
                                  ConcurrentMessageInputBuffer messageListenerManager,
                                  YoRegistry parentRegistry)
   {
      this.messageListenerManager = messageListenerManager;

      this.referenceFrames = referenceFrames;
      //lookAndStepParameters = robotModel.getLookAndStepParameters(); //TODO make separate parameter class
      bipedalSupportPlanarRegionCalculator = new BipedalSupportPlanarRegionCalculator(robotModel);

      yoDetectedTerrain = new YoEnum<>("detectedTerrain", registry, detectedTerrain.class, false);
      yoDetectedTerrain.set(detectedTerrain.ROUGH);
      yoDetectedTerrain.addListener(change -> System.out.println(status));

      parentRegistry.addChild(registry);
   }

   void acceptPlanarRegionsListCommand(PlanarRegionsListCommand planarRegionsListCommand)
   {
      PlanarRegionsList planarRegionsList = new PlanarRegionsList();

      for (int i = 0; i < planarRegionsListCommand.getPlanarRegions().size(); i++)
      {
         planarRegionsList.addPlanarRegion(new PlanarRegion());
         planarRegionsListCommand.getPlanarRegionCommand(i).getPlanarRegion(planarRegionsList.getPlanarRegion(i));
      }

      planarRegions = planarRegionsList;

      if (lookAndStepParameters.getAssumeFlatGround())
      {
         addPlanarRegionsToHistory(planarRegionsList);
         dequeueToSize();
      }
   }

   public Collection<PlanarRegionsList> getPlanarRegionsHistory()
   {
      return planarRegionsForPlanning;
   }

   public PlanarRegionsList getReceivedPlanarRegions()
   {
      return planarRegions;
   }

//   public void addCallback(Consumer<PlanarRegionsList> callback)
//   {
//      planarRegionsInput.addCallback(callback);
//   }

   public void clear()
   {
      planarRegionsHistory.clear();
   }

   private void consumeMessages()
   {
//      if (messageListenerManager.isNewMessageAvailable(PlanarRegionsListCommand.class))
//      {
//         acceptPlanarRegionsListCommand(messageListenerManager.pollNewestMessage(PlanarRegionsListCommand.class));
//      }
      if (messageListenerManager.isNewMessageAvailable(CapturabilityBasedStatus.class))
      {
         capturabilityBasedStatus.set(messageListenerManager.pollNewestMessage(CapturabilityBasedStatus.class));
         messageListenerManager.clearMessages(CapturabilityBasedStatus.class);
      }
      if (messageListenerManager.isNewMessageAvailable(RobotConfigurationData.class))
      {
         robotConfigurationData.set(messageListenerManager.pollNewestMessage(RobotConfigurationData.class));
      }
   }

   private boolean flatGroundDetected = false; //TODO move
   private RobotSide stanceSide = RobotSide.LEFT; //TODO move

   public boolean flatGroundDetected()
   {
      consumeMessages();


      List<PlanarRegionsList> regionsToUseFromHistory = new ArrayList<>();
      List<PlanarRegionsList> regionsCreatedLocally = new ArrayList<>();

      SideDependentList<MinimalFootstep> startFootPoses = calculateImminentStancePoses();

      if (planarRegionsHistory.isEmpty() && lookAndStepParameters.getPlanarRegionsHistorySize() > 0 && lookAndStepParameters.getUseInitialSupportRegions())
      {
         bipedalSupportPlanarRegionCalculator.calculateSupportRegions(lookAndStepParameters.getSupportRegionScaleFactor(),
                                                                      capturabilityBasedStatus,
                                                                      robotConfigurationData);
         regionsCreatedLocally.add(bipedalSupportPlanarRegionCalculator.getSupportRegionsAsList());
      }

      if (lookAndStepParameters.getAssumeFlatGround())
      {
         regionsCreatedLocally.add(constructFlatGroundCircleRegion(computeMidFeetPose(), lookAndStepParameters.getAssumedFlatGroundCircleRadius()));
         status = "Assuming Flat Ground.";
         flatGroundDetected = true;
      }
      else if (true)//lookAndStepParameters.getDetectFlatGround()) //TODO fix
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
         if (capturabilityBasedStatus.getLeftFootSupportPolygon3d().size() > 0.5 * capturabilityBasedStatus.getLeftFootSupportPolygon3d().getCurrentCapacity()
             && capturabilityBasedStatus.getLeftFootSupportPolygon3d().size() > 0.5 * capturabilityBasedStatus.getLeftFootSupportPolygon3d().getCurrentCapacity()
             && !areFeetCoplanar(startFootPoses))//toolbox.getFootContactState(RobotSide.LEFT).inContact() && toolbox.getFootContactState(RobotSide.RIGHT).inContact() && !areFeetCoplanar(startFootPoses))
         {
            status = "Flat ground not detected.";
            flatGroundDetected = false;
         }
         else if (toolbox.getFootContactState(RobotSide.LEFT).inContact() || toolbox.getFootContactState(RobotSide.RIGHT).inContact())
         {
            if (capturabilityBasedStatus.getLeftFootSupportPolygon3d().size() > capturabilityBasedStatus.getRightFootSupportPolygon3d().size() && toolbox.getFootContactState(RobotSide.LEFT).inContact())
               stanceSide = RobotSide.LEFT;

            else if (capturabilityBasedStatus.getLeftFootSupportPolygon3d().size() < capturabilityBasedStatus.getRightFootSupportPolygon3d().size() && toolbox.getFootContactState(RobotSide.RIGHT).inContact())
               stanceSide = RobotSide.RIGHT;

            Point3DReadOnly stanceFootPosition = startFootPoses.get(stanceSide).getSolePoseInWorld().getPosition();
            Vector3DReadOnly footNormal = getFootNormal(startFootPoses.get(stanceSide));


            List<PlanarRegion> largeNonCoplanarRegions = new ArrayList<>();
            List<PlanarRegion> largeCoplanarRegions = new ArrayList<>();
            for (PlanarRegion largeEnoughRegion : largeEnoughRegions)
            {
               boolean isAtFootHeight = EuclidCoreTools.epsilonEquals(stanceFootPosition.getZ(),
                                                                      largeEnoughRegion.getPlaneZGivenXY(stanceFootPosition.getX(), stanceFootPosition.getY()),
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

               if (closestNonCoplanarDistance >= 1.5)//lookAndStepParameters.getDetectFlatGroundMinRadius()) //TODO fix
               {
                  status = "Flat ground detected.";
                  regionsCreatedLocally.add(constructFlatGroundCircleRegion(midFeetPose, 0.9 * closestNonCoplanarDistance));
                  flatGroundDetected = true;
               }
               else
               {

                  status = "Flat ground not detected. Closest non-coplanar distance < min radius.";
                  flatGroundDetected = false;
               }
            }
            else
            {
               status = "Flat ground not detected. No large coplanar regions.";
               flatGroundDetected = false;
            }
         }
      }
      else
      {
         status = "Not looking for flat ground.";
         addPlanarRegionsToHistory(planarRegions);
         regionsToUseFromHistory.addAll(planarRegionsHistory.getPlanarRegions());
         flatGroundDetected = false;
      }

      //      if (useSLAMToCombineRegions)
      //      {
      //         planarRegionsForPlanning.clear();
      //         if (regionsToUseFromHistory.isEmpty())
      //         {
      //            planarRegionsForPlanning.addAll(regionsCreatedLocally);
      //         }
      //         else
      //         {
      //            PlanarRegionsList map = regionsToUseFromHistory.remove(0);
      //            while (!regionsToUseFromHistory.isEmpty())
      //            {
      //               map = PlanarRegionSLAM.slam(map, regionsToUseFromHistory.remove(0), parameters).getMergedMap();
      //            }
      //            while (!regionsCreatedLocally.isEmpty())
      //            {
      //               PlanarRegionsList regionsCreatedLocallyRemoved = regionsCreatedLocally.remove(0);
      //               regionsCreatedLocallyRemoved.removePlanarRegionsWithNaN();
      //               map = PlanarRegionSLAM.slam(map, regionsCreatedLocallyRemoved, parameters).getMergedMap();
      //            }
      //            planarRegionsForPlanning.add(map);
      //         }
      //      }
      //      else
      //      {
      //         planarRegionsForPlanning.clear();
      //         planarRegionsForPlanning.addAll(regionsToUseFromHistory);
      //         planarRegionsForPlanning.addAll(regionsCreatedLocally);
      //      }
      //
      //      for (PlanarRegionsList planarRegionsListForPlanning : planarRegionsForPlanning)
      //      {
      //         removeCloseRegionsToExcludeThoseFromTheBody(planarRegionsListForPlanning);
      //      }

      updateDetectedTerrainEnum(flatGroundDetected);
      return flatGroundDetected;
   }

   private void updateDetectedTerrainEnum(boolean flatGroundDetected)
   {
      yoDetectedTerrain.set(flatGroundDetected ? detectedTerrain.FLAT : detectedTerrain.ROUGH);
   }

   private SideDependentList<MinimalFootstep> calculateImminentStancePoses()
   {
      SideDependentList<MinimalFootstep> imminentStanceFeet = new SideDependentList<>();

      //      CapturabilityBasedStatus capturabilityBasedStatus = capturabilityBasedStatusInput.getLatest();
      for (RobotSide side : RobotSide.values)
      {
         FramePose3D solePose = new FramePose3D(referenceFrames.getSoleFrame(side));
         //solePose.set(statusMessage.actual_foot_position_in_world_, statusMessage.getActualFootOrientationInWorld());
         solePose.changeFrame(ReferenceFrame.getWorldFrame());
         List<? extends Point3D> rawPolygon = side == RobotSide.LEFT ? capturabilityBasedStatus.getLeftFootSupportPolygon3d() : capturabilityBasedStatus.getRightFootSupportPolygon3d();
         ConvexPolygon2D foothold = new ConvexPolygon2D();
         foothold.addVertices(Vertex3DSupplier.asVertex3DSupplier(rawPolygon));
         foothold.update();
         imminentStanceFeet.set(side, new MinimalFootstep(side, solePose, foothold,
                                                          "Flat Ground Detector " + side.getPascalCaseName() + " Imminent Stance (Prior)"));
      }

      return imminentStanceFeet;
   }

   private void removeCloseRegionsToExcludeThoseFromTheBody(PlanarRegionsList planarRegionsList)
   {
      // filter the planar regions from colliding with the body
      sensorPose.setToZero(referenceFrames.getSteppingCameraFrame());
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
      FramePose3D sensorPose = new FramePose3D(referenceFrames.getPelvisZUpFrame());
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
      FramePose3D midFeetPose = new FramePose3D(referenceFrames.getMidFootZUpGroundFrame());
      midFeetPose.changeFrame(ReferenceFrame.getWorldFrame());

      return midFeetPose;
   }

   private boolean areFeetCoplanar(SideDependentList<MinimalFootstep> startFootPoses)
   {
      double leftZ = startFootPoses.get(RobotSide.LEFT).getSolePoseInWorld().getPosition().getZ();
      double rightZ = startFootPoses.get(RobotSide.RIGHT).getSolePoseInWorld().getPosition().getZ();
      if (!EuclidCoreTools.epsilonEquals(leftZ, rightZ, lookAndStepParameters.getDetectFlatGroundZTolerance()))
         return false;
      //      else
      //         return true;

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

            List<FramePose3DReadOnly> posesToRemove = regionsFromSensors.keySet()
                                                                        .parallelStream()
                                                                        .filter(pose -> pose.getPositionDistance(sensorPose) < minimumTranslationToAppend
                                                                                        && pose.getOrientationDistance(sensorPose) < minimumRotationToAppend)
                                                                        .toList();

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
         List<FramePose3DReadOnly> samplesTooFarAway = regionsFromSensors.keySet()
                                                                         .stream()
                                                                         .filter(pose -> pose.getPositionDistance(lastPose) > maxDistance)
                                                                         .toList();
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
