package us.ihmc.humanoidBehaviors.exploreArea;

import us.ihmc.avatar.drcRobot.RemoteSyncedRobotModel;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidBehaviors.tools.BehaviorHelper;
import us.ihmc.humanoidBehaviors.tools.behaviorTree.*;
import us.ihmc.humanoidBehaviors.tools.interfaces.StatusLogger;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotEnvironmentAwareness.planarRegion.slam.PlanarRegionSLAM;
import us.ihmc.robotEnvironmentAwareness.planarRegion.slam.PlanarRegionSLAMParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.slam.PlanarRegionSLAMResult;
import us.ihmc.robotEnvironmentAwareness.tools.ConcaveHullMergerListenerAdapter;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.PlanarRegionFileTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.humanoidBehaviors.exploreArea.ExploreAreaBehavior.*;
import static us.ihmc.humanoidBehaviors.exploreArea.ExploreAreaBehaviorAPI.*;

public class ExploreAreaLookAroundNode extends SequenceNode
{
   public static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final ExploreAreaBehaviorParameters parameters;
   private final BehaviorHelper helper;
   private final RemoteSyncedRobotModel syncedRobot;
   private final StatusLogger statusLogger;

   private final AtomicReference<Boolean> hullGotLooped = new AtomicReference<>();
   private PlanarRegionsList latestPlanarRegionsList;
   private PlanarRegionsList concatenatedMap;
   private BoundingBox3D concatenatedMapBoundingBox;
   private final List<Point3D> pointsObservedFrom = new ArrayList<>();
   private final LookInADirection lookRight;
   private final LookInADirection lookCenter;
   private final LookInADirection lookLeft;

   public ExploreAreaLookAroundNode(ExploreAreaBehaviorParameters parameters,
                                    BehaviorHelper helper)
   {
      this.parameters = parameters;
      this.helper = helper;

      syncedRobot = helper.getOrCreateRobotInterface().newSyncedRobot();
      statusLogger = helper.getOrCreateStatusLogger();

      helper.createUICallback(DoSlam, this::doSlam);
      helper.createUICallback(ClearMap, this::clearMap);
      helper.createUICallback(RandomPoseUpdate, this::randomPoseUpdate);

      lookRight = new LookInADirection(-40.0, -20.0);
      lookCenter = new LookInADirection(0.0, 0.0);
      lookLeft = new LookInADirection(40.0, 20.0);

      addChild(lookRight);
      addChild(lookCenter);
      addChild(lookLeft);
   }

   public void reset()
   {
      lookRight.reset();
      lookCenter.reset();
      lookLeft.reset();
   }

   class LookInADirection extends ParallelNodeBasics
   {
      private final double chestYaw;
      private final double headPitch;

      public LookInADirection(double chestYaw, double headPitch)
      {
         this.chestYaw = Math.toRadians(chestYaw);
         this.headPitch = Math.toRadians(headPitch);
      }

      @Override
      public void doAction()
      {
         ExploreAreaBehaviorState currentState;
         if (chestYaw < 0.0)
         {
            currentState = ExploreAreaBehaviorState.LookRight;
         }
         else if (chestYaw == 0.0)
         {
            currentState = ExploreAreaBehaviorState.LookCenter;
         }
         else
         {
            currentState = ExploreAreaBehaviorState.LookLeft;
         }
         helper.publishToUI(CurrentState, currentState);

         turnChestWithRespectToMidFeetZUpFrame(chestYaw, parameters.getTurnChestTrajectoryDuration());
         pitchHeadWithRespectToChest(headPitch, parameters.getTurnChestTrajectoryDuration());

         double waitTime = parameters.getTurnTrajectoryWaitTimeMulitplier() * parameters.getTurnChestTrajectoryDuration();

         ThreadTools.sleepSeconds(waitTime);

         statusLogger.info("Entering perceive state. Clearing LIDAR");
         helper.getOrCreateREAInterface().clearREA();

         double perceiveDuration = parameters.getPerceiveDuration();
         statusLogger.info("Perceiving for {} s", perceiveDuration);
         ThreadTools.sleepSeconds(perceiveDuration);

         helper.publishToUI(ClearPlanarRegions);
         rememberObservationPoint();
         doSlam(true);
      }

      private void turnChestWithRespectToMidFeetZUpFrame(double chestYaw, double trajectoryTime)
      {
         syncedRobot.update();

         ReferenceFrame frame = syncedRobot.getReferenceFrames().getPelvisFrame();
         FrameQuaternion frameOrientation = new FrameQuaternion(frame, chestYaw, 0.0, 0.0);
         frameOrientation.changeFrame(worldFrame);
         helper.getOrCreateRobotInterface()
               .requestChestOrientationTrajectory(trajectoryTime,
                                                  frameOrientation,
                                                  worldFrame,
                                                  syncedRobot.getReferenceFrames().getPelvisZUpFrame());
         helper.getOrCreateRobotInterface().requestPelvisGoHome(trajectoryTime);
      }

      private void pitchHeadWithRespectToChest(double headPitch, double trajectoryTime)
      {
         syncedRobot.update();

         ReferenceFrame chestFrame = syncedRobot.getReferenceFrames().getChestFrame();
         FrameQuaternion headOrientation = new FrameQuaternion(chestFrame, 0.0, headPitch, 0.0);
         headOrientation.changeFrame(worldFrame);
         helper.getOrCreateRobotInterface()
               .requestHeadOrientationTrajectory(trajectoryTime,
                                                 headOrientation,
                                                 worldFrame,
                                                 syncedRobot.getReferenceFrames().getPelvisZUpFrame());
      }

      private void rotateChestAndPitchHeadToLookAtPointInWorld(double timeInState,
                                                               RobotSide swingSide,
                                                               Point3D pointToLookAtInWorld,
                                                               FullHumanoidRobotModel fullRobotModel,
                                                               HumanoidReferenceFrames referenceFrames)
      {
         //      ReferenceFrame headBaseFrame = fullRobotModel.getHeadBaseFrame();
         MovingReferenceFrame chestFrame = referenceFrames.getChestFrame();

         //      FramePoint3D footstepLocationInHeadFrame = new FramePoint3D(worldFrame, locationInWorld);
         //      footstepLocationInHeadFrame.changeFrame(headBaseFrame);

         FramePoint3D pointToLookAtInWorldFrame = new FramePoint3D(worldFrame, pointToLookAtInWorld);
         //      FramePoint3D supportFootLocationInWorld = new FramePoint3D(referenceFrames.getSoleFrame(swingSide.getOppositeSide()));
         //      supportFootLocationInWorld.changeFrame(worldFrame);

         //      FrameVector3D soleToPointToLookAtVector = new FrameVector3D(worldFrame);
         //      soleToPointToLookAtVector.sub(pointToLookAtInWorldFrame, supportFootLocationInWorld);

         FramePoint3D headLocationInWorld = new FramePoint3D(fullRobotModel.getHeadBaseFrame());
         headLocationInWorld.changeFrame(worldFrame);

         FrameVector3D headToPointToLookAtVector = new FrameVector3D(worldFrame);
         headToPointToLookAtVector.sub(pointToLookAtInWorldFrame, headLocationInWorld);

         double totalMotionTime = 1.0;
         double trajectoryTime = totalMotionTime - timeInState;
         if (trajectoryTime < 0.1)
            trajectoryTime = 0.1;

         //      MovingReferenceFrame neckFrame = referenceFrames.getNeckFrame(NeckJointName.PROXIMAL_NECK_PITCH);
         //      Vector2D footstepLocationXY = new Vector2D(footstepLocationInChestFrame);

         Vector2D headToFootstepFrameXY = new Vector2D(headToPointToLookAtVector);

         double yaw = Math.atan2(headToPointToLookAtVector.getY(), headToPointToLookAtVector.getX());
         double pitch = -Math.atan2(headToPointToLookAtVector.getZ(), headToFootstepFrameXY.length());

         //      LogTools.info("Turning to look at footstep. Yaw = {}, pitch = {}", yaw, pitch);

         FrameQuaternion chestOrientation = new FrameQuaternion(worldFrame);
         chestOrientation.setYawPitchRoll(yaw, 0.25 * pitch, 0.0);
         helper.getOrCreateRobotInterface().requestChestOrientationTrajectory(trajectoryTime, chestOrientation, worldFrame, worldFrame);

         //      double worldYaw = chestOrientation.getYaw();
         FrameQuaternion headOrientation = new FrameQuaternion(worldFrame);
         headOrientation.setYawPitchRoll(yaw, 0.75 * pitch, 0.0);
         helper.getOrCreateRobotInterface().requestHeadOrientationTrajectory(trajectoryTime, headOrientation, worldFrame, worldFrame);
      }
   }

   void rememberObservationPoint()
   {
      //TODO: Remember the LIDAR pointing at transform instead of just where the robot was at. But how to get that frame?
      syncedRobot.update();
      MovingReferenceFrame midFeetZUpFrame = syncedRobot.getReferenceFrames().getMidFeetZUpFrame();
      FramePoint3D midFeetLocation = new FramePoint3D(midFeetZUpFrame);
      midFeetLocation.changeFrame(worldFrame);

      helper.publishToUI(ObservationPosition, new Point3D(midFeetLocation));

      pointsObservedFrom.add(new Point3D(midFeetLocation));
   }

   private void doSlam(boolean doSlam)
   {
      latestPlanarRegionsList = helper.getOrCreateREAInterface().getLatestPlanarRegionsList();

      if (concatenatedMap == null)
      {
         concatenatedMap = latestPlanarRegionsList;
      }
      else
      {
         PlanarRegionSLAMParameters slamParameters = new PlanarRegionSLAMParameters();

         //TODO: Tune these and add parameter decay factor for each iteration.
         // Maybe have an array of values to use for each iteration.
         //TODO: Have SLAM return statistics, like residual error, to help determine the goodness of the data
         // and take action accordingly.
         slamParameters.setIterationsForMatching(5);
         slamParameters.setBoundingBoxHeight(0.05);
         slamParameters.setMinimumNormalDotProduct(0.99);
         slamParameters.setDampedLeastSquaresLambda(5.0);
         slamParameters.setMaximumPointProjectionDistance(0.10);

         hullGotLooped.set(false);
         syncedRobot.update();
         RigidBodyTransform referenceTransform = syncedRobot.getReferenceFrames().getIMUFrame().getTransformToWorldFrame();
         statusLogger.info("Doing SLAM with IMU reference Transform \n {} ", referenceTransform);

         PlanarRegionSLAMResult slamResult = PlanarRegionSLAM.slam(concatenatedMap,
                                                                   latestPlanarRegionsList,
                                                                   slamParameters,
                                                                   referenceTransform,
                                                                   createConcaveHullMergerListenerAdapter());

         concatenatedMap = slamResult.getMergedMap();
         RigidBodyTransform transformFromIncomingToMap = slamResult.getTransformFromIncomingToMap();
         statusLogger.info("SLAM transformFromIncomingToMap = \n {}", transformFromIncomingToMap);

         boolean sendingSlamCorrection = true;
         publishPoseUpdateForStateEstimator(transformFromIncomingToMap, sendingSlamCorrection);
      }

      computeMapBoundingBox3D();

      statusLogger.info("concatenatedMap has " + concatenatedMap.getNumberOfPlanarRegions() + " planar Regions");
      statusLogger.info("concatenatedMapBoundingBox = " + concatenatedMapBoundingBox);

      List<PlanarRegion> planarRegionsAsList = concatenatedMap.getPlanarRegionsAsList();

      int index = 0;
      for (PlanarRegion planarRegion : planarRegionsAsList)
      {
         helper.publishToUI(AddPlanarRegionToMap, TemporaryPlanarRegionMessage.convertToTemporaryPlanarRegionMessage(planarRegion, index));

         List<ConvexPolygon2D> convexPolygons = planarRegion.getConvexPolygons();
         for (ConvexPolygon2D polygon : convexPolygons)
         {
            helper.publishToUI(AddPolygonToPlanarRegion, TemporaryConvexPolygon2DMessage.convertToTemporaryConvexPolygon2DMessage(polygon, index));
         }

         index++;
      }

      helper.publishToUI(DrawMap);

      // Send it to the GUI for a viz...
      //         PlanarRegionsListMessage concatenatedMapMessage = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(concatenatedMap);
      //         helper.publishToUI(ExploreAreaBehavior.ConcatenatedMap, concatenatedMapMessage);

      // Find a point that has not been observed, but is close to a point that can be walked to, in order to observe it...
   }

   private ConcaveHullMergerListenerAdapter createConcaveHullMergerListenerAdapter()
   {
      return new ConcaveHullMergerListenerAdapter()
      {
         boolean savedOutTroublesomeRegions = false;

         @Override
         public void hullGotLooped(List<Point2D> hullOne, List<Point2D> hullTwo, List<Point2D> mergedVertices)
         {
            hullGotLooped.set(true);

            statusLogger.error("Hull got looped.");

            if (savedOutTroublesomeRegions)
               return;

            savedOutTroublesomeRegions = true;

            statusLogger.error("First occurance this run, so saving out PlanarRegions");

            try
            {
               Path planarRegionsPath = new File("Troublesome" + File.separator + "MapPlanarRegions").toPath();
               statusLogger.error("map planarRegionsPath = {}", planarRegionsPath);
               Files.createDirectories(planarRegionsPath);
               PlanarRegionFileTools.exportPlanarRegionData(planarRegionsPath, concatenatedMap.copy());

               planarRegionsPath = new File("Troublesome" + File.separator + "NewPlanarRegions").toPath();
               statusLogger.error("new planarRegionsPath = {}", planarRegionsPath);
               Files.createDirectories(planarRegionsPath);
               PlanarRegionFileTools.exportPlanarRegionData(planarRegionsPath, latestPlanarRegionsList.copy());
            }
            catch (IOException e)
            {
               e.printStackTrace();
            }

            // ThreadTools.sleepForever();
         }
      };
   }

   private void clearMap(boolean clearMap)
   {
      helper.publishToUI(ClearPlanarRegions);
      concatenatedMap = null;
   }

   private void computeMapBoundingBox3D()
   {
      List<PlanarRegion> mapPlanarRegions = concatenatedMap.getPlanarRegionsAsList();
      concatenatedMapBoundingBox = null;

      for (PlanarRegion planarRegion : mapPlanarRegions)
      {
         BoundingBox3D boundingBox = planarRegion.getBoundingBox3dInWorldCopy();
         if (concatenatedMapBoundingBox == null)
         {
            concatenatedMapBoundingBox = boundingBox;
         }
         else
         {
            concatenatedMapBoundingBox = BoundingBox3D.union(concatenatedMapBoundingBox, boundingBox);
         }
      }
   }

   private void randomPoseUpdate(boolean doRandomPoseUpdate)
   {
      if (doRandomPoseUpdate)
      {
         RigidBodyTransform transform = new RigidBodyTransform();

         //TODO: Make random or allow user to input update on gui.
         //         transform.setTranslation(0.01, -0.01, 0.01);
         //         transform.setTranslation(0.02, -0.02, 0.0);
         //         transform.setTranslation(0.02, -0.02, 0.02);
         transform.getRotation().setToYawOrientation(0.025);

         boolean sendingSlamCorrection = false;
         publishPoseUpdateForStateEstimator(transform, sendingSlamCorrection);
      }
   }

   private void publishPoseUpdateForStateEstimator(RigidBodyTransform transformFromIncomingToMap, boolean sendingSlamCorrection)
   {
      syncedRobot.update();

      FramePose3D framePose = new FramePose3D(syncedRobot.getReferenceFrames().getPelvisFrame());
      framePose.changeFrame(ReferenceFrame.getWorldFrame());
      Pose3D pose3D = new Pose3D(framePose);

      // TODO: Verify which transform or appendTransform to use...

      RigidBodyTransform transform = new RigidBodyTransform(transformFromIncomingToMap);

      if (sendingSlamCorrection)
      {
         pose3D.prependTransform(transform);
      }
      else
      {
         pose3D.appendTransform(transform);
      }

      double confidenceFactor = 1.0;
      helper.getOrCreateRobotInterface().publishPose(pose3D, confidenceFactor, syncedRobot.getTimestamp());
   }

   public PlanarRegionsList getConcatenatedMap()
   {
      return concatenatedMap;
   }

   public BoundingBox3D getConcatenatedMapBoundingBox()
   {
      return concatenatedMapBoundingBox;
   }

   public List<Point3D> getPointsObservedFrom()
   {
      return pointsObservedFrom;
   }
}
