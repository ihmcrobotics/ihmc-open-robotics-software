package us.ihmc.behaviors.upDownExploration;

import java.util.Optional;
import java.util.Random;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.commons.thread.Notification;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.communication.RemoteREAInterface;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.waypoints.Waypoint;
import us.ihmc.behaviors.waypoints.WaypointManager;
import us.ihmc.footstepPlanning.FootstepPlannerOutput;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.commons.thread.TypedNotification;

/**
 * Keep track of state and manage the specific flow of exploration for the May 2019 demo.
 *
 * Flow is after reset, go up or down. if went down, turn 180. if went up, turn random.
 *
 */
public class UpDownExplorer
{
   private static final double RADIAL_BOUNDARY = 1.5;
   private static final double FACING_CENTER_ALLOWED_ERROR = Math.PI / 2.0;

   private final UpDownFlatAreaFinder upDownFlatAreaFinder;
   private TypedNotification<Optional<FramePose3D>> upDownSearchNotification = new TypedNotification<>();
   private final AtomicReference<Point3D> upDownCenter = null;

   private double accumulatedTurnAmount = 0.0;
   private RobotSide turnDirection = RobotSide.LEFT;

   private final BehaviorHelper behaviorHelper;
   private final RemoteREAInterface rea;

   private Random random = new Random(System.nanoTime());
   private FramePose3DReadOnly midFeetZUpPose;
   private FramePoint2D midFeetZUpXYProjectionTemp = new FramePoint2D();
   private Point2D upDownCenterXYProjectionTemp = new Point2D();

   enum UpDownState
   {
      TURNING,
      TRAVERSING
   }

   private UpDownState state = UpDownState.TRAVERSING;
   private Notification plannerFailedOnLastRun = new Notification();

   public UpDownExplorer(BehaviorHelper behaviorHelper, RemoteREAInterface rea)
   {
      this.behaviorHelper = behaviorHelper;
      upDownFlatAreaFinder = null;
//      upDownFlatAreaFinder = new UpDownFlatAreaFinder(behaviorHelper.getMessager());
      this.rea = rea;

//      behaviorHelper.subscribeViaCallback(UpDownExplorationEnabled, enabled -> { if (enabled) state = UpDownState.TRAVERSING; });
//      upDownCenter = behaviorHelper.subscribeViaReference(UpDownCenter, new Point3D(0.0, 0.0, 0.0));
   }

   public void onNavigateEntry(ROS2SyncedRobotModel syncedRobot)
   {
      // TODO this should plan only if

      FramePose3D midFeetZUpPose = new FramePose3D();
      midFeetZUpPose.setFromReferenceFrame(syncedRobot.getReferenceFrames().getMidFeetZUpFrame());

      state = decideNextAction(midFeetZUpPose, true);

      if (state == UpDownState.TRAVERSING)
      {
         boolean isCloseToCenter = isCloseToCenter(midFeetZUpPose);
         boolean requireHeightChange = isCloseToCenter;
         PlanarRegionsList latestPlanarRegionList = rea.getLatestPlanarRegionsList();
         
         upDownSearchNotification = upDownFlatAreaFinder.upOrDownOnAThread(syncedRobot.getReferenceFrames().getMidFeetZUpFrame(),
                                                                           latestPlanarRegionList,
                                                                           requireHeightChange);
      }
   }

   private UpDownState decideNextAction(FramePose3DReadOnly midFeetZUpPose, boolean calledFromNavigateEntry)
   {
      if (calledFromNavigateEntry)
      {
         if (plannerFailedOnLastRun.poll())
         {
            if (calledFromNavigateEntry)
               LogTools.warn("Planner failed. Turning...");
            return UpDownState.TURNING;
         }
      }

      boolean isCloseToCenter = isCloseToCenter(midFeetZUpPose);

      Vector3D robotToCenter = new Vector3D(upDownCenter.get());
      robotToCenter.sub(midFeetZUpPose.getPosition());
      double centerFacingYaw = Math.atan2(robotToCenter.getY(), robotToCenter.getX());
      if (calledFromNavigateEntry) LogTools.debug("centerFacingYaw: {}", centerFacingYaw);

      double robotYaw = midFeetZUpPose.getYaw();
      if (calledFromNavigateEntry) LogTools.debug("robotYaw: {}", robotYaw);

      double difference = AngleTools.computeAngleDifferenceMinusPiToPi(robotYaw, centerFacingYaw);
      if (calledFromNavigateEntry) LogTools.debug("difference: {}", difference);

      boolean isFacingCenter = Math.abs(difference) < FACING_CENTER_ALLOWED_ERROR;

      if (calledFromNavigateEntry) LogTools.warn("isCloseToCenter {} || isFacingCenter {}", isCloseToCenter, isFacingCenter);
      return isCloseToCenter || isFacingCenter ? UpDownState.TRAVERSING : UpDownState.TURNING;
   }

   public void poll()
   {
      if (state == UpDownState.TRAVERSING)
      {
         upDownSearchNotification.poll();
      }
   }

   public boolean shouldTransitionToPlan()
   {
      return state == UpDownState.TURNING || upDownSearchNotification.hasValue();
   }

   public void onPlanEntry(FramePose3DReadOnly midFeetZUpPose, WaypointManager waypointManager)
   {
      waypointManager.clearWaypoints();
      Waypoint newWaypoint = waypointManager.appendNewWaypoint();

      if (state == UpDownState.TRAVERSING) // going to what the updown found
      {
         if (upDownSearchNotification.read().isPresent()) // success
         {
            newWaypoint.getPose().set(upDownSearchNotification.read().get());
         }
         else
         {
            computeRandomTurn(midFeetZUpPose, newWaypoint);
         }
      }
      else // if (state == UpDownState.TURNING)
      {
         computeRandomTurn(midFeetZUpPose, newWaypoint);
      }

      waypointManager.publish();
      waypointManager.setNextFromIndex(0);
   }

   private void computeRandomTurn(FramePose3DReadOnly midFeetZUpPose, Waypoint newWaypoint)
   {
      newWaypoint.getPose().set(midFeetZUpPose);

      double randomTurn = 0.25 * Math.PI + random.nextDouble() * 0.25 * Math.PI;

      Vector3D robotToCenter = new Vector3D(upDownCenter.get());
      robotToCenter.sub(midFeetZUpPose.getPosition());
      double centerFacingYaw = Math.atan2(robotToCenter.getY(), robotToCenter.getX());

      double robotYaw = midFeetZUpPose.getYaw();
      LogTools.debug("robotYaw: {} centerFacingYaw: {} accumulatedTurnAmountBefore: {}" , robotYaw, centerFacingYaw, accumulatedTurnAmount);

      double robotYawFromCenter = AngleTools.computeAngleDifferenceMinusPiToPi(robotYaw, centerFacingYaw);

      if (Math.abs(robotYawFromCenter) > 3.0 * Math.PI / 4.0)
      {
         if (accumulatedTurnAmount > 0.0 * Math.PI)
         {
            randomTurn = -Math.abs(randomTurn);
         }
         else
         {
            randomTurn = Math.abs(randomTurn);
         }
      }
      else
      {
         double negativeYawResult = AngleTools.computeAngleDifferenceMinusPiToPi(robotYaw, randomTurn);
         double positiveYawResult = robotYaw + randomTurn;

         double closenessToCenterIfSubtractRandom = AngleTools.computeAngleDifferenceMinusPiToPi(centerFacingYaw, negativeYawResult);
         double closenessToCenterIfAdditionRandom = AngleTools.computeAngleDifferenceMinusPiToPi(centerFacingYaw, positiveYawResult);

         if (Math.abs(closenessToCenterIfSubtractRandom) < Math.abs(closenessToCenterIfAdditionRandom))
         {
            randomTurn = -randomTurn;
         }
      }

//      if (accumulatedTurnAmount >= 2.0 * Math.PI && randomTurn > 0.0)
//      {
//         randomTurn = -randomTurn;
//      }
//      else if (accumulatedTurnAmount <= -2.0 * Math.PI && randomTurn < 0.0)
//      {
//         randomTurn = -randomTurn;
//      }

      LogTools.debug("amountToTurn: {}", randomTurn);

      accumulatedTurnAmount += randomTurn;
      LogTools.debug("accumulatedTurnAmountAfter: {}", accumulatedTurnAmount);
      newWaypoint.getPose().appendYawRotation(randomTurn);
   }

   public void onPlanFinished(FootstepPlannerOutput result)
   {
      if (!result.getFootstepPlanningResult().validForExecution())
      {
         plannerFailedOnLastRun.set();
      }
   }

   private boolean isCloseToCenter(FramePose3DReadOnly midFeetZUpPose)
   {
      midFeetZUpXYProjectionTemp.set(midFeetZUpPose.getPosition());
      upDownCenterXYProjectionTemp.set(upDownCenter.get());
      return midFeetZUpXYProjectionTemp.distance(upDownCenterXYProjectionTemp) < RADIAL_BOUNDARY;
   }

   public void setMidFeetZUpPose(FramePose3DReadOnly midFeetZUpPose)
   {
      this.midFeetZUpPose = midFeetZUpPose;
   }

   public boolean shouldTransitionFromPerceive()
   {
      return decideNextAction(midFeetZUpPose, false) == UpDownState.TURNING;
   }

   public void abortPlanning()
   {
      upDownFlatAreaFinder.abort();
   }
}
