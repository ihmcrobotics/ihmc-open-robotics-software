package us.ihmc.humanoidBehaviors.exploreArea;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.WalkingStatusMessage;
import us.ihmc.avatar.drcRobot.RemoteSyncedRobotModel;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.footstepPlanning.FootstepDataMessageConverter;
import us.ihmc.footstepPlanning.FootstepPlannerOutput;
import us.ihmc.footstepPlanning.FootstepPlannerRequest;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogger;
import us.ihmc.humanoidBehaviors.tools.BehaviorHelper;
import us.ihmc.humanoidBehaviors.tools.behaviorTree.ParallelNodeBasics;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.geometry.AngleTools;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.humanoidBehaviors.exploreArea.ExploreAreaBehaviorAPI.CurrentState;

public class ExploreAreaTurnInPlace extends ParallelNodeBasics
{
   public static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final ExploreAreaBehaviorParameters parameters;
   private final BehaviorHelper helper;
   private final RemoteSyncedRobotModel syncedRobot;
   private final ExploreAreaLatticePlanner exploreAreaLatticePlanner;

   private final FootstepPlanningModule footstepPlanner;
   private final FootstepPlannerLogger footstepPlannerLogger;

   private final AtomicReference<Point3D> userRequestedPointToLookAt = new AtomicReference<>();

   public ExploreAreaTurnInPlace(double expectedTickPeriod,
                                 ExploreAreaBehaviorParameters parameters,
                                 BehaviorHelper helper,
                                 ExploreAreaLatticePlanner exploreAreaLatticePlanner)
   {
      this.parameters = parameters;
      this.helper = helper;
      this.exploreAreaLatticePlanner = exploreAreaLatticePlanner;
      this.footstepPlanner = FootstepPlanningModuleLauncher.createModule(helper.getRobotModel());
      this.footstepPlannerLogger = new FootstepPlannerLogger(footstepPlanner);

      helper.getManagedMessager().registerTopicListener(ExploreAreaBehaviorAPI.UserRequestedPointToLookAt, userRequestedPointToLookAt::set);
      syncedRobot = helper.getOrCreateRobotInterface().newSyncedRobot();
   }

   @Override
   public void doAction()
   {
      helper.publishToUI(CurrentState, ExploreAreaBehavior.ExploreAreaBehaviorState.TurnInPlace);

      Point2D pointToLookAt2D = new Point2D();
      if (userRequestedPointToLookAt.get() != null)
      {
         Point3D pointToLookAt = userRequestedPointToLookAt.getAndSet(null);
         pointToLookAt2D.set(pointToLookAt.getX(), pointToLookAt.getY());
      }
      else
      {
         LatticeCell nearestHole = findNearestHole();
         double holeX = ExploredAreaLattice.toDouble(nearestHole.getX());
         double holeY = ExploredAreaLattice.toDouble(nearestHole.getY());
         pointToLookAt2D.setX(holeX);
         pointToLookAt2D.setY(holeY);
      }

      syncedRobot.update();
      FramePose3D pelvisFrame = new FramePose3D(syncedRobot.getReferenceFrames().getPelvisZUpFrame());
      pelvisFrame.changeFrame(ReferenceFrame.getWorldFrame());

      double robotX = pelvisFrame.getX();
      double robotY = pelvisFrame.getY();
      double robotYaw = pelvisFrame.getYaw();

      double heading = Math.atan2(pointToLookAt2D.getX() - robotY, pointToLookAt2D.getY() - robotX);
      double turnYaw = AngleTools.computeAngleDifferenceMinusPiToPi(heading, robotYaw);
      helper.getManagedMessager().submitMessage(ExploreAreaBehaviorAPI.EnvironmentGapToLookAt, pointToLookAt2D);

      turnInPlace(turnYaw);
      ThreadTools.sleepSeconds(3.0);
   }

   private LatticeCell findNearestHole()
   {
      ExploredAreaLattice.CellStatus[][] lattice = exploreAreaLatticePlanner.getExploredAreaLattice().getLattice();

      syncedRobot.update();
      Vector3DBasics pelvisTranslation = syncedRobot.getReferenceFrames().getPelvisZUpFrame().getTransformToWorldFrame().getTranslation();
      LatticeCell robotCell = new LatticeCell(pelvisTranslation.getX(), pelvisTranslation.getY());

      double minDistanceToHole = Double.MAX_VALUE;
      int minX = exploreAreaLatticePlanner.getExploredAreaLattice().getMinX();
      int minY = exploreAreaLatticePlanner.getExploredAreaLattice().getMinY();

      int minIndexX = -1;
      int minIndexY = -1;

      double minThreshold = MathTools.square(0.8 / ExploredAreaLattice.cellWidth);

      for (int i = 0; i < lattice.length; i++)
      {
         for (int j = 0; j < lattice[0].length; j++)
         {
            if (lattice[i][j] != null)
               continue;

            double distanceSquared = new LatticeCell(i + minX, j + minY).distanceSquared(robotCell);
            if (distanceSquared < minDistanceToHole && distanceSquared > minThreshold)
            {
               minDistanceToHole = distanceSquared;
               minIndexX = i;
               minIndexY = j;
            }
         }
      }

      return new LatticeCell(minIndexX + minX, minIndexY + minY);
   }

   public void turnInPlace(double yaw)
   {
      FootstepPlannerRequest request = new FootstepPlannerRequest();
      request.setAssumeFlatGround(false);

      syncedRobot.update();
      for (RobotSide robotSide : RobotSide.values)
      {
         FramePose3D solePose = new FramePose3D(syncedRobot.getReferenceFrames().getSoleFrame(robotSide));
         solePose.changeFrame(ReferenceFrame.getWorldFrame());
         request.setStartFootPose(robotSide, solePose);
      }

      FramePose3D goalPose = new FramePose3D(syncedRobot.getReferenceFrames().getMidFeetZUpFrame());
      goalPose.changeFrame(ReferenceFrame.getWorldFrame());
      goalPose.getOrientation().appendYawRotation(yaw);
      goalPose.getPosition().addX(1e-4);
      request.setGoalFootPoses(footstepPlanner.getFootstepPlannerParameters().getIdealFootstepWidth(), goalPose);

      request.setPlanBodyPath(false);
      request.setSnapGoalSteps(false);
      FootstepPlannerOutput output = footstepPlanner.handleRequest(request);

      FootstepDataListMessage footstepDataListMessage = FootstepDataMessageConverter.createFootstepDataListFromPlan(output.getFootstepPlan(), -1.0, -1.0);
      TypedNotification<WalkingStatusMessage> walkingCompleted = helper.getOrCreateRobotInterface().requestWalk(footstepDataListMessage);
      walkingCompleted.blockingPoll();

      footstepPlannerLogger.logSession();
   }

}
