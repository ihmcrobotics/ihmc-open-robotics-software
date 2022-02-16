package us.ihmc.behaviors.lookAndStep;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.FootstepStatusMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.behaviors.tools.footstepPlanner.MinimalFootstep;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.PlannedFootstepReadOnly;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayDeque;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

public class LookAndStepImminentStanceTracker
{
   private final ArrayDeque<PlannedFootstepReadOnly> commandedFootstepQueue = new ArrayDeque<>();
   private final SideDependentList<PlannedFootstepReadOnly> commandedImminentStancePoses = new SideDependentList<>();
   private final ROS2SyncedRobotModel syncedRobot;
   private int stepsCompletedSinceCommanded = Integer.MAX_VALUE;
   private int stepsStartedSinceCommanded = -1;
//   private int previousStepsCompletedSinceCommanded = Integer.MAX_VALUE;
   private final TypedInput<CapturabilityBasedStatus> capturabilityBasedStatusInput = new TypedInput<>();

   private final AtomicReference<PlannedFootstepReadOnly> lastCompletedFootstep = new AtomicReference<>(null);
   private final AtomicReference<PlannedFootstepReadOnly> lastStartedFootstep = new AtomicReference<>(null);

   public LookAndStepImminentStanceTracker(BehaviorHelper helper)
   {
      this.syncedRobot = helper.newSyncedRobot();
      helper.subscribeToControllerViaCallback(FootstepStatusMessage.class, this::acceptFootstepStatusMessage);
   }

   private void acceptFootstepStatusMessage(FootstepStatusMessage footstepStatusMessage)
   {
      synchronized (this)
      {
         RobotSide robotSide = RobotSide.fromByte(footstepStatusMessage.getRobotSide());

         if (FootstepStatus.fromByte(footstepStatusMessage.getFootstepStatus()) == FootstepStatus.STARTED)
         {
            Pose3D completedPose = new Pose3D(footstepStatusMessage.getDesiredFootPositionInWorld(),
                                              footstepStatusMessage.getDesiredFootOrientationInWorld());
            FramePose3D commandedPose = new FramePose3D();

            lastStartedFootstep.set(new PlannedFootstepCopier(commandedImminentStancePoses.get(robotSide)));

            stepsStartedSinceCommanded++;

            if (!commandedFootstepQueue.isEmpty())
            {
               commandedFootstepQueue.getFirst().getFootstepPose(commandedPose);
               commandedPose.changeFrame(ReferenceFrame.getWorldFrame());
               if (completedPose.getPosition().epsilonEquals(commandedPose.getPosition(), 0.03)) // they should be exactly the same
               {
                  LogTools.info("Commanded step {} completed. {}", stepsCompletedSinceCommanded,
                                robotSide.name());

                  PlannedFootstepReadOnly stepStarted = commandedFootstepQueue.removeFirst();
                  commandedImminentStancePoses.put(stepStarted.getRobotSide(), stepStarted);
               }
            }
         }
         else
         {
            lastCompletedFootstep.set(new PlannedFootstepCopier(commandedImminentStancePoses.get(robotSide)));
            if (!commandedFootstepQueue.isEmpty())
            {
               commandedImminentStancePoses.put(commandedFootstepQueue.getFirst().getRobotSide(), commandedFootstepQueue.getFirst());
            }

            ++stepsCompletedSinceCommanded;
         }
      }
   }

   public void acceptCapturabilityBasedStatus(CapturabilityBasedStatus capturabilityBasedStatus)
   {
      capturabilityBasedStatusInput.set(capturabilityBasedStatus);
   }

   public void addCommandedFootsteps(FootstepPlan commandedFootstepPlan, ExecutionMode executionMode)
   {
      synchronized (this)
      {
         if (executionMode == ExecutionMode.OVERRIDE)
            commandedFootstepQueue.clear();

         for (int i = 0; i < commandedFootstepPlan.getNumberOfSteps(); i++)
         {
            commandedFootstepQueue.addLast(commandedFootstepPlan.getFootstep(i));
         }

         commandedImminentStancePoses.put(commandedFootstepQueue.getFirst().getRobotSide(), commandedFootstepQueue.getFirst());

         stepsCompletedSinceCommanded = 0;
         stepsStartedSinceCommanded = 0;
      }
   }

   public SideDependentList<MinimalFootstep> calculateImminentStancePoses()
   {
      SideDependentList<MinimalFootstep> imminentStanceFeet = new SideDependentList<>();
      syncedRobot.update();
      synchronized (this)
      {
         CapturabilityBasedStatus capturabilityBasedStatus = capturabilityBasedStatusInput.getLatest();
         for (RobotSide side : RobotSide.values)
         {
            if (commandedImminentStancePoses.get(side) == null) // in the case we are just starting to walk and haven't sent a step for this foot yet
            {
               FramePose3D solePose = new FramePose3D(syncedRobot.getReferenceFrames().getSoleFrame(side));
               solePose.changeFrame(ReferenceFrame.getWorldFrame());
               List<? extends Point3D> rawPolygon =
                     side == RobotSide.LEFT ? capturabilityBasedStatus.getLeftFootSupportPolygon3d() : capturabilityBasedStatus.getRightFootSupportPolygon3d();
               ConvexPolygon2D foothold = new ConvexPolygon2D();
               foothold.addVertices(Vertex3DSupplier.asVertex3DSupplier(rawPolygon));
               foothold.update();
               imminentStanceFeet.set(side, new MinimalFootstep(side, solePose, foothold,
                                                                "Look and Step " + side.getPascalCaseName() + " Imminent Stance (Prior)"));
            }
            else
            {
               // TODO: We need to operate not on "eventual stance", but stance after this. This should be settable.
               // currently executing step would finish. We want to be reactive on each step.
               FramePose3D stepPose = new FramePose3D();
               commandedImminentStancePoses.get(side).getFootstepPose(stepPose);
               stepPose.changeFrame(ReferenceFrame.getWorldFrame());
               imminentStanceFeet.set(side,
                                      new MinimalFootstep(side,
                                                          stepPose,
                                                          commandedImminentStancePoses.get(side).getFoothold(),
                                                          side.getPascalCaseName() + " Imminent Stance (Commanded)"));
            }
         }
      }
      return imminentStanceFeet;
   }

   public int getStepsCompletedSinceCommanded()
   {
      return stepsCompletedSinceCommanded;
   }

   public int getStepsStartedSinceCommanded()
   {
      return stepsStartedSinceCommanded;
   }

   public PlannedFootstepReadOnly getLastCompletedFootstep()
   {
      return lastCompletedFootstep.get();
   }

   public PlannedFootstepReadOnly getLastStartedFootstep()
   {
      return lastStartedFootstep.get();
   }

   public void clear()
   {
      synchronized (this)
      {
         commandedFootstepQueue.clear();
         commandedImminentStancePoses.clear();
         lastCompletedFootstep.set(null);
         lastStartedFootstep.set(null);
      }
   }

   private static class PlannedFootstepCopier implements PlannedFootstepReadOnly
   {
      private final RobotSide robotSide;
      private final FramePose3D footstepPose = new FramePose3D();
      private final ConvexPolygon2D convexPolygon2D;
      private final boolean hasFoothold;

      public PlannedFootstepCopier(PlannedFootstepReadOnly other)
      {
         robotSide = other.getRobotSide();
         other.getFootstepPose(footstepPose);
         convexPolygon2D = new ConvexPolygon2D(other.getFoothold());
         hasFoothold = other.hasFoothold();
      }

      public RobotSide getRobotSide()
      {
         return robotSide;
      }

      public void getFootstepPose(FramePose3D footstepPoseToPack)
      {
         footstepPoseToPack.set(footstepPose);
      }

      public ConvexPolygon2DReadOnly getFoothold()
      {
         return convexPolygon2D;
      }

      public boolean hasFoothold()
      {
         return hasFoothold;
      }
   }
}
