package us.ihmc.behaviors.lookAndStep;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.FootstepStatusMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.behaviors.tools.footstepPlanner.MinimalFootstep;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
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

public class LookAndStepImminentStanceTracker
{
   private final ArrayDeque<PlannedFootstepReadOnly> commandedFootstepQueue = new ArrayDeque<>();
   private final ArrayDeque<PlannedFootstepReadOnly> previousCommandedFootstepQueue = new ArrayDeque<>();
   private final SideDependentList<PlannedFootstepReadOnly> commandedImminentStancePoses = new SideDependentList<>();
   private final ROS2SyncedRobotModel syncedRobot;
   private int stepsCompletedSinceCommanded = Integer.MAX_VALUE;
   private int previousStepsCompletedSinceCommanded = Integer.MAX_VALUE;
   private final TypedInput<CapturabilityBasedStatus> capturabilityBasedStatusInput = new TypedInput<>();

   public LookAndStepImminentStanceTracker(BehaviorHelper helper)
   {
      this.syncedRobot = helper.newSyncedRobot();
      helper.subscribeToControllerViaCallback(FootstepStatusMessage.class, this::acceptFootstepStatusMessage);
   }

   private void acceptFootstepStatusMessage(FootstepStatusMessage footstepStatusMessage)
   {
      synchronized (this)
      {
         if (FootstepStatus.fromByte(footstepStatusMessage.getFootstepStatus()) == FootstepStatus.COMPLETED)
         {
            Pose3D completedPose = new Pose3D(footstepStatusMessage.getDesiredFootPositionInWorld(),
                                              footstepStatusMessage.getDesiredFootOrientationInWorld());
            FramePose3D commandedPose = new FramePose3D();

            if (!commandedFootstepQueue.isEmpty())
            {
               commandedFootstepQueue.getFirst().getFootstepPose(commandedPose);
               commandedPose.changeFrame(ReferenceFrame.getWorldFrame());
               if (completedPose.getPosition().epsilonEquals(commandedPose.getPosition(), 0.03)) // they should be exactly the same
               {
                  LogTools.info("Commanded step {} completed. {}", stepsCompletedSinceCommanded,
                                RobotSide.fromByte(footstepStatusMessage.getRobotSide()).name());
                  ++stepsCompletedSinceCommanded;
                  commandedFootstepQueue.removeFirst();
                  if (!commandedFootstepQueue.isEmpty())
                  {
                     commandedImminentStancePoses.put(commandedFootstepQueue.getFirst().getRobotSide(), commandedFootstepQueue.getFirst());
                  }
               }
            }

            if (!previousCommandedFootstepQueue.isEmpty())
            {
               previousCommandedFootstepQueue.getFirst().getFootstepPose(commandedPose);
               commandedPose.changeFrame(ReferenceFrame.getWorldFrame());
               if (completedPose.getPosition().epsilonEquals(commandedPose.getPosition(), 0.03)) // they should be exactly the same
               {
                  LogTools.info("Previous commanded step {} completed. {}",
                                previousStepsCompletedSinceCommanded,
                                RobotSide.fromByte(footstepStatusMessage.getRobotSide()).name());
                  ++previousStepsCompletedSinceCommanded;
                  previousCommandedFootstepQueue.removeFirst();
               }
            }
         }
      }
   }

   public void acceptCapturabilityBasedStatus(CapturabilityBasedStatus capturabilityBasedStatus)
   {
      capturabilityBasedStatusInput.set(capturabilityBasedStatus);
   }

   public void addCommandedFootsteps(FootstepPlan commandedFootstepPlan)
   {
      synchronized (this)
      {
         commandedImminentStancePoses.put(commandedFootstepPlan.getFootstep(0).getRobotSide(), commandedFootstepPlan.getFootstep(0));

         previousCommandedFootstepQueue.clear();
         while (!commandedFootstepQueue.isEmpty())
         {
            previousCommandedFootstepQueue.addFirst(commandedFootstepQueue.pollLast());
         }
         for (int i = 0; i < commandedFootstepPlan.getNumberOfSteps(); i++)
         {
            commandedFootstepQueue.addLast(commandedFootstepPlan.getFootstep(i));
         }
         previousStepsCompletedSinceCommanded = stepsCompletedSinceCommanded;
         stepsCompletedSinceCommanded = 0;
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
            FramePose3D solePose = new FramePose3D();
            if (commandedImminentStancePoses.get(side) == null) // in the case we are just starting to walk and haven't sent a step for this foot yet
            {
               solePose.setFromReferenceFrame(syncedRobot.getReferenceFrames().getSoleFrame(side));
               solePose.changeFrame(ReferenceFrame.getWorldFrame());
               us.ihmc.idl.IDLSequence.Object<Point3D> rawPolygon =
                     side == RobotSide.LEFT ? capturabilityBasedStatus.getLeftFootSupportPolygon3d() : capturabilityBasedStatus.getRightFootSupportPolygon3d();
               ConvexPolygon2D foothold = new ConvexPolygon2D();
               for (Point3D vertex : rawPolygon)
               {
                  foothold.addVertex(vertex);
               }
               imminentStanceFeet.set(side, new MinimalFootstep(side, solePose, foothold, side.getPascalCaseName() + " Prior Stance"));
            }
            else
            {
               // TODO: We need to operate not on "eventual stance", but stance after this
               // currently executing step would finish. We want to be reactive on each step.
               commandedImminentStancePoses.get(side).getFootstepPose(solePose);
               solePose.changeFrame(ReferenceFrame.getWorldFrame());
               imminentStanceFeet.set(side,
                                      new MinimalFootstep(side,
                                                          solePose,
                                                          commandedImminentStancePoses.get(side).getFoothold(),
                                                          side.getPascalCaseName() + " Commanded Stance"));
            }
         }
      }
      return imminentStanceFeet;
   }

   public int getStepsCompletedSinceCommanded()
   {
      return stepsCompletedSinceCommanded;
   }

   public int getPreviousStepsCompletedSinceCommanded()
   {
      return previousStepsCompletedSinceCommanded;
   }

   public void clear()
   {
      synchronized (this)
      {
         commandedFootstepQueue.clear();
         commandedImminentStancePoses.clear();
      }
   }
}
