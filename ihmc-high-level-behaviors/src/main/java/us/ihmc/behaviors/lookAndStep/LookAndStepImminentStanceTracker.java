package us.ihmc.behaviors.lookAndStep;

import controller_msgs.msg.dds.FootstepStatusMessage;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
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
   private final SideDependentList<PlannedFootstepReadOnly> imminentStancePoses = new SideDependentList<>();
   private int stepsCompletedSinceCommanded = Integer.MAX_VALUE;
   private int previousStepsCompletedSinceCommanded = Integer.MAX_VALUE;

   public LookAndStepImminentStanceTracker(BehaviorHelper helper)
   {
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
                     imminentStancePoses.put(commandedFootstepQueue.getFirst().getRobotSide(), commandedFootstepQueue.getFirst());
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

   public void addCommandedFootsteps(FootstepPlan commandedFootstepPlan)
   {
      synchronized (this)
      {
         imminentStancePoses.put(commandedFootstepPlan.getFootstep(0).getRobotSide(), commandedFootstepPlan.getFootstep(0));

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

   public SideDependentList<PlannedFootstepReadOnly> getImminentStancePoses()
   {
      return imminentStancePoses;
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
         imminentStancePoses.clear();
      }
   }
}
