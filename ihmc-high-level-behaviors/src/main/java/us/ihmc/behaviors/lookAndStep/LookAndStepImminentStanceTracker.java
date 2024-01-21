package us.ihmc.behaviors.lookAndStep;

import controller_msgs.msg.dds.CapturabilityBasedStatus;
import controller_msgs.msg.dds.FootstepStatusMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.behaviors.tools.MinimalFootstep;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.PlannedFootstep;
import us.ihmc.footstepPlanning.PlannedFootstepReadOnly;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.HashMap;
import java.util.List;

/**
 * Footsteps get "commanded" given to this class and sent to the controller simultaneously.
 * When any "started" or "completed" footstep status comes in, that's an imminent stance,
 * so we store it.
 * We track when the first step newly commanded is started and completed and provide that
 * status for the "wait for part of swing" method, so it can be robust.
 */
public class LookAndStepImminentStanceTracker
{
   private final HashMap<Long, PlannedFootstepReadOnly> footstepHistory = new HashMap<>();
   private PlannedFootstepReadOnly firstCommandedStep = null;
   private boolean firstCommandedStepHasStarted = false;
   private boolean firstCommandedStepHasCompleted = false;
   private final SideDependentList<PlannedFootstepReadOnly> imminentStanceFootsteps = new SideDependentList<>();
   private final ROS2SyncedRobotModel syncedRobot;
   private final TypedInput<CapturabilityBasedStatus> capturabilityBasedStatusInput = new TypedInput<>();
   private volatile RobotSide lastStartedRobotSide = null;

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
         PlannedFootstepReadOnly imminentFootstep = footstepHistory.get(footstepStatusMessage.getSequenceId());
         if (imminentFootstep != null)
         {
            imminentStanceFootsteps.put(robotSide, imminentFootstep);
         }

         if (FootstepStatus.fromByte(footstepStatusMessage.getFootstepStatus()) == FootstepStatus.STARTED)
         {
            lastStartedRobotSide = robotSide;

            if (firstCommandedStep != null && firstCommandedStep.getSequenceId() == footstepStatusMessage.getSequenceId())
            {
               firstCommandedStepHasStarted = true;
            }
         }
         else // received completed footstep status
         {
            if (firstCommandedStep != null && firstCommandedStep.getSequenceId() == footstepStatusMessage.getSequenceId())
            {
               firstCommandedStepHasCompleted = true;
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
         firstCommandedStepHasStarted = false;
         firstCommandedStepHasCompleted = false;
         firstCommandedStep = commandedFootstepPlan.getFootstep(0);

         for (int i = 0; i < commandedFootstepPlan.getNumberOfSteps(); i++)
         {
            PlannedFootstep footstep = commandedFootstepPlan.getFootstep(i);
            footstepHistory.put(footstep.getSequenceId(), footstep);
         }
      }
   }

   public SideDependentList<MinimalFootstep> calculateImminentStancePoses()
   {
      SideDependentList<MinimalFootstep> imminentStanceFeet = new SideDependentList<>();
      synchronized (this)
      {
         syncedRobot.update();
         CapturabilityBasedStatus capturabilityBasedStatus = capturabilityBasedStatusInput.getLatest();
         for (RobotSide side : RobotSide.values)
         {
            if (imminentStanceFootsteps.get(side) == null) // in the case we are just starting to walk and haven't sent a step for this foot yet
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
               FramePose3D solePose = new FramePose3D();
               PlannedFootstepReadOnly commandedStep = imminentStanceFootsteps.get(side);
               commandedStep.getFootstepPose(solePose);
               solePose.changeFrame(ReferenceFrame.getWorldFrame());
               MinimalFootstep imminentCommandedFootstep = new MinimalFootstep(side,
                                                                               solePose,
                                                                               commandedStep.getFoothold(),
                                                                               side.getPascalCaseName() + " Imminent Stance (Commanded)");
               imminentStanceFeet.set(side, imminentCommandedFootstep);
            }
         }
      }
      return imminentStanceFeet;
   }

   public boolean getFirstCommandedStepHasStarted()
   {
      return firstCommandedStepHasStarted;
   }

   public boolean getFirstCommandedStepHasCompleted()
   {
      return firstCommandedStepHasCompleted;
   }

   public RobotSide getLastStartedRobotSide()
   {
      return lastStartedRobotSide;
   }

   public void clear()
   {
      synchronized (this)
      {
         imminentStanceFootsteps.clear();
         lastStartedRobotSide = null;
         firstCommandedStepHasStarted = false;
         firstCommandedStepHasCompleted = false;
         firstCommandedStep = null;
      }
   }
}
