package us.ihmc.commonWalkingControlModules.dynamicPlanning.lipm;

import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;

public class BasicCoPPlanner
{
   private final static int numberOfStepsToCompute = 3;
   private final static double finalTransferDuration = 1.0;

   private final static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final ArrayList<Footstep> upcomingFootsteps = new ArrayList<>();
   private final ArrayList<FootstepTiming> upcomingTimings = new ArrayList<>();

   private final SideDependentList<? extends ContactablePlaneBody> contactableFeet;

   private final FramePoint3D desiredCoPPosition = new FramePoint3D();
   private final FrameVector3D desiredCoPVelocity = new FrameVector3D();
   private final FrameVector3D desiredCoPAcceleration = new FrameVector3D();

   private final MultipleWaypointsPositionTrajectoryGenerator copTrajectory;

   private boolean isInTransfer = false;
   private boolean isInStanding = true;

   private double initialTime = 0.0;
   private final YoDouble timeInCurrentStateRemaining;

   public BasicCoPPlanner(SideDependentList<? extends ContactablePlaneBody> contactableFeet, YoRegistry registry)
   {
      this.contactableFeet = contactableFeet;

      timeInCurrentStateRemaining = new YoDouble("timeInCurrentStateRemaining", registry);

      desiredCoPPosition.setToZero();
      copTrajectory = new MultipleWaypointsPositionTrajectoryGenerator("copTrajectory", worldFrame, registry);
   }

   public void clearPlan()
   {
      upcomingFootsteps.clear();
      upcomingTimings.clear();
      copTrajectory.clear();
   }

   public void submitFootstep(Footstep footstep, FootstepTiming timing)
   {
      upcomingFootsteps.add(footstep);
      upcomingTimings.add(timing);
   }

   private final FramePoint3D initialSegmentCoP = new FramePoint3D();
   private final FramePoint3D finalSegmentCoP = new FramePoint3D();
   private final FramePoint3D footstepPosition = new FramePoint3D();

   public void initializeForStanding()
   {
      isInStanding = true;
      isInTransfer = false;
   }

   private final FrameVector3D zeroVelocity = new FrameVector3D();

   public void initializeForTransfer(double currentTime)
   {
      initialTime = currentTime;
      isInStanding = false;
      isInTransfer = true;

      Footstep currentFootstep = upcomingFootsteps.get(0);
      RobotSide supportSide = currentFootstep.getRobotSide().getOppositeSide();
      finalSegmentCoP.setToZero(contactableFeet.get(supportSide).getSoleFrame());

      desiredCoPPosition.changeFrame(worldFrame);
      finalSegmentCoP.changeFrame(worldFrame);

      double endTime = upcomingTimings.get(0).getTransferTime();
      copTrajectory.appendWaypoint(0.0, desiredCoPPosition, zeroVelocity);
      copTrajectory.appendWaypoint(endTime, finalSegmentCoP, zeroVelocity);

      endTime += upcomingTimings.get(0).getSwingTime();

      copTrajectory.appendWaypoint(endTime, finalSegmentCoP, zeroVelocity);

      Footstep previousFootstep = currentFootstep;

      int stepIndex;
      for (stepIndex = 1; stepIndex < Math.min(numberOfStepsToCompute, upcomingFootsteps.size()); stepIndex++)
      {
         endTime += upcomingTimings.get(stepIndex).getTransferTime();
         initialSegmentCoP.set(finalSegmentCoP);
         finalSegmentCoP.setToZero(previousFootstep.getSoleReferenceFrame());
         finalSegmentCoP.changeFrame(worldFrame);

         copTrajectory.appendWaypoint(endTime, finalSegmentCoP, zeroVelocity);

         endTime += upcomingTimings.get(stepIndex).getSwingTime();

         copTrajectory.appendWaypoint(endTime, finalSegmentCoP, zeroVelocity);

         previousFootstep = upcomingFootsteps.get(stepIndex);
      }

      upcomingFootsteps.get(stepIndex - 1).getPosition(footstepPosition);

      endTime += finalTransferDuration;

      initialSegmentCoP.set(finalSegmentCoP);
      finalSegmentCoP.interpolate(initialSegmentCoP, footstepPosition, 0.5);

      copTrajectory.appendWaypoint(endTime, finalSegmentCoP, zeroVelocity);
      copTrajectory.initialize();
   }

   public void initializeForSingleSupport(double currentTime)
   {
      initialTime = currentTime;
      isInStanding = false;
      isInTransfer = false;

      Footstep currentFootstep = upcomingFootsteps.get(0);
      RobotSide supportSide = currentFootstep.getRobotSide().getOppositeSide();
      finalSegmentCoP.setToZero(contactableFeet.get(supportSide).getSoleFrame());

      desiredCoPPosition.changeFrame(worldFrame);
      finalSegmentCoP.changeFrame(worldFrame);

      double endTime = upcomingTimings.get(0).getSwingTime();

      copTrajectory.appendWaypoint(0.0, desiredCoPPosition, zeroVelocity);
      copTrajectory.appendWaypoint(endTime, finalSegmentCoP, zeroVelocity);

      Footstep previousFootstep = currentFootstep;

      int stepIndex;
      for (stepIndex = 1; stepIndex < Math.min(numberOfStepsToCompute, upcomingFootsteps.size()); stepIndex++)
      {
         endTime += upcomingTimings.get(stepIndex).getTransferTime();
         initialSegmentCoP.set(finalSegmentCoP);
         finalSegmentCoP.setToZero(previousFootstep.getSoleReferenceFrame());
         finalSegmentCoP.changeFrame(worldFrame);

         copTrajectory.appendWaypoint(endTime, finalSegmentCoP, zeroVelocity);

         endTime += upcomingTimings.get(stepIndex).getSwingTime();

         copTrajectory.appendWaypoint(endTime, finalSegmentCoP, zeroVelocity);

         previousFootstep = upcomingFootsteps.get(stepIndex);
      }

      upcomingFootsteps.get(stepIndex - 1).getPosition(footstepPosition);

      endTime += finalTransferDuration;

      initialSegmentCoP.set(finalSegmentCoP);
      finalSegmentCoP.interpolate(initialSegmentCoP, footstepPosition, 0.5);

      copTrajectory.appendWaypoint(endTime, finalSegmentCoP, zeroVelocity);
      copTrajectory.initialize();
   }

   public void compute(double time)
   {
      double timeInCurrentState = time - initialTime;
      timeInCurrentStateRemaining.set(getCurrentStateDuration() - timeInCurrentState);
      if (isInStanding)
      {
         desiredCoPVelocity.setToZero();
         desiredCoPAcceleration.setToZero();
      }
      else
      {
         copTrajectory.compute(timeInCurrentState);
         desiredCoPPosition.set(copTrajectory.getPosition());
         desiredCoPVelocity.set(copTrajectory.getVelocity());
         desiredCoPAcceleration.set(copTrajectory.getAcceleration());
      }
   }

   public double getCurrentStateDuration()
   {
      if (isInTransfer)
         return upcomingTimings.get(0).getTransferTime();
      else
         return upcomingTimings.get(0).getSwingTime();
   }

   public MultipleWaypointsPositionTrajectoryGenerator getCoPTrajectory()
   {
      return copTrajectory;
   }

   public void getDesiredCoPData(FramePoint3D desiredCoPPosition, FrameVector3D desiredCoPVelocity, FrameVector3D desiredCoPAcceleration)
   {
      desiredCoPPosition.set(this.desiredCoPPosition);
      desiredCoPVelocity.set(this.desiredCoPVelocity);
      desiredCoPAcceleration.set(this.desiredCoPAcceleration);
   }

   public boolean isDone()
   {
      return timeInCurrentStateRemaining.getDoubleValue() <= 0;
   }
}
