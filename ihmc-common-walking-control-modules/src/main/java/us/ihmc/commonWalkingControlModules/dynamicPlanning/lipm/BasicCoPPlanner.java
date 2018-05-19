package us.ihmc.commonWalkingControlModules.dynamicPlanning.lipm;

import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.FootstepTiming;
import us.ihmc.robotics.math.trajectories.FrameTrajectory3D;
import us.ihmc.robotics.math.trajectories.SegmentedFrameTrajectory3D;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;

public class BasicCoPPlanner
{
   private final static int numberOfStepsToCompute = 3;
   private final static double finalTransferDuration = 1.0;

   private final static ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final ArrayList<Footstep> upcomingFootsteps = new ArrayList<>();
   private final ArrayList<FootstepTiming> upcomingTimings = new ArrayList<>();

   private final FrameTrajectory3D finalTransferTrajectory = new FrameTrajectory3D(5, worldFrame);

   private final SideDependentList<? extends ContactablePlaneBody> contactableFeet;

   private final FramePoint3D desiredCoPPosition = new FramePoint3D();
   private final FrameVector3D desiredCoPVelocity = new FrameVector3D();
   private final FrameVector3D desiredCoPAcceleration = new FrameVector3D();

   private final SegmentedFrameTrajectory3D copTrajectory;

   private boolean isInTransfer = false;
   private boolean isInStanding = true;

   private double initialTime = 0.0;
   private final YoDouble timeInCurrentStateRemaining;

   public BasicCoPPlanner(SideDependentList<? extends ContactablePlaneBody> contactableFeet, ReferenceFrame midFootZUpGroundFrame, YoVariableRegistry registry)
   {
      this.contactableFeet = contactableFeet;

      timeInCurrentStateRemaining = new YoDouble("timeInCurrentStateRemaining", registry);

      desiredCoPPosition.setToZero();
      copTrajectory = new SegmentedFrameTrajectory3D(10, 5);
   }

   public void clearPlan()
   {
      upcomingFootsteps.clear();
      upcomingTimings.clear();
      copTrajectory.reset();
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

      double startTime = 0.0;
      double endTime = upcomingTimings.get(0).getTransferTime();
      FrameTrajectory3D transferTrajectory = copTrajectory.add();
      transferTrajectory.setCubic(startTime, endTime, desiredCoPPosition, finalSegmentCoP);

      startTime = endTime;
      endTime += upcomingTimings.get(0).getSwingTime();

      FrameTrajectory3D swingTrajectory = copTrajectory.add();
      swingTrajectory.setConstant(startTime, endTime, finalSegmentCoP);

      Footstep previousFootstep = currentFootstep;

      int stepIndex;
      for (stepIndex = 1; stepIndex < Math.min(numberOfStepsToCompute, upcomingFootsteps.size()); stepIndex++)
      {
         startTime = endTime;
         endTime += upcomingTimings.get(stepIndex).getTransferTime();
         initialSegmentCoP.set(finalSegmentCoP);
         finalSegmentCoP.setToZero(previousFootstep.getSoleReferenceFrame());
         finalSegmentCoP.changeFrame(worldFrame);

         transferTrajectory = copTrajectory.add();
         transferTrajectory.setCubic(startTime, endTime, initialSegmentCoP, finalSegmentCoP);

         startTime = endTime;
         endTime += upcomingTimings.get(stepIndex).getSwingTime();

         swingTrajectory = copTrajectory.add();
         swingTrajectory.setConstant(startTime, endTime, finalSegmentCoP);

         previousFootstep = upcomingFootsteps.get(stepIndex);
      }

      upcomingFootsteps.get(stepIndex - 1).getPosition(footstepPosition);

      startTime = endTime;
      endTime += finalTransferDuration;

      initialSegmentCoP.set(finalSegmentCoP);
      finalSegmentCoP.interpolate(initialSegmentCoP, footstepPosition, 0.5);

      FrameTrajectory3D finalTransferTrajectory = copTrajectory.add();
      finalTransferTrajectory.setCubic(startTime, endTime, initialSegmentCoP, finalSegmentCoP);
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

      double startTime = 0.0;
      double endTime = upcomingTimings.get(0).getSwingTime();

      FrameTrajectory3D swingTrajectory = copTrajectory.add();
      swingTrajectory.setCubic(startTime, endTime, desiredCoPPosition, finalSegmentCoP);

      Footstep previousFootstep = currentFootstep;

      int stepIndex;
      for (stepIndex = 1; stepIndex < Math.min(numberOfStepsToCompute, upcomingFootsteps.size()); stepIndex++)
      {
         startTime = endTime;
         endTime += upcomingTimings.get(stepIndex).getTransferTime();
         initialSegmentCoP.set(finalSegmentCoP);
         finalSegmentCoP.setToZero(previousFootstep.getSoleReferenceFrame());
         finalSegmentCoP.changeFrame(worldFrame);

         FrameTrajectory3D transferTrajectory = copTrajectory.add();
         transferTrajectory.setCubic(startTime, endTime, initialSegmentCoP, finalSegmentCoP);

         startTime = endTime;
         endTime += upcomingTimings.get(stepIndex).getSwingTime();

         swingTrajectory = copTrajectory.add();
         swingTrajectory.setConstant(startTime, endTime, finalSegmentCoP);

         previousFootstep = upcomingFootsteps.get(stepIndex);
      }

      upcomingFootsteps.get(stepIndex - 1).getPosition(footstepPosition);

      startTime = endTime;
      endTime += finalTransferDuration;

      initialSegmentCoP.set(finalSegmentCoP);
      finalSegmentCoP.interpolate(initialSegmentCoP, footstepPosition, 0.5);

      FrameTrajectory3D finalTransferTrajectory = copTrajectory.add();
      finalTransferTrajectory.setCubic(startTime, endTime, initialSegmentCoP, finalSegmentCoP);
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
         copTrajectory.update(timeInCurrentState, desiredCoPPosition, desiredCoPVelocity, desiredCoPAcceleration);
   }

   public double getCurrentStateDuration()
   {
      if (isInTransfer)
         return upcomingTimings.get(0).getTransferTime();
      else
         return upcomingTimings.get(0).getSwingTime();
   }

   public SegmentedFrameTrajectory3D getCoPTrajectory()
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
