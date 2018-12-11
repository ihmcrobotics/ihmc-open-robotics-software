package us.ihmc.quadrupedRobotics.planning.icp;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.quadrupedRobotics.planning.trajectory.DCMPlannerInterface;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.ArrayList;
import java.util.List;

public class DCMBasedCoMPlanner implements DCMPlannerInterface
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final List<QuadrupedTimedStep> stepSequence = new ArrayList<>();

   private final DoubleProvider timestamp;
   private final YoDouble timeInContactPhase = new YoDouble("timeInContactPhase", registry);

   private final QuadrupedContactSequenceUpdater contactSequenceUpdater;
   private final CoMTrajectoryPlanner comTrajectoryPlanner;

   private final List<RobotQuadrant> currentFeetInContact = new ArrayList<>();

   public DCMBasedCoMPlanner(QuadrantDependentList<MovingReferenceFrame> soleFrames, DoubleProvider timestamp, DoubleProvider omega, double gravity, double nominalHeight,
                             YoVariableRegistry parentRegistry)
   {
      this.timestamp = timestamp;
      contactSequenceUpdater = new QuadrupedContactSequenceUpdater(soleFrames, 4, 10);

      comTrajectoryPlanner = new CoMTrajectoryPlanner(contactSequenceUpdater.getContactSequence(), omega, gravity, nominalHeight, registry);

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      contactSequenceUpdater.initialize();
   }

   @Override
   public void setNominalCoMHeight(double comHeight)
   {
      comTrajectoryPlanner.setNominalCoMHeight(comHeight);
   }



   public void clearStepSequence()
   {
      stepSequence.clear();
   }

   public void addStepToSequence(QuadrupedTimedStep step)
   {
      stepSequence.add(step);
   }

   @Override
   public void initializeForStanding()
   {
      currentFeetInContact.clear();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         currentFeetInContact.add(robotQuadrant);

      contactSequenceUpdater.initialize();
   }

   @Override
   public void initializeForStepping(QuadrantDependentList<YoEnum<ContactState>> currentContactStates, FramePoint3DReadOnly dcmPosition)
   {
      updateFeetInContactFromContactStates(currentContactStates);
      contactSequenceUpdater.initialize();
   }

   @Override
   public void computeDcmSetpoints(QuadrantDependentList<YoEnum<ContactState>> currentContactStates, FixedFramePoint3DBasics desiredDCMPositionToPack,
                                   FixedFrameVector3DBasics desiredDCMVelocityToPack)
   {
      updateFeetInContactFromContactStates(currentContactStates);

      computeSetpoints(timestamp.getValue(), currentFeetInContact, desiredDCMPositionToPack, desiredDCMVelocityToPack);
   }

   @Override
   public void getFinalDCMPosition(FixedFramePoint3DBasics finalDesiredDCMToPack)
   {

   }

   @Override
   public void getDesiredECMPPosition(FramePoint3DBasics eCMPPositionToPack)
   {
      eCMPPositionToPack.set(comTrajectoryPlanner.getDesiredECMPPosition());
   }

   void computeSetpoints(double currentTime, List<RobotQuadrant> currentFeetInContact, FixedFramePoint3DBasics desiredDCMPositionToPack,
                                FixedFrameVector3DBasics desiredDCMVelocityToPack)
   {
      contactSequenceUpdater.update(stepSequence, currentFeetInContact, currentTime);

      double timeInPhase = currentTime - contactSequenceUpdater.getContactSequence().get(0).getTimeInterval().getStartTime();
      timeInContactPhase.set(timeInPhase);

      comTrajectoryPlanner.solveForTrajectory();
      comTrajectoryPlanner.compute(timeInContactPhase.getDoubleValue());

      desiredDCMPositionToPack.set(comTrajectoryPlanner.getDesiredDCMPosition());
      desiredDCMVelocityToPack.set(comTrajectoryPlanner.getDesiredDCMVelocity());
   }

   private void updateFeetInContactFromContactStates(QuadrantDependentList<YoEnum<ContactState>> currentContactStates)
   {
      currentFeetInContact.clear();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (currentContactStates.get(robotQuadrant).getEnumValue().isLoadBearing())
            currentFeetInContact.add(robotQuadrant);
      }
   }
}
