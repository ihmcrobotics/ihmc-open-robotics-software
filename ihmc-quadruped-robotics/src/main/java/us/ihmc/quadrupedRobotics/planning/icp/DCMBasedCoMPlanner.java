package us.ihmc.quadrupedRobotics.planning.icp;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedStep;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;

public class DCMBasedCoMPlanner
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final List<QuadrupedTimedStep> stepSequence = new ArrayList<>();

   private final double gravity;
   private final YoDouble omega = new YoDouble("omega", registry);
   private final YoDouble timeInContactPhase = new YoDouble("timeInContactPhase", registry);

   private final QuadrupedContactSequence contactSequence;
   private final CoMTrajectoryPlanner comTrajectoryPlanner;

   public DCMBasedCoMPlanner(QuadrantDependentList<ReferenceFrame> soleFrames, double gravity, double nominalHeight, YoVariableRegistry parentRegistry)
   {
      this.gravity = gravity;
      contactSequence = new QuadrupedContactSequence(soleFrames, 4, 10);
      omega.set(Math.sqrt(gravity / nominalHeight));

      comTrajectoryPlanner = new CoMTrajectoryPlanner(contactSequence, omega, registry);

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      contactSequence.initialize();
   }

   public void computeSetpoints(double currentTime, QuadrantDependentList<ContactState> currentContactStates, FixedFramePoint3DBasics desiredDCMPositionToPack,
                                FixedFrameVector3DBasics desiredDCMVelocityToPack)
   {
      double timeInPhase = currentTime - contactSequence.getFirst().getTimeInterval().getStartTime();
      timeInContactPhase.set(timeInPhase);

      contactSequence.update(stepSequence, currentContactStates, currentTime);
      comTrajectoryPlanner.solveForTrajectory();
      comTrajectoryPlanner.compute(timeInPhase);

      desiredDCMPositionToPack.set(comTrajectoryPlanner.getDesiredICPPosition());
      desiredDCMVelocityToPack.set(comTrajectoryPlanner.getDesiredICPVelocity());
   }

   public void clearStepSequence()
   {
      stepSequence.clear();
   }

   public void addStepToSequence(QuadrupedTimedStep step)
   {
      stepSequence.add(step);
   }

   void setCoMHeight(double comHeight)
   {
      omega.set(Math.sqrt(gravity / comHeight));
   }

   public void initializeForStanding()
   {
      contactSequence.initialize();
   }

   public void initializeForStepping()
   {
      contactSequence.initialize();
   }

   void getFinalDesiredDCM(FixedFramePoint3DBasics finalDesiredDCMToPack)
   {

   }

   double getFinalTime()
   {
      return Double.POSITIVE_INFINITY;
   }

   private void computeTrajectories(double currentTime, QuadrantDependentList<ContactState> currentContactStates)
   {
      contactSequence.update(stepSequence, currentContactStates, currentTime);
   }

}
