package us.ihmc.quadrupedRobotics.planning.icp;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.ArrayList;
import java.util.List;

public class DCMBasedCoMPlanner
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final List<QuadrupedTimedStep> stepSequence = new ArrayList<>();

   private final YoDouble timeInContactPhase = new YoDouble("timeInContactPhase", registry);

   private final QuadrupedContactSequenceUpdater contactSequenceUpdater;
   private final CoMTrajectoryPlanner comTrajectoryPlanner;

   public DCMBasedCoMPlanner(QuadrantDependentList<ReferenceFrame> soleFrames, YoDouble omega, double gravity, double nominalHeight, YoVariableRegistry parentRegistry)
   {
      contactSequenceUpdater = new QuadrupedContactSequenceUpdater(soleFrames, 4, 10);

      comTrajectoryPlanner = new CoMTrajectoryPlanner(contactSequenceUpdater.getContactSequence(), omega, gravity, nominalHeight, registry);

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      contactSequenceUpdater.initialize();
   }

   void setNominalCoMHeight(double comHeight)
   {
      comTrajectoryPlanner.setNominalCoMHeight(comHeight);
   }

   public void computeSetpoints(double currentTime, List<RobotQuadrant> currentFeetInContact, FixedFramePoint3DBasics desiredDCMPositionToPack,
                                FixedFrameVector3DBasics desiredDCMVelocityToPack)
   {
      contactSequenceUpdater.update(stepSequence, currentFeetInContact, currentTime);

      double timeInPhase = currentTime - contactSequenceUpdater.getContactSequence().getFirst().getTimeInterval().getStartTime();
      timeInContactPhase.set(timeInPhase);

      comTrajectoryPlanner.solveForTrajectory();
      comTrajectoryPlanner.compute(timeInContactPhase.getDoubleValue());

      desiredDCMPositionToPack.set(comTrajectoryPlanner.getDesiredDCMPosition());
      desiredDCMVelocityToPack.set(comTrajectoryPlanner.getDesiredDCMVelocity());
   }

   public void clearStepSequence()
   {
      stepSequence.clear();
   }

   public void addStepToSequence(QuadrupedTimedStep step)
   {
      stepSequence.add(step);
   }

   public void initializeForStanding()
   {
      contactSequenceUpdater.initialize();
   }

   public void initializeForStepping()
   {
      contactSequenceUpdater.initialize();
   }
}
