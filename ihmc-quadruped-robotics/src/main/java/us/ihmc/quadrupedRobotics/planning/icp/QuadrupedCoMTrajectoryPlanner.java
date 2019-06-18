package us.ihmc.quadrupedRobotics.planning.icp;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMTrajectoryPlanner;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMTrajectoryPlannerInterface;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateProvider;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.ArrayList;
import java.util.List;

/**
 * This is a base class for quadrupeds for dynamic trajectory planning. It is used to generate feasible DCM, CoM, and VRP trajectories. The inputs to this class
 * are a list of {@link QuadrupedTimedStep}, which are converted to a list of {@link ContactStateProvider}, which is then used by the {@link CoMTrajectoryPlanner}.
 * This is done using {@link QuadrupedContactSequenceUpdater} class.
 */
public class QuadrupedCoMTrajectoryPlanner implements CoMTrajectoryPlannerInterface
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final List<QuadrupedTimedStep> stepSequence = new ArrayList<>();

   private final DoubleProvider timestamp;
   private final YoDouble timeInContactPhase = new YoDouble("timeInContactPhase", registry);

   private final QuadrupedContactSequenceUpdater contactSequenceUpdater;
   private final CoMTrajectoryPlanner comTrajectoryPlanner;

   private final List<RobotQuadrant> currentFeetInContact = new ArrayList<>();

   public QuadrupedCoMTrajectoryPlanner(QuadrantDependentList<MovingReferenceFrame> soleFrames, DoubleProvider timestamp, DoubleProvider omega, double gravity,
                                        double nominalHeight, YoVariableRegistry parentRegistry)
   {
      this(soleFrames, timestamp, omega, gravity, nominalHeight, parentRegistry, null);
   }

   public QuadrupedCoMTrajectoryPlanner(QuadrantDependentList<MovingReferenceFrame> soleFrames, DoubleProvider timestamp, DoubleProvider omega, double gravity,
                                        double nominalHeight, YoVariableRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
   {
      this.timestamp = timestamp;
      contactSequenceUpdater = new QuadrupedContactSequenceUpdater(soleFrames, 4, 10, registry, graphicsListRegistry);

      comTrajectoryPlanner = new CoMTrajectoryPlanner(omega, gravity, nominalHeight, registry, graphicsListRegistry);

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      contactSequenceUpdater.initialize();
   }

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

   public void setInitialCenterOfMassState(FramePoint3DReadOnly centerOfMassPosition, FrameVector3DReadOnly centerOfMassVelocity)
   {
      comTrajectoryPlanner.setInitialCenterOfMassState(centerOfMassPosition, centerOfMassVelocity);
   }

   public void initializeForStanding()
   {
      currentFeetInContact.clear();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         currentFeetInContact.add(robotQuadrant);
      contactSequenceUpdater.initialize();
   }

   public void initializeForStepping(QuadrantDependentList<YoEnum<ContactState>> currentContactStates)
   {
      updateFeetInContactFromContactStates(currentContactStates);
      contactSequenceUpdater.initialize();
   }

   void computeSetpoints(double currentTime, List<RobotQuadrant> currentFeetInContact)
   {
      contactSequenceUpdater.update(stepSequence, currentFeetInContact, currentTime);

      double timeInPhase = currentTime - contactSequenceUpdater.getContactSequence().get(0).getTimeInterval().getStartTime();
      timeInContactPhase.set(timeInPhase);

      comTrajectoryPlanner.solveForTrajectory(contactSequenceUpdater.getContactSequence());
      comTrajectoryPlanner.compute(timeInContactPhase.getDoubleValue());
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

   @Override
   public void solveForTrajectory(List<? extends ContactStateProvider> contactSequence)
   {
      comTrajectoryPlanner.solveForTrajectory(contactSequenceUpdater.getContactSequence());
   }

   @Override
   public void compute(double timeInPhase)
   {
      comTrajectoryPlanner.compute(timeInPhase);
   }

   @Override
   public void compute(int segmentId, double timeInPhase)
   {
      throw new RuntimeException("This method is not valid for this implementation of the planner.");
   }

   /** {@inheritDoc} */
   @Override
   public void compute(int segmentId, double timeInPhase, FixedFramePoint3DBasics comPositionToPack, FixedFrameVector3DBasics comVelocityToPack,
                       FixedFrameVector3DBasics comAccelerationToPack, FixedFramePoint3DBasics dcmPositionToPack, FixedFrameVector3DBasics dcmVelocityToPack,
                       FixedFramePoint3DBasics vrpPositionToPack)
   {
      throw new RuntimeException("This method is not valid for this implementation of the planner.");
   }

   @Override
   public FramePoint3DReadOnly getDesiredDCMPosition()
   {
      return comTrajectoryPlanner.getDesiredDCMPosition();
   }

   @Override
   public FrameVector3DReadOnly getDesiredDCMVelocity()
   {
      return comTrajectoryPlanner.getDesiredDCMVelocity();
   }

   @Override
   public FramePoint3DReadOnly getDesiredCoMPosition()
   {
      return comTrajectoryPlanner.getDesiredCoMPosition();
   }

   @Override
   public FrameVector3DReadOnly getDesiredCoMVelocity()
   {
      return comTrajectoryPlanner.getDesiredCoMVelocity();
   }

   @Override
   public FrameVector3DReadOnly getDesiredCoMAcceleration()
   {
      return comTrajectoryPlanner.getDesiredCoMAcceleration();
   }

   @Override
   public FramePoint3DReadOnly getDesiredVRPPosition()
   {
      return comTrajectoryPlanner.getDesiredVRPPosition();
   }

}
