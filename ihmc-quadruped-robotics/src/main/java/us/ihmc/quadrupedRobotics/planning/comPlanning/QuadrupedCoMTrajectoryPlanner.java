package us.ihmc.quadrupedRobotics.planning.comPlanning;

import java.util.List;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CoMTrajectoryPlanner;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactSequenceCalculator;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateProvider;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.CornerPointViewer;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.planning.trajectory.DCMPlannerInterface;
import us.ihmc.quadrupedRobotics.planning.trajectory.DCMPlannerParameters;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * This is a base class for quadrupeds for dynamic trajectory planning. It is used to generate feasible DCM, CoM, and VRP trajectories. The inputs to this class
 * are a list of {@link QuadrupedTimedStep}, which are converted to a list of {@link ContactStateProvider}, which is then used by the {@link CoMTrajectoryPlanner}.
 * This is done using {@link ContactSequenceCalculator} class.
 */
public class QuadrupedCoMTrajectoryPlanner implements DCMPlannerInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final YoDouble timeAtStartOfState = new YoDouble("timeAtStartOfState", registry);
   private final YoDouble timeInContactPhase = new YoDouble("timeInContactPhase", registry);

   private final YoBoolean holdPosition = new YoBoolean("holdPosition", registry);

   private final YoFramePoint3D comPositionToHold = new YoFramePoint3D("comPositionToHold", ReferenceFrame.getWorldFrame(), registry);

   private final YoFramePoint3D finalDCMPosition = new YoFramePoint3D("plannerFinalDesiredDCMPosition", worldFrame, registry);

   private final YoFramePoint3D desiredDCMPosition = new YoFramePoint3D("plannerDesiredDCMPosition", worldFrame, registry);
   private final YoFrameVector3D desiredDCMVelocity = new YoFrameVector3D("plannerDesiredDCMVelocity", worldFrame, registry);
   private final YoFramePoint3D desiredCoMPosition = new YoFramePoint3D("plannerDesiredCoMPosition", worldFrame, registry);
   private final YoFrameVector3D desiredCoMVelocity = new YoFrameVector3D("plannerDesiredCoMVelocity", worldFrame, registry);
   private final YoFrameVector3D desiredCoMAcceleration = new YoFrameVector3D("plannerDesiredCoMAcceleration", worldFrame, registry);
   private final YoFramePoint3D desiredVRPPosition = new YoFramePoint3D("plannerPerfectVRPPosition", worldFrame, registry);
   private final YoFramePoint3D desiredECMPPosition = new YoFramePoint3D("plannerPerfectECMPPosition", worldFrame, registry);

   private final QuadrantDependentList<MovingReferenceFrame> soleFrames;
   private final QuadrupedNominalContactPhaseCalculator nominalContactPhaseCalculator;
   private final ContactSequenceCalculator<QuadrupedTimedContactInterval> contactSequenceCalculator;
   private final CoMTrajectoryPlanner comTrajectoryPlanner;

   public QuadrupedCoMTrajectoryPlanner(DCMPlannerParameters plannerParameters, QuadrantDependentList<MovingReferenceFrame> soleFrames, double gravity,
                                        double nominalHeight, YoRegistry parentRegistry)
   {
      this(plannerParameters, soleFrames, gravity, nominalHeight, parentRegistry, null);
   }

   public QuadrupedCoMTrajectoryPlanner(DCMPlannerParameters plannerParameters, QuadrantDependentList<MovingReferenceFrame> soleFrames, double gravity,
                                        double nominalHeight, YoRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
   {
      this.soleFrames = soleFrames;
      nominalContactPhaseCalculator = new QuadrupedNominalContactPhaseCalculator(100);
      contactSequenceCalculator = new ContactSequenceCalculator<>(new QuadrupedCoPWaypointCalculator(plannerParameters), registry);

      comTrajectoryPlanner = new CoMTrajectoryPlanner(gravity, nominalHeight, registry);
      if (graphicsListRegistry != null)
         comTrajectoryPlanner.setCornerPointViewer(new CornerPointViewer(registry, graphicsListRegistry));

      parentRegistry.addChild(registry);
   }


   public void initialize()
   {
      nominalContactPhaseCalculator.initialize();
   }

   public void setNominalCoMHeight(double comHeight)
   {
      comTrajectoryPlanner.setNominalCoMHeight(comHeight);
   }

   public void setHoldCurrentDesiredPosition(boolean holdPosition)
   {
      this.holdPosition.set(holdPosition);

      if (holdPosition)
      {
         comPositionToHold.set(comTrajectoryPlanner.getDesiredDCMPosition());
      }
   }

   public void setInitialState(double initialTime, FramePoint3DReadOnly centerOfMassPosition, FrameVector3DReadOnly initialVelocity, FramePoint3DReadOnly copPosition)
   {
      timeAtStartOfState.set(initialTime);
      comTrajectoryPlanner.setInitialCenterOfMassState(centerOfMassPosition, initialVelocity);
      contactSequenceCalculator.setInitialCoP(initialTime, copPosition);
   }

   public void computeSetpoints(double currentTime, List<? extends QuadrupedTimedStep> stepSequence, List<RobotQuadrant> currentFeetInContact)
   {
      if (holdPosition.getBooleanValue())
      {
         desiredCoMPosition.set(comPositionToHold);
         desiredDCMPosition.set(comPositionToHold);

         desiredVRPPosition.set(comPositionToHold);
         desiredECMPPosition.set(desiredVRPPosition);
         desiredECMPPosition.subZ(comTrajectoryPlanner.getNominalCoMHeight());

         desiredCoMVelocity.setToZero();
         desiredCoMAcceleration.setToZero();
         desiredDCMVelocity.setToZero();
      }
      else
      {
         nominalContactPhaseCalculator.computeFromSteps(stepSequence, soleFrames, currentFeetInContact, currentTime, timeAtStartOfState.getDoubleValue());

         List<QuadrupedTimedContactInterval> contactPhases = nominalContactPhaseCalculator.getContactPhases();
         double timeInPhase = currentTime - timeAtStartOfState.getDoubleValue();
         timeInContactPhase.set(timeInPhase);

         List<? extends ContactStateProvider> contactSequence = contactSequenceCalculator.compute(contactPhases);

         comTrajectoryPlanner.solveForTrajectory(contactSequence);
         comTrajectoryPlanner.compute(timeInPhase);

         desiredCoMPosition.set(comTrajectoryPlanner.getDesiredCoMPosition());
         desiredCoMVelocity.set(comTrajectoryPlanner.getDesiredCoMVelocity());
         desiredCoMAcceleration.set(comTrajectoryPlanner.getDesiredCoMAcceleration());

         desiredDCMPosition.set(comTrajectoryPlanner.getDesiredDCMPosition());
         desiredDCMVelocity.set(comTrajectoryPlanner.getDesiredDCMVelocity());

         desiredVRPPosition.set(comTrajectoryPlanner.getDesiredVRPPosition());
         desiredECMPPosition.set(comTrajectoryPlanner.getDesiredECMPPosition());

         comTrajectoryPlanner.compute(contactPhases.get(0).getTimeInterval().getDuration());
         finalDCMPosition.set(comTrajectoryPlanner.getDesiredDCMPosition());
      }
   }

   public FramePoint3DReadOnly getDesiredDCMPosition()
   {
      return desiredDCMPosition;
   }

   public FrameVector3DReadOnly getDesiredDCMVelocity()
   {
      return desiredDCMVelocity;
   }

   public FramePoint3DReadOnly getDesiredCoMPosition()
   {
      return desiredCoMPosition;
   }

   public FrameVector3DReadOnly getDesiredCoMVelocity()
   {
      return desiredCoMVelocity;
   }

   public FrameVector3DReadOnly getDesiredCoMAcceleration()
   {
      return desiredCoMAcceleration;
   }

   public FramePoint3DReadOnly getDesiredVRPPosition()
   {
      return desiredVRPPosition;
   }

   public FramePoint3DReadOnly getDesiredECMPPosition()
   {
      return desiredECMPPosition;
   }

   public FramePoint3DReadOnly getFinalDCMPosition()
   {
      return finalDCMPosition;
   }
}
