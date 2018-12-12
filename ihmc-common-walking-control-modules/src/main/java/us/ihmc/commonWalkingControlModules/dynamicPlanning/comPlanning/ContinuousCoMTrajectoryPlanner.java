package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.math.trajectories.FrameTrajectory3D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

import java.util.List;

public class ContinuousCoMTrajectoryPlanner implements CoMTrajectoryPlannerInterface
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final CoMTrajectoryPlannerInterface comTrajectoryPlanner;
   private final List<? extends ContactStateProvider> contactSequence;

   private final YoFramePoint3D initialCenterOfMassPosition = new YoFramePoint3D("initialCoMPosition", worldFrame, registry);
   private final YoFrameVector3D initialCenterOfMassVelocity = new YoFrameVector3D("initialCoMVelocity", worldFrame, registry);

   private final YoFramePoint3D comPositionAfterTransition = new YoFramePoint3D("comPositionAfterTransition", worldFrame, registry);
   private final YoFrameVector3D comVelocityAfterTransition = new YoFrameVector3D("comVelocityAfterTransition", worldFrame, registry);

   private final YoDouble maximumSplineDuration = new YoDouble("maximumSplineDuration", registry);

   private final YoDouble currentSegmentDuration = new YoDouble("currentSegmentDuration", registry);
   private final YoDouble nextSegmentDuration = new YoDouble("nextSegmentDuration", registry);

   private final SegmentedCoMTrajectory transitionTrajectory = new SegmentedCoMTrajectory();

   private final FixedFramePoint3DBasics desiredCoMPosition = new FramePoint3D(worldFrame);
   private final FixedFrameVector3DBasics desiredCoMVelocity = new FrameVector3D(worldFrame);
   private final FixedFrameVector3DBasics desiredCoMAcceleration = new FrameVector3D(worldFrame);

   private final FixedFramePoint3DBasics desiredDCMPosition = new FramePoint3D(worldFrame);
   private final FixedFrameVector3DBasics desiredDCMVelocity = new FrameVector3D(worldFrame);

   private final FixedFramePoint3DBasics desiredVRPPosition = new FramePoint3D(worldFrame);

   private final DoubleProvider omega;
   private final double gravityZ;

   public ContinuousCoMTrajectoryPlanner(CoMTrajectoryPlannerInterface comTrajectoryPlanner, List<? extends ContactStateProvider> contactSequence,
                                         DoubleProvider omega, double gravityZ, YoVariableRegistry parentRegistry,
                                         YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this.comTrajectoryPlanner = comTrajectoryPlanner;
      this.contactSequence = contactSequence;
      this.omega = omega;
      this.gravityZ = gravityZ;

      parentRegistry.addChild(registry);
   }

   /** {@inheritDoc} */
   @Override
   public void setNominalCoMHeight(double nominalCoMHeight)
   {
      comTrajectoryPlanner.setNominalCoMHeight(nominalCoMHeight);
   }

   /** {@inheritDoc} */
   @Override
   public void solveForTrajectory()
   {
      comTrajectoryPlanner.solveForTrajectory();
   }

   /** {@inheritDoc} */
   @Override
   public void compute(double timeInPhase)
   {
      currentSegmentDuration.set(contactSequence.get(0).getTimeInterval().getDuration());
      nextSegmentDuration.set(contactSequence.get(1).getTimeInterval().getDuration());

      ContactState firstContactState = contactSequence.get(0).getContactState();
      ContactState secondContactState = contactSequence.get(1).getContactState();
      double omega = this.omega.getValue();

      double alpha = 0.5;
      double startOfSplineTime = currentSegmentDuration.getDoubleValue() - alpha * maximumSplineDuration.getDoubleValue();
      startOfSplineTime = Math.max(startOfSplineTime, 0.0);
      double endOfSplineTime = currentSegmentDuration.getDoubleValue() + (1.0 - alpha) * maximumSplineDuration.getDoubleValue();
      endOfSplineTime = Math.min(endOfSplineTime, nextSegmentDuration.getDoubleValue());

      comTrajectoryPlanner.compute(1, endOfSplineTime);
      comPositionAfterTransition.set(comTrajectoryPlanner.getDesiredCoMPosition());
      comVelocityAfterTransition.set(comTrajectoryPlanner.getDesiredDCMVelocity());

      transitionTrajectory.set(0.0, startOfSplineTime, startOfSplineTime + endOfSplineTime, omega, gravityZ, firstContactState, secondContactState,
                               initialCenterOfMassPosition, initialCenterOfMassVelocity, comPositionAfterTransition, comVelocityAfterTransition);

      transitionTrajectory.compute(timeInPhase);

      desiredCoMPosition.set(transitionTrajectory.getDesiredComPosition());
      desiredCoMVelocity.set(transitionTrajectory.getDesiredComVelocity());
      desiredCoMAcceleration.set(transitionTrajectory.getDesiredComAcceleration());

      CapturePointTools.computeDesiredCapturePointPosition(desiredCoMPosition, desiredCoMVelocity, omega, desiredDCMPosition);
      CapturePointTools.computeDesiredCapturePointVelocity(desiredCoMVelocity, desiredCoMAcceleration, omega, desiredDCMVelocity);
      CapturePointTools.computeDesiredCentroidalMomentumPivot(desiredDCMPosition, desiredDCMVelocity, omega, desiredVRPPosition);
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

   /** {@inheritDoc} */
   public void setInitialCenterOfMassState(FramePoint3DReadOnly centerOfMassPosition, FrameVector3DReadOnly centerOfMassVelocity)
   {
      initialCenterOfMassPosition.set(centerOfMassPosition);
      initialCenterOfMassVelocity.set(centerOfMassVelocity);

      comTrajectoryPlanner.setInitialCenterOfMassState(initialCenterOfMassPosition, initialCenterOfMassVelocity);
   }

   /** {@inheritDoc} */
   @Override
   public FramePoint3DReadOnly getDesiredDCMPosition()
   {
      return desiredDCMPosition;
   }

   /** {@inheritDoc} */
   @Override
   public FrameVector3DReadOnly getDesiredDCMVelocity()
   {
      return desiredDCMVelocity;
   }

   /** {@inheritDoc} */
   @Override
   public FramePoint3DReadOnly getDesiredCoMPosition()
   {
      return desiredCoMPosition;
   }

   /** {@inheritDoc} */
   @Override
   public FrameVector3DReadOnly getDesiredCoMVelocity()
   {
      return desiredCoMVelocity;
   }

   /** {@inheritDoc} */
   @Override
   public FrameVector3DReadOnly getDesiredCoMAcceleration()
   {
      return desiredCoMAcceleration;
   }

   /** {@inheritDoc} */
   @Override
   public FramePoint3DReadOnly getDesiredVRPPosition()
   {
      return desiredVRPPosition;
   }
}
