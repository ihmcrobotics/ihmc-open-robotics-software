package us.ihmc.robotics.math.trajectories;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.robotics.controllers.pidGains.GainCalculator;
import us.ihmc.robotics.linearAlgebra.MatrixExponentialCalculator;
import us.ihmc.robotics.math.trajectories.interfaces.FixedFramePositionTrajectoryGenerator;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class C1ContinuousTrajectorySmoother implements FixedFramePositionTrajectoryGenerator
{
   private final FixedFramePositionTrajectoryGenerator trajectoryToTrack;

   private final YoFrameVector3D desiredPositionError;
   private final YoFrameVector3D desiredVelocityError;

   private final FramePoint3D desiredPosition;
   private final FrameVector3D desiredVelocity;
   private final FrameVector3D desiredAcceleration;

   private final YoDouble timeToStartErrorCancellation;
   private final DoubleParameter trackingStiffness;
   private final DoubleParameter trackingZeta;

   private final MatrixExponentialCalculator matrixExponentialCalculator = new MatrixExponentialCalculator(2);
   private final DMatrixRMaj closedLoopStateMatrix = new DMatrixRMaj(2, 2);
   private final DMatrixRMaj scaledClosedLoopStateMatrix = new DMatrixRMaj(2, 2);
   private final DMatrixRMaj stateTransitionMatrix = new DMatrixRMaj(2, 2);

   public C1ContinuousTrajectorySmoother(String namePrefix, FixedFramePositionTrajectoryGenerator trajectoryToTrack, YoRegistry parentRegistry)
   {
      this.trajectoryToTrack = trajectoryToTrack;

      YoRegistry registry = new YoRegistry(namePrefix + getClass().getSimpleName());

      trackingStiffness = new DoubleParameter(namePrefix + "TrackingStiffness", registry, 25.0);
      trackingZeta = new DoubleParameter(namePrefix + "TrackingZeta", registry, 0.7);

      timeToStartErrorCancellation = new YoDouble(namePrefix + "TimeWhenStartingErrorCancellation", registry);
      desiredPositionError = new YoFrameVector3D(namePrefix + "PositionErrorWhenStartingCancellation", trajectoryToTrack.getReferenceFrame(), registry);
      desiredVelocityError = new YoFrameVector3D(namePrefix + "VelocityErrorWhenStartingCancellation", trajectoryToTrack.getReferenceFrame(), registry);

      desiredPosition = new FramePoint3D(trajectoryToTrack.getReferenceFrame());
      desiredVelocity = new FrameVector3D(trajectoryToTrack.getReferenceFrame());
      desiredAcceleration = new FrameVector3D(trajectoryToTrack.getReferenceFrame());

//      closedLoopStateMatrix.set(0, 1, 1.0);
//      closedLoopStateMatrix.set(1, 0, trackingStiffness.getValue());
//      closedLoopStateMatrix.set(1, 1, trackingDamping.getValue());

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      desiredPositionError.setToZero();
      desiredVelocityError.setToZero();
      timeToStartErrorCancellation.set(0.0);
   }

   public void updateErrorDynamicsAtTime(double time, FramePoint3DReadOnly desiredPositionAtTime, FrameVector3DReadOnly desiredVelocityAtTime)
   {
      closedLoopStateMatrix.set(0, 1, 1.0);
      closedLoopStateMatrix.set(1, 0, -trackingStiffness.getValue());
      closedLoopStateMatrix.set(1, 1, -GainCalculator.computeDerivativeGain(trackingStiffness.getValue(), trackingZeta.getValue()));

      trajectoryToTrack.compute(time);

      desiredPositionError.sub(desiredPositionAtTime, trajectoryToTrack.getPosition());
      desiredVelocityError.sub(desiredVelocityAtTime, trajectoryToTrack.getVelocity());

      timeToStartErrorCancellation.set(time);
   }

   private final FrameVector3D instantaneousPositionErrorSolution = new FrameVector3D();
   private final FrameVector3D instantaneousVelocityErrorSolution = new FrameVector3D();
   private final FrameVector3D instantaneousErrorAccelerationSolution = new FrameVector3D();

   @Override
   public void compute(double time)
   {
      double relativeTime = time - timeToStartErrorCancellation.getDoubleValue();
      CommonOps_DDRM.scale(relativeTime, closedLoopStateMatrix, scaledClosedLoopStateMatrix);

      matrixExponentialCalculator.compute(stateTransitionMatrix, scaledClosedLoopStateMatrix);

      double dampingGain = GainCalculator.computeDerivativeGain(trackingStiffness.getValue(), trackingZeta.getValue());
      for (int element = 0; element < 3; element++)
      {
         double position = stateTransitionMatrix.get(0, 0) * desiredPositionError.getElement(element) +
                           stateTransitionMatrix.get(0, 1) * desiredVelocityError.getElement(element);
         double velocity = stateTransitionMatrix.get(1, 0) * desiredPositionError.getElement(element) +
                           stateTransitionMatrix.get(1, 1) * desiredVelocityError.getElement(element);
         instantaneousPositionErrorSolution.setElement(element, position);
         instantaneousVelocityErrorSolution.setElement(element, velocity);
         instantaneousErrorAccelerationSolution.setElement(element, trackingStiffness.getValue() * position + dampingGain * velocity);
      }

      trajectoryToTrack.compute(time);

      desiredPosition.add(trajectoryToTrack.getPosition(), instantaneousPositionErrorSolution);
      desiredVelocity.add(trajectoryToTrack.getVelocity(), instantaneousVelocityErrorSolution);
      desiredAcceleration.add(trajectoryToTrack.getAcceleration(), instantaneousErrorAccelerationSolution);
   }

   @Override
   public boolean isDone()
   {
      return trajectoryToTrack.isDone();
   }

   @Override
   public FramePoint3DReadOnly getPosition()
   {
      return desiredPosition;
   }

   @Override
   public FrameVector3DReadOnly getVelocity()
   {
      return desiredVelocity;
   }

   @Override
   public FrameVector3DReadOnly getAcceleration()
   {
      return desiredAcceleration;
   }

   @Override
   public void showVisualization()
   {

   }

   @Override
   public void hideVisualization()
   {

   }

   FrameVector3DReadOnly getReferenceTrackingPositionError()
   {
      return desiredPositionError;
   }

   FrameVector3DReadOnly getReferenceTrackingVelocityError()
   {
      return desiredVelocityError;
   }
}
