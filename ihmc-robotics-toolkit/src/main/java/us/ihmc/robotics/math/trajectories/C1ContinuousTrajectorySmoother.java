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

   private final YoFrameVector3D referencePositionError;
   private final YoFrameVector3D referenceVelocityError;
   private final YoFrameVector3D referenceAccelerationError;

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

   private boolean hasDriftDynamicsMatrixBeenSet = false;

   public C1ContinuousTrajectorySmoother(String namePrefix, FixedFramePositionTrajectoryGenerator trajectoryToTrack, YoRegistry parentRegistry)
   {
      this.trajectoryToTrack = trajectoryToTrack;

      YoRegistry registry = new YoRegistry(namePrefix + getClass().getSimpleName());

      trackingStiffness = new DoubleParameter(namePrefix + "TrackingStiffness", registry, 200.0);
      trackingZeta = new DoubleParameter(namePrefix + "TrackingZeta", registry, 0.8);

      trackingStiffness.addListener((v) -> updateClosedLoopDriftDynamicsMatrix());
      trackingZeta.addListener((v) -> updateClosedLoopDriftDynamicsMatrix());

      timeToStartErrorCancellation = new YoDouble(namePrefix + "TimeWhenStartingErrorCancellation", registry);
      desiredPositionError = new YoFrameVector3D(namePrefix + "PositionErrorWhenStartingCancellation", trajectoryToTrack.getReferenceFrame(), registry);
      desiredVelocityError = new YoFrameVector3D(namePrefix + "VelocityErrorWhenStartingCancellation", trajectoryToTrack.getReferenceFrame(), registry);

      referencePositionError = new YoFrameVector3D(namePrefix + "ReferencePositionError", trajectoryToTrack.getReferenceFrame(), registry);
      referenceVelocityError = new YoFrameVector3D(namePrefix + "ReferenceVelocityError", trajectoryToTrack.getReferenceFrame(), registry);
      referenceAccelerationError = new YoFrameVector3D(namePrefix + "ReferenceAccelerationError", trajectoryToTrack.getReferenceFrame(), registry);

      desiredPosition = new FramePoint3D(trajectoryToTrack.getReferenceFrame());
      desiredVelocity = new FrameVector3D(trajectoryToTrack.getReferenceFrame());
      desiredAcceleration = new FrameVector3D(trajectoryToTrack.getReferenceFrame());

      parentRegistry.addChild(registry);
   }

   private void updateClosedLoopDriftDynamicsMatrix()
   {
      hasDriftDynamicsMatrixBeenSet = true;

      closedLoopStateMatrix.set(0, 1, 1.0);
      closedLoopStateMatrix.set(1, 0, -trackingStiffness.getValue());
      closedLoopStateMatrix.set(1, 1, -GainCalculator.computeDerivativeGain(trackingStiffness.getValue(), trackingZeta.getValue()));
   }

   public void updateErrorDynamicsAtTime(double time, FramePoint3DReadOnly desiredPositionAtTime, FrameVector3DReadOnly desiredVelocityAtTime)
   {
      trajectoryToTrack.compute(time);

      desiredPositionError.sub(desiredPositionAtTime, trajectoryToTrack.getPosition());
      desiredVelocityError.sub(desiredVelocityAtTime, trajectoryToTrack.getVelocity());

      timeToStartErrorCancellation.set(time);
   }

   @Override
   public void initialize()
   {
      desiredPositionError.setToZero();
      desiredVelocityError.setToZero();
      timeToStartErrorCancellation.set(0.0);
   }

   @Override
   public void compute(double time)
   {
      if (!hasDriftDynamicsMatrixBeenSet)
         updateClosedLoopDriftDynamicsMatrix();

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
         referencePositionError.setElement(element, position);
         referenceVelocityError.setElement(element, velocity);
         referenceAccelerationError.setElement(element, -trackingStiffness.getValue() * position - dampingGain * velocity);
      }

      trajectoryToTrack.compute(time);

      desiredPosition.add(trajectoryToTrack.getPosition(), referencePositionError);
      desiredVelocity.add(trajectoryToTrack.getVelocity(), referenceVelocityError);
      desiredAcceleration.add(trajectoryToTrack.getAcceleration(), referenceAccelerationError);
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
}
