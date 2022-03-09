package us.ihmc.robotics.math.trajectories;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.robotics.linearAlgebra.MatrixExponentialCalculator;
import us.ihmc.robotics.math.trajectories.interfaces.FixedFramePositionTrajectoryGenerator;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class C1ContinuousTrajectorySmoother implements FixedFramePositionTrajectoryGenerator
{
   private final FixedFramePositionTrajectoryGenerator trajectoryToTrack;

   private final YoFrameVector3D desiredPositionError;
   private final YoFrameVector3D desiredVelocityError;
   private final YoFrameVector3D desiredErrorAcceleration;

   private final YoFramePoint3D desiredPositionState;
   private final YoFrameVector3D desiredVelocityState;
   private final YoFrameVector3D desiredAccelerationState;

   private final FramePoint3D desiredPosition;
   private final FrameVector3D desiredVelocity;
   private final FrameVector3D desiredAcceleration;

   private final YoDouble timeLastTick;
   private final DoubleParameter trackingStiffness;
   private final DoubleParameter trackingDamping;

   private final MatrixExponentialCalculator matrixExponentialCalculator = new MatrixExponentialCalculator(2);
   private final DMatrixRMaj closedLoopStateMatrix = new DMatrixRMaj(2, 2);
   private final DMatrixRMaj scaledClosedLoopStateMatrix = new DMatrixRMaj(2, 2);
   private final DMatrixRMaj stateTransitionMatrix = new DMatrixRMaj(2, 2);

   public C1ContinuousTrajectorySmoother(String namePrefix, FixedFramePositionTrajectoryGenerator trajectoryToTrack, YoRegistry parentRegistry)
   {
      this.trajectoryToTrack = trajectoryToTrack;

      YoRegistry registry = new YoRegistry(namePrefix + getClass().getSimpleName());

      trackingStiffness = new DoubleParameter(namePrefix + "TrackingStiffness", registry, 10.0);
      trackingDamping = new DoubleParameter(namePrefix + "TrackingDamping", registry, 2.0);

      timeLastTick = new YoDouble(namePrefix + "TimeLastTick", registry);
      desiredPositionError = new YoFrameVector3D(namePrefix + "DesiredPositionError", trajectoryToTrack.getReferenceFrame(), registry);
      desiredVelocityError = new YoFrameVector3D(namePrefix + "DesiredVelocityError", trajectoryToTrack.getReferenceFrame(), registry);
      desiredErrorAcceleration = new YoFrameVector3D(namePrefix + "DesiredErrorAcceleration", trajectoryToTrack.getReferenceFrame(), registry);

      desiredPositionState = new YoFramePoint3D(namePrefix + "DesiredPositionState", trajectoryToTrack.getReferenceFrame(), registry);
      desiredVelocityState = new YoFrameVector3D(namePrefix + "DesiredVelocityState", trajectoryToTrack.getReferenceFrame(), registry);
      desiredAccelerationState = new YoFrameVector3D(namePrefix + "DesiredAccelerationState", trajectoryToTrack.getReferenceFrame(), registry);

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
      timeLastTick.set(0.0);
   }

   public void updateUsingCurrentErrorAtTime(double time)
   {
      double dt = time - timeLastTick.getDoubleValue();

      // integrate to get the new desired value
      desiredPositionState.scaleAdd(dt, desiredVelocityState, desiredPositionState);
      desiredPositionState.scaleAdd(0.5 * dt * dt, desiredAccelerationState, desiredPositionState);

      desiredVelocityState.scaleAdd(dt, desiredAccelerationState, desiredVelocityState);

      trajectoryToTrack.compute(time);

      desiredPositionError.sub(desiredPositionState, trajectoryToTrack.getPosition());
      desiredVelocityError.sub(desiredVelocityState, trajectoryToTrack.getVelocity());

      desiredErrorAcceleration.setAndScale(trackingStiffness.getValue(), desiredPositionError);
      desiredErrorAcceleration.scaleAdd(trackingDamping.getValue(), desiredVelocityError, desiredErrorAcceleration);

      desiredAccelerationState.add(desiredErrorAcceleration, trajectoryToTrack.getAcceleration());

      desiredPosition.set(desiredPositionState);
      desiredVelocity.set(desiredVelocityState);
      desiredAcceleration.set(desiredAccelerationState);

      timeLastTick.set(time);

      closedLoopStateMatrix.set(0, 1, 1.0);
      closedLoopStateMatrix.set(1, 0, trackingStiffness.getValue());
      closedLoopStateMatrix.set(1, 1, trackingDamping.getValue());
   }

   private final FrameVector3D instantaneousPositionErrorSolution = new FrameVector3D();
   private final FrameVector3D instantaneousVelocityErrorSolution = new FrameVector3D();
   private final FrameVector3D instantaneousErrorAccelerationSolution = new FrameVector3D();

   @Override
   public void compute(double time)
   {
      double relativeTime = time - timeLastTick.getDoubleValue();
      CommonOps_DDRM.scale(relativeTime, closedLoopStateMatrix, scaledClosedLoopStateMatrix);

      matrixExponentialCalculator.compute(stateTransitionMatrix, scaledClosedLoopStateMatrix);

      for (int element = 0; element < 3; element++)
      {
         double position = stateTransitionMatrix.get(0, 0) * desiredPositionError.getElement(element) +
                           stateTransitionMatrix.get(0, 1) * desiredVelocityError.getElement(element);
         double velocity = stateTransitionMatrix.get(1, 0) * desiredPositionError.getElement(element) +
                           stateTransitionMatrix.get(1, 1) * desiredVelocityError.getElement(element);
         instantaneousPositionErrorSolution.setElement(element, position);
         instantaneousVelocityErrorSolution.setElement(element, velocity);
         instantaneousErrorAccelerationSolution.setElement(element, trackingStiffness.getValue() * position + trackingDamping.getValue() * velocity);
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
      return desiredPositionState;
   }

   @Override
   public FrameVector3DReadOnly getVelocity()
   {
      return desiredVelocityState;
   }

   @Override
   public FrameVector3DReadOnly getAcceleration()
   {
      return desiredAccelerationState;
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
