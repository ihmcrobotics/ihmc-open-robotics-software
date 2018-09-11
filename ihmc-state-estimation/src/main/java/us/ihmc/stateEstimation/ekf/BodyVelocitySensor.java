package us.ihmc.stateEstimation.ekf;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.screwTheory.GeometricJacobianCalculator;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

/**
 * Provides the common functionality used in the {@link LinearVelocitySensor} and {@link AngularVelocitySensor}. Both those sensors work
 * extremely similar as they both use the robot Jacobian to fill the measurement matrix {@code H} that relates the measurement to the
 * robot state via {@code z = H * x + v} where {@code z} is the measurement, {@code x} is the robot state, and {@code v} is the measurement
 * noise with covariance matrix {@code R}.
 * <p>
 * The only difference between the {@link LinearVelocitySensor} and {@link AngularVelocitySensor} is that one uses the linear part of the
 * robot Jacobian matrix (rows 3-6) and the other one uses the angular part (rows 0-3). As a result both those sensors only provide the
 * row offset in the Jacobian matrix.
 * </p>
 *
 * @author Georg
 *
 */
public abstract class BodyVelocitySensor extends Sensor
{
   private static final int measurementSize = 3;

   private final FrameVector3D measurement;
   private final BiasState biasState;
   private final DoubleProvider variance;

   private final DenseMatrix64F jacobianMatrix = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F jacobianRelevantPart = new DenseMatrix64F(0, 0);
   private final GeometricJacobianCalculator robotJacobian = new GeometricJacobianCalculator();
   private final List<String> oneDofJointNames = new ArrayList<>();

   private final DenseMatrix64F tempRobotState = new DenseMatrix64F(0, 0);

   private final double sqrtHz;

   public BodyVelocitySensor(String sensorName, double dt, RigidBody body, ReferenceFrame measurementFrame, boolean estimateBias, YoVariableRegistry registry)
   {
      this.sqrtHz = 1.0 / Math.sqrt(dt);

      measurement = new FrameVector3D(measurementFrame);
      robotJacobian.setKinematicChain(ScrewTools.getRootBody(body), body);
      robotJacobian.setJacobianFrame(measurementFrame);
      List<OneDoFJoint> oneDofJoints = ScrewTools.filterJoints(robotJacobian.getJointsFromBaseToEndEffector(), OneDoFJoint.class);
      oneDofJoints.stream().forEach(joint -> oneDofJointNames.add(joint.getName()));
      variance = new DoubleParameter(sensorName + "Variance", registry, 1.0);

      if (estimateBias)
      {
         biasState = new BiasState(sensorName, dt, registry);
      }
      else
      {
         biasState = null;
      }

      int degreesOfFreedom = robotJacobian.getNumberOfDegreesOfFreedom();
      jacobianRelevantPart.reshape(measurementSize, degreesOfFreedom);
   }

   protected abstract void packRelevantJacobianPart(DenseMatrix64F relevantPartToPack, DenseMatrix64F fullJacobian);

   @Override
   public State getSensorState()
   {
      return biasState == null ? super.getSensorState() : biasState;
   }

   @Override
   public void getSensorJacobian(DenseMatrix64F jacobianToPack)
   {
      if (biasState == null)
      {
         super.getSensorJacobian(jacobianToPack);
      }
      else
      {
         jacobianToPack.reshape(biasState.getSize(), biasState.getSize());
         CommonOps.setIdentity(jacobianToPack);
      }
   }

   @Override
   public int getMeasurementSize()
   {
      return measurementSize;
   }

   @Override
   public void getRobotJacobianAndResidual(DenseMatrix64F jacobianToPack, DenseMatrix64F residualToPack, RobotState robotState)
   {
      robotJacobian.computeJacobianMatrix();
      robotJacobian.getJacobianMatrix(jacobianMatrix);

      packRelevantJacobianPart(jacobianRelevantPart, jacobianMatrix);
      FilterTools.insertForVelocity(jacobianToPack, oneDofJointNames, jacobianRelevantPart, robotState);

      // Compute the sensor measurement based on the robot state:
      residualToPack.reshape(measurementSize, 1);
      robotState.getStateVector(tempRobotState);
      CommonOps.mult(jacobianToPack, tempRobotState, residualToPack);

      // Compute the residual considering the sensor bias and the current measurement:
      residualToPack.set(0, measurement.getX() - residualToPack.get(0));
      residualToPack.set(1, measurement.getY() - residualToPack.get(1));
      residualToPack.set(2, measurement.getZ() - residualToPack.get(2));

      if (biasState != null)
      {
         residualToPack.set(0, residualToPack.get(0) - biasState.getBias(0));
         residualToPack.set(1, residualToPack.get(1) - biasState.getBias(1));
         residualToPack.set(2, residualToPack.get(2) - biasState.getBias(2));
      }
   }

   @Override
   public void getRMatrix(DenseMatrix64F matrixToPack)
   {
      matrixToPack.reshape(measurementSize, measurementSize);
      CommonOps.setIdentity(matrixToPack);
      CommonOps.scale(variance.getValue() * sqrtHz, matrixToPack);
   }

   public void setMeasurement(Vector3DReadOnly measurement)
   {
      this.measurement.set(measurement);
   }

   public void reset()
   {
      if (biasState != null)
      {
         biasState.reset();
      }
   }
}
