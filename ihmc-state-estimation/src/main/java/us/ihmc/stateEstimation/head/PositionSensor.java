package us.ihmc.stateEstimation.head;

import org.ejml.data.DMatrix1Row;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;

import us.ihmc.ekf.filter.FilterTools;
import us.ihmc.ekf.filter.RobotState;
import us.ihmc.ekf.filter.sensor.Sensor;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * A simple implementation of a body position sensor for the EKF framework.
 * <p>
 * Note, that this sensor assumes that it measures the position of the root joint directly. There is
 * currently no offset measurement frame or position sensor for other parts of the robot. This should
 * be added before this is moved to the EKF package.
 *
 * @author Georg Wiedebach
 *
 */
public class PositionSensor extends Sensor
{
   private final String sensorName;

   private final double sqrtHz;
   private final DoubleProvider variance;

   private final DMatrixRMaj stateVector = new DMatrixRMaj(0, 0);
   private final Point3D measurement = new Point3D();

   public PositionSensor(String sensorName, double dt, YoRegistry registry)
   {
      this.sensorName = sensorName;
      sqrtHz = 1.0 / Math.sqrt(dt);
      variance = FilterTools.findOrCreate(sensorName + "Variance", registry, 1.0);
   }

   public void setMeasurement(Point3DReadOnly position)
   {
      measurement.set(position);
   }

   @Override
   public String getName()
   {
      return sensorName;
   }

   @Override
   public int getMeasurementSize()
   {
      return 3;
   }

   @Override
   public void getMeasurementJacobian(DMatrix1Row jacobianToPack, RobotState robotState)
   {
      // Could use this to also correct the joint angles. For now only use for base orientation.
      if (!robotState.isFloating())
         throw new RuntimeException("This sensor is currently only supported for floating robots.");

      jacobianToPack.reshape(getMeasurementSize(), robotState.getSize());
      CommonOps_DDRM.fill(jacobianToPack, 0.0);

      jacobianToPack.set(0, robotState.findPositionIndex() + 0, 1.0);
      jacobianToPack.set(1, robotState.findPositionIndex() + 1, 1.0);
      jacobianToPack.set(2, robotState.findPositionIndex() + 2, 1.0);
   }

   @Override
   public void getResidual(DMatrix1Row residualToPack, RobotState robotState)
   {
      robotState.getStateVector(stateVector);
      residualToPack.reshape(getMeasurementSize(), 1);
      residualToPack.set(0, measurement.getX() - stateVector.get(robotState.findPositionIndex() + 0));
      residualToPack.set(1, measurement.getY() - stateVector.get(robotState.findPositionIndex() + 1));
      residualToPack.set(2, measurement.getZ() - stateVector.get(robotState.findPositionIndex() + 2));
   }

   @Override
   public void getRMatrix(DMatrix1Row noiseCovarianceToPack)
   {
      noiseCovarianceToPack.reshape(getMeasurementSize(), getMeasurementSize());
      CommonOps_DDRM.setIdentity(noiseCovarianceToPack);
      CommonOps_DDRM.scale(variance.getValue() * sqrtHz, noiseCovarianceToPack);
   }
}
