package us.ihmc.stateEstimation.ekf;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang3.mutable.MutableInt;
import org.apache.commons.lang3.tuple.ImmutablePair;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

public class ComposedSensor extends Sensor
{
   private final List<ImmutablePair<MutableInt, Sensor>> subSensorList = new ArrayList<>();
   private final ComposedState sensorState = new ComposedState();

   private final DenseMatrix64F tempJacobian = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F tempResidual = new DenseMatrix64F(0, 0);
   private final int robotStateSize;

   private final DenseMatrix64F tempRobotJacobian = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F tempSensorJacobian = new DenseMatrix64F(0, 0);

   public ComposedSensor(List<Sensor> sensors, int robotStateSize)
   {
      this.robotStateSize = robotStateSize;

      for (Sensor sensor : sensors)
      {
         addSensor(sensor);
      }
   }

   public int addSensor(Sensor sensorToAdd)
   {
      int stateIndex = subSensorList.size();
      int oldSize = getMeasurementSize();
      subSensorList.add(new ImmutablePair<>(new MutableInt(oldSize), sensorToAdd));
      sensorState.addState(sensorToAdd.getSensorState());
      return stateIndex;
   }

   @Override
   public State getSensorState()
   {
      return sensorState;
   }

   @Override
   public int getMeasurementSize()
   {
      if (subSensorList.isEmpty())
      {
         return 0;
      }

      ImmutablePair<MutableInt, Sensor> lastSubState = subSensorList.get(subSensorList.size() - 1);
      return lastSubState.getLeft().intValue() + lastSubState.getRight().getMeasurementSize();
   }

   @Override
   public void getRobotJacobianAndResidual(DenseMatrix64F jacobianToPack, DenseMatrix64F residualToPack, RobotState robotState)
   {
      jacobianToPack.reshape(getMeasurementSize(), robotStateSize);
      CommonOps.fill(jacobianToPack, 0.0);

      residualToPack.reshape(getMeasurementSize(), 1);
      CommonOps.fill(residualToPack, 0.0);

      for (int i = 0; i < subSensorList.size(); i++)
      {
         ImmutablePair<MutableInt, Sensor> pair = subSensorList.get(i);
         int startIndex = pair.getLeft().intValue();
         Sensor subSensor = pair.getRight();

         subSensor.getRobotJacobianAndResidual(tempJacobian, tempResidual, robotState);
         CommonOps.insert(tempJacobian, jacobianToPack, startIndex, 0);
         CommonOps.insert(tempResidual, residualToPack, startIndex, 0);
      }
   }

   @Override
   public void getSensorJacobian(DenseMatrix64F jacobianToPack)
   {
      jacobianToPack.reshape(getMeasurementSize(), sensorState.getSize());
      CommonOps.fill(jacobianToPack, 0.0);

      for (int i = 0; i < subSensorList.size(); i++)
      {
         ImmutablePair<MutableInt, Sensor> pair = subSensorList.get(i);
         int startIndexMeasurement = pair.getLeft().intValue();
         Sensor subSensor = pair.getRight();
         int startIndexSensor = sensorState.getStartIndex(i);

         subSensor.getSensorJacobian(tempJacobian);
         CommonOps.insert(tempJacobian, jacobianToPack, startIndexMeasurement, startIndexSensor);
      }
   }

   @Override
   public void getRMatrix(DenseMatrix64F matrixToPack)
   {
      matrixToPack.reshape(getMeasurementSize(), getMeasurementSize());
      CommonOps.fill(matrixToPack, 0.0);

      for (int i = 0; i < subSensorList.size(); i++)
      {
         ImmutablePair<MutableInt, Sensor> pair = subSensorList.get(i);
         int startIndex = pair.getLeft().intValue();
         Sensor subSensor = pair.getRight();

         subSensor.getRMatrix(tempJacobian);
         CommonOps.insert(tempJacobian, matrixToPack, startIndex, startIndex);
      }
   }

   public void assembleFullJacobian(DenseMatrix64F matrixToPack, DenseMatrix64F residualToPack, RobotState robotState)
   {
      getRobotJacobianAndResidual(tempRobotJacobian, residualToPack, robotState);
      getSensorJacobian(tempSensorJacobian);
      matrixToPack.reshape(getMeasurementSize(), robotStateSize + sensorState.getSize());
      CommonOps.insert(tempRobotJacobian, matrixToPack, 0, 0);
      CommonOps.insert(tempSensorJacobian, matrixToPack, 0, robotStateSize);
   }

}
