package us.ihmc.stateEstimation.ekf;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commons.MathTools;
import us.ihmc.ekf.filter.FilterTools;
import us.ihmc.ekf.filter.RobotState;
import us.ihmc.ekf.filter.sensor.implementations.JointPositionSensor;
import us.ihmc.ekf.filter.state.implementations.JointState;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class JointPositionSensorWithBacklash extends JointPositionSensor
{
   private final String jointName;
   private final double dt;

   private double previousVelocity;
   private int ticksWithoutSignChange;

   private final DoubleParameter slopTime;
   private final DoubleParameter maxVarianceMultiplier;
   private final YoDouble currentVarianceMultiplier;

   public JointPositionSensorWithBacklash(String jointName, String parameterGroup, double dt, YoVariableRegistry registry)
   {
      super(jointName, parameterGroup, dt, registry);
      this.dt = dt;
      this.jointName = jointName;
      String prefix = FilterTools.stringToPrefix(jointName);
      maxVarianceMultiplier = FilterTools.findOrCreate(parameterGroup + "MaxVarianceMultiplier", registry, 1.0);
      slopTime = FilterTools.findOrCreate(parameterGroup + "SlopTime", registry, 0.03);
      currentVarianceMultiplier = new YoDouble(prefix + "CurrentVarianceMultiplier", registry);
   }

   @Override
   public void getRobotJacobianAndResidual(DenseMatrix64F jacobianToPack, DenseMatrix64F residualToPack, RobotState robotState)
   {
      super.getRobotJacobianAndResidual(jacobianToPack, residualToPack, robotState);

      JointState jointState = robotState.getJointState(jointName);
      double currentVelocity = jointState.getQd();

      if (Math.signum(previousVelocity) != Math.signum(currentVelocity))
      {
         ticksWithoutSignChange = 0;
      }
      else
      {
         ticksWithoutSignChange++;
      }

      double timeWithoutSlop = dt * ticksWithoutSignChange;
      double percentInSlop = MathTools.clamp(timeWithoutSlop / slopTime.getValue(), 0.0, 1.0);
      currentVarianceMultiplier.set((maxVarianceMultiplier.getValue() - 1.0) * (1.0 - percentInSlop) + 1.0);

      previousVelocity = currentVelocity;
   }

   // TODO: pass robot state here.
   @Override
   public void getRMatrix(DenseMatrix64F matrixToPack)
   {
      super.getRMatrix(matrixToPack);
      CommonOps.scale(currentVarianceMultiplier.getValue(), matrixToPack);
   }
}
