package us.ihmc.stateEstimation.ekf;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class JointState extends State
{
   public static final int size = 3;

   private final String jointName;

   private final DenseMatrix64F stateVector = new DenseMatrix64F(size, 1);
   private final DenseMatrix64F tempStateVector = new DenseMatrix64F(size, 1);
   private final DenseMatrix64F F = new DenseMatrix64F(size, size);

   private final DoubleProvider accelerationVariance;

   private final double sqrtHz;

   public JointState(String jointName, double dt, YoVariableRegistry registry)
   {
      this.jointName = jointName;
      this.sqrtHz = 1.0 / Math.sqrt(dt);

      CommonOps.setIdentity(F);
      F.set(0, 1, dt);
      F.set(0, 2, 0.5 * dt * dt);
      F.set(1, 2, dt);

      accelerationVariance = new DoubleParameter(FilterTools.stringToPrefix(jointName) + "AccelerationVariance", registry, 1.0);
   }

   public void initialize(double initialPosition, double initialVelocity)
   {
      stateVector.set(0, initialPosition);
      stateVector.set(1, initialVelocity);
      stateVector.set(2, 0.0);
   }

   public String getJointName()
   {
      return jointName;
   }

   @Override
   public void setStateVector(DenseMatrix64F newState)
   {
      FilterTools.checkVectorDimensions(newState, stateVector);
      System.arraycopy(newState.data, 0, stateVector.data, 0, getSize());
   }

   @Override
   public void getStateVector(DenseMatrix64F vectorToPack)
   {
      vectorToPack.set(stateVector);
   }

   @Override
   public int getSize()
   {
      return size;
   }

   @Override
   public void predict()
   {
      tempStateVector.set(stateVector);
      CommonOps.mult(F, tempStateVector, stateVector);
   }

   @Override
   public void getFMatrix(DenseMatrix64F matrixToPack)
   {
      matrixToPack.set(F);
   }

   @Override
   public void getQMatrix(DenseMatrix64F matrixToPack)
   {
      matrixToPack.reshape(size, size);
      CommonOps.fill(matrixToPack, 0.0);
      matrixToPack.set(2, 2, accelerationVariance.getValue() * sqrtHz);
   }

   public double getQ()
   {
      return stateVector.get(0);
   }

   public double getQd()
   {
      return stateVector.get(1);
   }

   public double getQdd()
   {
      return stateVector.get(2);
   }
}
