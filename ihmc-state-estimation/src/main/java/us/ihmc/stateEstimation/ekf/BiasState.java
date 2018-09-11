package us.ihmc.stateEstimation.ekf;

import java.util.ArrayList;
import java.util.List;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class BiasState extends State
{
   private static final int size = 3;

   private final DenseMatrix64F bias = new DenseMatrix64F(size, 1);
   private final List<YoDouble> yoState = new ArrayList<>();
   private final DoubleProvider variance;

   private final double sqrtHz;

   public BiasState(String prefix, double dt, YoVariableRegistry registry)
   {
      this.sqrtHz = 1.0 / Math.sqrt(dt);

      for (int i = 0; i < size; i++)
      {
         yoState.add(new YoDouble(prefix + "Bias" + i, registry));
      }
      variance = new DoubleParameter(prefix + "BiasVariance", registry, 1.0);
   }

   public double getBias(int index)
   {
      return bias.get(index);
   }

   @Override
   public void setStateVector(DenseMatrix64F newState)
   {
      FilterTools.checkVectorDimensions(newState, bias);
      bias.set(newState);

      for (int i = 0; i < size; i++)
      {
         yoState.get(i).set(bias.get(i));
      }
   }

   @Override
   public void getStateVector(DenseMatrix64F vectorToPack)
   {
      vectorToPack.set(bias);
   }

   @Override
   public int getSize()
   {
      return size;
   }

   @Override
   public void predict()
   {
   }

   @Override
   public void getFMatrix(DenseMatrix64F matrixToPack)
   {
      matrixToPack.reshape(size, size);
      CommonOps.setIdentity(matrixToPack);
   }

   @Override
   public void getQMatrix(DenseMatrix64F matrixToPack)
   {
      matrixToPack.reshape(size, size);
      CommonOps.setIdentity(matrixToPack);
      CommonOps.scale(variance.getValue() * sqrtHz, matrixToPack);
   }

   public void reset()
   {
      for (int i = 0; i < size; i++)
      {
         bias.set(i, 0.0);
         yoState.get(i).set(0.0);
      }
   }

}
