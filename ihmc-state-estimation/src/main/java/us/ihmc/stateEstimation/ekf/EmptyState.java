package us.ihmc.stateEstimation.ekf;

import org.ejml.data.DenseMatrix64F;

public class EmptyState extends State
{
   @Override
   public void setStateVector(DenseMatrix64F newState)
   {
   }

   @Override
   public void getStateVector(DenseMatrix64F vectorToPack)
   {
      vectorToPack.reshape(0, 1);
   }

   @Override
   public int getSize()
   {
      return 0;
   }

   @Override
   public void predict()
   {
   }

   @Override
   public void getFMatrix(DenseMatrix64F matrixToPack)
   {
      matrixToPack.reshape(0, 0);
   }

   @Override
   public void getQMatrix(DenseMatrix64F matrixToPack)
   {
      matrixToPack.reshape(0, 0);
   }

}
