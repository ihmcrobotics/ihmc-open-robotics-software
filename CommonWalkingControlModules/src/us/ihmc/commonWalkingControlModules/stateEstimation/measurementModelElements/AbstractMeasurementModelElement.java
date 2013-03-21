package us.ihmc.commonWalkingControlModules.stateEstimation.measurementModelElements;

import java.util.HashMap;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.stateEstimation.MeasurementModelElement;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.utilities.math.MathTools;

public abstract class AbstractMeasurementModelElement implements MeasurementModelElement
{
   protected final Map<ControlFlowOutputPort<?>, DenseMatrix64F> outputMatrixBlocks;
   private final DenseMatrix64F measurementCovarianceMatrixBlock;

   public AbstractMeasurementModelElement(int covarianceMatrixSize, int numberOfOutputMatrixBlocks)
   {
      measurementCovarianceMatrixBlock = new DenseMatrix64F(covarianceMatrixSize, covarianceMatrixSize);
      outputMatrixBlocks = new HashMap<ControlFlowOutputPort<?>, DenseMatrix64F>(numberOfOutputMatrixBlocks);
   }

   public DenseMatrix64F getOutputMatrixBlock(ControlFlowOutputPort<?> statePort)
   {
      return outputMatrixBlocks.get(statePort);
   }

   public DenseMatrix64F getMeasurementCovarianceMatrixBlock()
   {
      return measurementCovarianceMatrixBlock;
   }

   public void setNoiseStandardDeviation(double standardDeviation)
   {
      double variance = MathTools.square(standardDeviation);
      CommonOps.setIdentity(measurementCovarianceMatrixBlock);
      CommonOps.scale(variance, measurementCovarianceMatrixBlock);
   }
}