package us.ihmc.commonWalkingControlModules.stateEstimation.measurementModelElements;

import java.util.HashMap;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.stateEstimation.MeasurementModelElement;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.utilities.math.MathTools;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public abstract class AbstractMeasurementModelElement implements MeasurementModelElement
{
   protected final Map<ControlFlowOutputPort<?>, DenseMatrix64F> outputMatrixBlocks;
   private final DenseMatrix64F measurementCovarianceMatrixBlock;
   private final DenseMatrix64F scaledMeasurementCovarianceMatrixBlock;
   private final DoubleYoVariable covarianceMatrixScaling;

   public AbstractMeasurementModelElement(int covarianceMatrixSize, int numberOfOutputMatrixBlocks, String name, YoVariableRegistry registry)
   {
      measurementCovarianceMatrixBlock = new DenseMatrix64F(covarianceMatrixSize, covarianceMatrixSize);
      scaledMeasurementCovarianceMatrixBlock = new DenseMatrix64F(covarianceMatrixSize, covarianceMatrixSize);
      outputMatrixBlocks = new HashMap<ControlFlowOutputPort<?>, DenseMatrix64F>(numberOfOutputMatrixBlocks);
      covarianceMatrixScaling = new DoubleYoVariable(name + "CovScaling", registry);
      covarianceMatrixScaling.set(1.0);
   }

   public DenseMatrix64F getOutputMatrixBlock(ControlFlowOutputPort<?> statePort)
   {
      return outputMatrixBlocks.get(statePort);
   }

   public DenseMatrix64F getMeasurementCovarianceMatrixBlock()
   {
      scaledMeasurementCovarianceMatrixBlock.set(measurementCovarianceMatrixBlock);
      CommonOps.scale(covarianceMatrixScaling.getDoubleValue(), scaledMeasurementCovarianceMatrixBlock);

      return scaledMeasurementCovarianceMatrixBlock;
   }

   public void setNoiseStandardDeviation(double standardDeviation)
   {
      double variance = MathTools.square(standardDeviation);
      CommonOps.setIdentity(measurementCovarianceMatrixBlock);
      CommonOps.scale(variance, measurementCovarianceMatrixBlock);
   }
   
   public void setNoiseCovariance(DenseMatrix64F measurementNoiseCovariance)
   {
      measurementCovarianceMatrixBlock.set(measurementNoiseCovariance);
   }
}
