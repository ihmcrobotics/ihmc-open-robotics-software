package us.ihmc.sensorProcessing.stateEstimation.measurmentModelElements;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Set;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.controlFlow.ControlFlowInputPort;
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

   public AbstractMeasurementModelElement(int covarianceMatrixSize, String name, YoVariableRegistry registry)
   {
      measurementCovarianceMatrixBlock = new DenseMatrix64F(covarianceMatrixSize, covarianceMatrixSize);
      scaledMeasurementCovarianceMatrixBlock = new DenseMatrix64F(covarianceMatrixSize, covarianceMatrixSize);
      outputMatrixBlocks = new LinkedHashMap<ControlFlowOutputPort<?>, DenseMatrix64F>();
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

   public void setCovarianceMatrixScaling(double covarianceMatrixScaling)
   {
      this.covarianceMatrixScaling.set(covarianceMatrixScaling);
   }

   public void setNoiseCovariance(DenseMatrix64F measurementNoiseCovariance)
   {
      measurementCovarianceMatrixBlock.set(measurementNoiseCovariance);
   }

   public Set<ControlFlowOutputPort<?>> getStatePorts()
   {
      return outputMatrixBlocks.keySet();
   }

}
