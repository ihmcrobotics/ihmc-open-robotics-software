package us.ihmc.sensorProcessing.stateEstimation.measurementModelElements;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;


public abstract class AbstractMeasurementModelElement implements MeasurementModelElement
{
   private final ArrayList<ControlFlowOutputPort<?>> controlFlowOutputPortList;
   private final Map<ControlFlowOutputPort<?>, DenseMatrix64F> outputMatrixBlocks;
   private final DenseMatrix64F measurementCovarianceMatrixBlock;
   private final DenseMatrix64F scaledMeasurementCovarianceMatrixBlock;
   private final DoubleYoVariable covarianceMatrixScaling;

   public AbstractMeasurementModelElement(int covarianceMatrixSize, String name, YoVariableRegistry registry)
   {
      measurementCovarianceMatrixBlock = new DenseMatrix64F(covarianceMatrixSize, covarianceMatrixSize);
      scaledMeasurementCovarianceMatrixBlock = new DenseMatrix64F(covarianceMatrixSize, covarianceMatrixSize);
      controlFlowOutputPortList = new ArrayList<ControlFlowOutputPort<?>>();
      outputMatrixBlocks = new LinkedHashMap<ControlFlowOutputPort<?>, DenseMatrix64F>();
      covarianceMatrixScaling = new DoubleYoVariable(name + "CovScaling", registry);
      covarianceMatrixScaling.set(1.0);
   }

   protected final void initialize(int outputMatriBlockSize, ControlFlowOutputPort<?> ...controlFlowOutputPorts)
   {
      for (ControlFlowOutputPort<?> controlFlowOutputPort : controlFlowOutputPorts)
      {
         controlFlowOutputPortList.add(controlFlowOutputPort);
      }
      
      populateOutputMatrixMap(outputMatriBlockSize);
   }
   
   private void populateOutputMatrixMap(int matrixSize)
   {
      for (ControlFlowOutputPort<?> controlFlowOutputPort : controlFlowOutputPortList)
      {
         outputMatrixBlocks.put(controlFlowOutputPort, new DenseMatrix64F(matrixSize, matrixSize));
      }
   }
   
   public final DenseMatrix64F getOutputMatrixBlock(ControlFlowOutputPort<?> statePort)
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

   public final ArrayList<ControlFlowOutputPort<?>> getStatePorts()
   {
      return controlFlowOutputPortList;
   }

}
