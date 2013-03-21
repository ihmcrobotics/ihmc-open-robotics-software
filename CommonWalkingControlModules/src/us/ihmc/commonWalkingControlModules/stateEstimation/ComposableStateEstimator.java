package us.ihmc.commonWalkingControlModules.stateEstimation;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.ejml.alg.dense.mult.MatrixVectorMult;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.controlFlow.AbstractControlFlowElement;
import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.controlFlow.ControlFlowPort;
import us.ihmc.kalman.YoKalmanFilter;
import us.ihmc.utilities.linearDynamicSystems.StateSpaceSystemDiscretizer;

import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class ComposableStateEstimator extends AbstractControlFlowElement
{
   private final String name;
   private final YoVariableRegistry registry;
   private ComposableStateEstimatorKalmanFilter kalmanFilter;
   private final double controlDT;

   // model elements
   private final Map<ControlFlowOutputPort<?>, ProcessModelElement> processModelElements = new HashMap<ControlFlowOutputPort<?>, ProcessModelElement>();
   private final Map<ControlFlowInputPort<?>, MeasurementModelElement> measurementModelElements = new HashMap<ControlFlowInputPort<?>,
                                                                                                     MeasurementModelElement>();

   // states
   private final List<ControlFlowOutputPort<?>> statePorts = new ArrayList<ControlFlowOutputPort<?>>();
   private final Map<ControlFlowOutputPort<?>, Integer> stateStartIndices = new HashMap<ControlFlowOutputPort<?>, Integer>();
   private final Map<ControlFlowOutputPort<?>, Integer> stateSizes = new HashMap<ControlFlowOutputPort<?>, Integer>();

   // measurements
   private final List<ControlFlowInputPort<?>> measurementInputPorts = new ArrayList<ControlFlowInputPort<?>>();
   private final Map<ControlFlowInputPort<?>, Integer> measurementStartIndices = new HashMap<ControlFlowInputPort<?>, Integer>();
   private final Map<ControlFlowInputPort<?>, Integer> measurementSizes = new HashMap<ControlFlowInputPort<?>, Integer>();

   // inputs
   private final List<ControlFlowInputPort<?>> processInputPorts = new ArrayList<ControlFlowInputPort<?>>();
   private final Map<ControlFlowInputPort<?>, Integer> processInputStartIndices = new HashMap<ControlFlowInputPort<?>, Integer>();
   private final Map<ControlFlowInputPort<?>, Integer> processInputSizes = new HashMap<ControlFlowInputPort<?>, Integer>();


   public ComposableStateEstimator(String name, double controlDT)
   {
      this.name = name;
      this.registry = new YoVariableRegistry(name);
      this.controlDT = controlDT;
   }

   public void startComputation()
   {
      kalmanFilter.configure();
      kalmanFilter.predict(null);
      kalmanFilter.update(null);
   }

   public <T> ControlFlowOutputPort<T> createStatePort(int size)
   {
      ControlFlowOutputPort<T> statePort = createOutputPort();
      statePorts.add(statePort);
      stateSizes.put(statePort, size);

      return statePort;
   }

   public <T> ControlFlowInputPort<T> createProcessInputPort(int size)
   {
      ControlFlowInputPort<T> processInputPort = createInputPort();
      processInputPorts.add(processInputPort);
      processInputSizes.put(processInputPort, size);

      return processInputPort;
   }

   public <T> ControlFlowInputPort<T> createMeasurementInputPort(int size)
   {
      ControlFlowInputPort<T> measurementInputPort = createInputPort();
      measurementInputPorts.add(measurementInputPort);
      measurementSizes.put(measurementInputPort, size);

      return measurementInputPort;
   }

   public void addProcessModelElement(ControlFlowOutputPort<?> statePort, ProcessModelElement processModelElement)
   {
      processModelElements.put(statePort, processModelElement);
   }

   public void addMeasurementModelElement(ControlFlowInputPort<?> measurementPort, MeasurementModelElement measurementModelElement)
   {
      measurementModelElements.put(measurementPort, measurementModelElement);
   }

   public void initialize()
   {
      int stateSize = computeIndicesIntoVector(statePorts, stateStartIndices, stateSizes);
      int inputSize = computeIndicesIntoVector(processInputPorts, processInputStartIndices, processInputSizes);
      int measurementSize = computeIndicesIntoVector(measurementInputPorts, measurementStartIndices, measurementSizes);

      kalmanFilter = new ComposableStateEstimatorKalmanFilter(stateSize, inputSize, measurementSize);
      initializeState();
   }
   
   private void initializeState()
   {
      // TODO Auto-generated method stub
   }

   public void waitUntilComputationIsDone()
   {
      // empty
   }

   private static void insertMatrixBlock(DenseMatrix64F bigMatrix, DenseMatrix64F matrixBlock, ControlFlowPort<?> rowPort,
           Map<? extends ControlFlowPort<?>, Integer> rowStartIndices, ControlFlowPort<?> columnPort,
           Map<? extends ControlFlowPort<?>, Integer> columnStartIndices)
   {
      if (matrixBlock != null)
      {
         int rowStartIndex = rowStartIndices.get(rowPort);
         int columnStartIndex = columnStartIndices.get(columnPort);
         CommonOps.insert(matrixBlock, bigMatrix, rowStartIndex, columnStartIndex);
      }
   }

   private void insertStateVectorBlock(DenseMatrix64F bigVector, DenseMatrix64F vectorBlock, ControlFlowPort<?> rowPort)
   {
      if (vectorBlock != null)
      {
         int rowStartIndex = stateStartIndices.get(rowPort);
         CommonOps.insert(vectorBlock, bigVector, rowStartIndex, 0);
      }
   }

   private void extractStateVectorBlock(DenseMatrix64F vectorBlockToPack, DenseMatrix64F bigVector, ControlFlowOutputPort<?> rowPort)
   {
      int rowStartIndex = stateStartIndices.get(rowPort);
      int stateSize = stateSizes.get(rowPort);
      CommonOps.extract(bigVector, rowStartIndex, rowStartIndex + stateSize, 0, 1, vectorBlockToPack, 0, 0);
   }

   private static <T extends ControlFlowPort<?>> int computeIndicesIntoVector(List<T> ports, Map<T, Integer> indices, Map<T, Integer> sizes)
   {
      int stateStartIndex = 0;

      for (T port : ports)
      {
         indices.put(port, stateStartIndex);
         stateStartIndex += sizes.get(port);
      }

      return stateStartIndex;
   }

   private class ComposableStateEstimatorKalmanFilter extends YoKalmanFilter
   {
      // correction
      private final DenseMatrix64F residual;
      private final DenseMatrix64F correction;
      private final DenseMatrix64F correctionBlock = new DenseMatrix64F(1, 1);

      // discretization
      private final StateSpaceSystemDiscretizer discretizer;
      private final DenseMatrix64F F;
      private final DenseMatrix64F G;
      private final DenseMatrix64F H;
      private final DenseMatrix64F Q;
      private final DenseMatrix64F R;

      public ComposableStateEstimatorKalmanFilter(int stateSize, int inputSize, int measurementSize)
      {
         super(name, stateSize, inputSize, measurementSize, ComposableStateEstimator.this.registry);
         this.correction = new DenseMatrix64F(stateSize, 1);
         this.residual = new DenseMatrix64F(measurementSize, 1);

         this.discretizer = new StateSpaceSystemDiscretizer(stateSize, inputSize);
         this.F = new DenseMatrix64F(stateSize, stateSize);
         this.G = new DenseMatrix64F(stateSize, inputSize);
         this.H = new DenseMatrix64F(measurementSize, stateSize);
         this.Q = new DenseMatrix64F(stateSize, stateSize);
         this.R = new DenseMatrix64F(measurementSize, measurementSize);
      }

      protected void configure()
      {
         updateContinuousTimeModel();
         discretizer.discretize(F, G, Q, R, controlDT);
         configure(F, G, H);
         setProcessNoiseCovariance(Q);
         setMeasurementNoiseCovariance(R);
      }

      @Override
      protected void updateAPrioriState(DenseMatrix64F x, DenseMatrix64F u)
      {
         for (ProcessModelElement processModelElement : processModelElements.values())
         {
            processModelElement.propagateState();    // should update what's in the output ports
         }
      }

      @Override
      protected void updateAPosterioriState(DenseMatrix64F x, DenseMatrix64F y, DenseMatrix64F K)
      {
         for (ControlFlowInputPort<?> measurementInputPort : measurementInputPorts)
         {
            MeasurementModelElement measurementModelElement = measurementModelElements.get(measurementInputPort);

            DenseMatrix64F residualBlock = measurementModelElement.computeResidual();
            insertStateVectorBlock(residual, residualBlock, measurementInputPort);
         }

         if (residual.getNumRows() > 0)
            MatrixVectorMult.mult(K, residual, correction);

         for (ControlFlowOutputPort<?> statePort : statePorts)
         {
            ProcessModelElement processModelElement = processModelElements.get(statePort);
            extractStateVectorBlock(correctionBlock, correction, statePort);

            processModelElement.correctState(correctionBlock);
         }
      }

      private void updateContinuousTimeModel()
      {
         updateProcessModel();
         updateMeasurementModel();
      }

      private void updateProcessModel()
      {
         for (ProcessModelElement processModelElement : processModelElements.values())
         {
            processModelElement.computeMatrixBlocks();
         }

         for (ControlFlowOutputPort<?> rowPort : statePorts)
         {
            ProcessModelElement processModelElement = processModelElements.get(rowPort);

            for (ControlFlowOutputPort<?> columnPort : statePorts)
            {
               DenseMatrix64F stateMatrixBlock = processModelElement.getStateMatrixBlock(columnPort);
               insertMatrixBlock(F, stateMatrixBlock, rowPort, stateStartIndices, columnPort, stateStartIndices);
            }

            for (ControlFlowInputPort<?> inputPort : processInputPorts)
            {
               DenseMatrix64F inputMatrixBlock = processModelElement.getInputMatrixBlock(inputPort);
               insertMatrixBlock(G, inputMatrixBlock, rowPort, stateStartIndices, inputPort, processInputStartIndices);
            }

            DenseMatrix64F processCovarianceMatrixBlock = processModelElement.getProcessCovarianceMatrixBlock();
            insertMatrixBlock(Q, processCovarianceMatrixBlock, rowPort, stateStartIndices, rowPort, stateStartIndices);
         }
      }

      private void updateMeasurementModel()
      {
         for (ControlFlowInputPort<?> measurementInputPort : measurementInputPorts)
         {
            MeasurementModelElement measurementModelElement = measurementModelElements.get(measurementInputPort);
            measurementModelElement.computeMatrixBlocks();

            for (ControlFlowOutputPort<?> statePort : statePorts)
            {
               DenseMatrix64F outputMatrixBlock = measurementModelElement.getOutputMatrixBlock(statePort);
               insertMatrixBlock(H, outputMatrixBlock, measurementInputPort, measurementStartIndices, statePort, stateStartIndices);
            }

            DenseMatrix64F measurementCovarianceMatrixBlock = measurementModelElement.getMeasurementCovarianceMatrixBlock();
            insertMatrixBlock(R, measurementCovarianceMatrixBlock, measurementInputPort, measurementStartIndices, measurementInputPort,
                              measurementStartIndices);
         }
      }
   }

}
