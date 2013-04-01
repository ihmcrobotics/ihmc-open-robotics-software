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
import us.ihmc.kalman.YoKalmanFilter;
import us.ihmc.utilities.linearDynamicSystems.StateSpaceSystemDiscretizer;
import us.ihmc.utilities.math.MatrixTools;

import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class ComposableStateEstimator extends AbstractControlFlowElement
{
   protected final YoVariableRegistry registry;
   protected ComposableStateEstimatorKalmanFilter kalmanFilter;
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


   public ComposableStateEstimator(String name, double controlDT, YoVariableRegistry parentRegistry)
   {
      this.registry = new YoVariableRegistry(name);
      this.controlDT = controlDT;
      parentRegistry.addChild(registry);
   }

   public void startComputation()
   {
      kalmanFilter.configure();
      kalmanFilter.predict(null);
      kalmanFilter.update(null);
   }

   public <T> void registerStatePort(ControlFlowOutputPort<T> statePort, int size)
   {
      statePorts.add(statePort);
      stateSizes.put(statePort, size);
      registerOutputPort(statePort);
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
      int stateSize = MatrixTools.computeIndicesIntoVector(statePorts, stateStartIndices, stateSizes);
      int inputSize = MatrixTools.computeIndicesIntoVector(processInputPorts, processInputStartIndices, processInputSizes);
      int measurementSize = MatrixTools.computeIndicesIntoVector(measurementInputPorts, measurementStartIndices, measurementSizes);

      kalmanFilter = new ComposableStateEstimatorKalmanFilter(stateSize, inputSize, measurementSize);

      kalmanFilter.configure();
      initializeState();
   }

   private void initializeState()
   {
      DenseMatrix64F x = kalmanFilter.getState();
      kalmanFilter.computeSteadyStateGainAndCovariance(50);    // TODO: magic number
      DenseMatrix64F P = kalmanFilter.getCovariance();
      CommonOps.scale(10.0, P);    // TODO: magic number
      kalmanFilter.setState(x, P);
   }

   public void waitUntilComputationIsDone()
   {
      // empty
   }

   protected class ComposableStateEstimatorKalmanFilter extends YoKalmanFilter
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
         super(ComposableStateEstimatorKalmanFilter.class.getSimpleName(), stateSize, inputSize, measurementSize, ComposableStateEstimator.this.registry);
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
         updateProcessModel();
         updateMeasurementModel();
         discretizer.discretize(F, G, Q, controlDT);
         super.configure(F, G, H);
         setProcessNoiseCovariance(Q);
         setMeasurementNoiseCovariance(R);
      }

      @Override
      protected void updateAPrioriState(DenseMatrix64F x, DenseMatrix64F u)
      {
         for (ProcessModelElement processModelElement : processModelElements.values())
         {
            processModelElement.propagateState(controlDT);    // should update what's in the output ports
         }
      }

      @Override
      protected void updateAPosterioriState(DenseMatrix64F x, DenseMatrix64F y, DenseMatrix64F K)
      {
         for (ControlFlowInputPort<?> measurementInputPort : measurementInputPorts)
         {
            MeasurementModelElement measurementModelElement = measurementModelElements.get(measurementInputPort);

            DenseMatrix64F residualBlock = measurementModelElement.computeResidual();
            MatrixTools.insertVectorBlock(residual, residualBlock, measurementInputPort, measurementStartIndices);
         }

         if (residual.getNumRows() > 0)
            MatrixVectorMult.mult(K, residual, correction);

         for (ControlFlowOutputPort<?> statePort : statePorts)
         {
            ProcessModelElement processModelElement = processModelElements.get(statePort);
            MatrixTools.extractVectorBlock(correctionBlock, correction, statePort, stateStartIndices, stateSizes);

            processModelElement.correctState(correctionBlock);
         }
      }

      private void updateProcessModel()
      {
         F.zero();
         G.zero();
         Q.zero();
         
         for (ProcessModelElement processModelElement : processModelElements.values())
         {
            processModelElement.computeMatrixBlocks();
         }
         
         for (ControlFlowOutputPort<?> rowPort : stateStartIndices.keySet())
         {
            ProcessModelElement processModelElement = processModelElements.get(rowPort);
         
            for (ControlFlowOutputPort<?> columnPort : statePorts)
            {
               DenseMatrix64F stateMatrixBlock = processModelElement.getStateMatrixBlock(columnPort);
               MatrixTools.insertMatrixBlock(F, stateMatrixBlock, rowPort, stateStartIndices, columnPort, stateStartIndices);
            }
         
            for (ControlFlowInputPort<?> inputPort : processInputPorts)
            {
               DenseMatrix64F inputMatrixBlock = processModelElement.getInputMatrixBlock(inputPort);
               MatrixTools.insertMatrixBlock(G, inputMatrixBlock, rowPort, stateStartIndices, inputPort, processInputStartIndices);
            }
         
            DenseMatrix64F processCovarianceMatrixBlock = processModelElement.getProcessCovarianceMatrixBlock();
            MatrixTools.insertMatrixBlock(Q, processCovarianceMatrixBlock, rowPort, stateStartIndices, rowPort, stateStartIndices);
         }
      }

      private void updateMeasurementModel()
      {
         H.zero();
         R.zero();

         for (ControlFlowInputPort<?> measurementInputPort : measurementInputPorts)
         {
            MeasurementModelElement measurementModelElement = measurementModelElements.get(measurementInputPort);
            measurementModelElement.computeMatrixBlocks();

            for (ControlFlowOutputPort<?> statePort : statePorts)
            {
               DenseMatrix64F outputMatrixBlock = measurementModelElement.getOutputMatrixBlock(statePort);
               MatrixTools.insertMatrixBlock(H, outputMatrixBlock, measurementInputPort, measurementStartIndices, statePort, stateStartIndices);
            }

            DenseMatrix64F measurementCovarianceMatrixBlock = measurementModelElement.getMeasurementCovarianceMatrixBlock();
            MatrixTools.insertMatrixBlock(R, measurementCovarianceMatrixBlock, measurementInputPort, measurementStartIndices, measurementInputPort,
                              measurementStartIndices);
         }
      }
   }
}
