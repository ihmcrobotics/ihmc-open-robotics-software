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
   private final List<Runnable> postStateChangeRunnables = new ArrayList<Runnable>();

   // model elements
   private final Map<ControlFlowOutputPort<?>, ProcessModelElement> processModelElements = new HashMap<ControlFlowOutputPort<?>, ProcessModelElement>();
   private final Map<ControlFlowInputPort<?>, MeasurementModelElement> measurementModelElements = new HashMap<ControlFlowInputPort<?>, MeasurementModelElement>();

   // states
   private final List<ControlFlowOutputPort<?>> continuousStatePorts = new ArrayList<ControlFlowOutputPort<?>>();
   private final Map<ControlFlowOutputPort<?>, Integer> continuousStateStartIndices = new HashMap<ControlFlowOutputPort<?>, Integer>();

   private final List<ControlFlowOutputPort<?>> discreteStatePorts = new ArrayList<ControlFlowOutputPort<?>>();
   private final Map<ControlFlowOutputPort<?>, Integer> discreteStateStartIndices = new HashMap<ControlFlowOutputPort<?>, Integer>();

   private final List<ControlFlowOutputPort<?>> allStatePorts = new ArrayList<ControlFlowOutputPort<?>>();
   private final Map<ControlFlowOutputPort<?>, Integer> allStateStartIndices = new HashMap<ControlFlowOutputPort<?>, Integer>();

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

   public <T> ControlFlowInputPort<T> createProcessInputPort(int size)
   {
      ControlFlowInputPort<T> processInputPort = createInputPort();
      processInputPorts.add(processInputPort);
      processInputSizes.put(processInputPort, size);

      return processInputPort;
   }

   public <T> void registerProcessInputPort(ControlFlowInputPort<T> processInputPort, int size)
   {
      registerPort(processInputPort);
      processInputPorts.add(processInputPort);
      processInputSizes.put(processInputPort, size);
   }
   
   
   public <T> ControlFlowInputPort<T> createMeasurementInputPort(int size)
   {
      ControlFlowInputPort<T> measurementInputPort = createInputPort();
      measurementInputPorts.add(measurementInputPort);
      measurementSizes.put(measurementInputPort, size);

      return measurementInputPort;
   }

   public void addProcessModelElement(ControlFlowOutputPort<?> statePort, ProcessModelElement processModelElement, int stateSize)
   {
      processModelElements.put(statePort, processModelElement);
      switch (processModelElement.getTimeDomain())
      {
      case CONTINUOUS:
         continuousStatePorts.add(statePort);
         break;

      case DISCRETE:
         discreteStatePorts.add(statePort);
         break;

      default:
         throw new RuntimeException("Time domain not recognized: " + processModelElement.getTimeDomain());
      }

      stateSizes.put(statePort, stateSize);
      registerOutputPort(statePort);
   }

   public void addMeasurementModelElement(ControlFlowInputPort<?> measurementPort, MeasurementModelElement measurementModelElement)
   {
      measurementModelElements.put(measurementPort, measurementModelElement);
   }

   public void addPostStateChangeRunnable(Runnable runnable)
   {
      this.postStateChangeRunnables.add(runnable);
   }
   
   public void initialize()
   {
      int continuousStateSize = MatrixTools.computeIndicesIntoVector(continuousStatePorts, continuousStateStartIndices, stateSizes);
      int discreteStateSize = MatrixTools.computeIndicesIntoVector(discreteStatePorts, discreteStateStartIndices, stateSizes);

      allStatePorts.addAll(continuousStatePorts);
      allStatePorts.addAll(discreteStatePorts);
      MatrixTools.computeIndicesIntoVector(allStatePorts, allStateStartIndices, stateSizes);

      int inputSize = MatrixTools.computeIndicesIntoVector(processInputPorts, processInputStartIndices, processInputSizes);
      int measurementSize = MatrixTools.computeIndicesIntoVector(measurementInputPorts, measurementStartIndices, measurementSizes);

      kalmanFilter = new ComposableStateEstimatorKalmanFilter(continuousStateSize, discreteStateSize, inputSize, measurementSize);

      for (Runnable runnable : postStateChangeRunnables)
         runnable.run();

      kalmanFilter.configure();
      initializeCovariance();
   }

   private void initializeCovariance()
   {
      DenseMatrix64F x = kalmanFilter.getState();
      kalmanFilter.computeSteadyStateGainAndCovariance(50); // TODO: magic number
      DenseMatrix64F P = kalmanFilter.getCovariance();
      CommonOps.scale(10.0, P); // TODO: magic number
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

      // continuous time process model (requires discretization)
      private final StateSpaceSystemDiscretizer discretizer;
      private final DenseMatrix64F FContinuous;
      private final DenseMatrix64F GContinuous;
      private final DenseMatrix64F QContinuous;

      // discrete time process model (does not require discretization)
      private final DenseMatrix64F FDiscrete;
      private final DenseMatrix64F GDiscrete;
      private final DenseMatrix64F QDiscrete;

      // combined process model
      private final DenseMatrix64F F;
      private final DenseMatrix64F G;
      private final DenseMatrix64F Q;

      // measurement model
      private final DenseMatrix64F H;
      private final DenseMatrix64F R;

      public ComposableStateEstimatorKalmanFilter(int continuousStateSize, int discreteStateSize, int inputSize, int measurementSize)
      {
         super(ComposableStateEstimatorKalmanFilter.class.getSimpleName(), continuousStateSize + discreteStateSize, inputSize, measurementSize,
               ComposableStateEstimator.this.registry);

         this.discretizer = new StateSpaceSystemDiscretizer(continuousStateSize, inputSize);
         this.FContinuous = new DenseMatrix64F(continuousStateSize, continuousStateSize);
         this.GContinuous = new DenseMatrix64F(continuousStateSize, inputSize);
         this.QContinuous = new DenseMatrix64F(continuousStateSize, continuousStateSize);

         this.FDiscrete = new DenseMatrix64F(discreteStateSize, discreteStateSize);
         this.GDiscrete = new DenseMatrix64F(discreteStateSize, inputSize);
         this.QDiscrete = new DenseMatrix64F(discreteStateSize, discreteStateSize);

         int stateSize = continuousStateSize + discreteStateSize;
         this.F = new DenseMatrix64F(stateSize, stateSize);
         this.G = new DenseMatrix64F(stateSize, inputSize);
         this.Q = new DenseMatrix64F(stateSize, stateSize);

         this.H = new DenseMatrix64F(measurementSize, stateSize);
         this.R = new DenseMatrix64F(measurementSize, measurementSize);
         
         this.correction = new DenseMatrix64F(stateSize, 1);
         this.residual = new DenseMatrix64F(measurementSize, 1);
      }

      protected void configure()
      {
         updateProcessModel(F, G, Q);
         updateMeasurementModel(H, R);
         super.configure(F, G, H);
         setProcessNoiseCovariance(Q);
         setMeasurementNoiseCovariance(R);
      }

      @Override
      protected void updateAPrioriState(DenseMatrix64F x, DenseMatrix64F u)
      {
         for (ProcessModelElement processModelElement : processModelElements.values())
         {
            processModelElement.propagateState(controlDT); // should update what's in the output ports
         }

         for (Runnable runnable : postStateChangeRunnables)
         {
            runnable.run();
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

         for (ControlFlowOutputPort<?> statePort : allStatePorts)
         {
            ProcessModelElement processModelElement = processModelElements.get(statePort);
            MatrixTools.extractVectorBlock(correctionBlock, correction, statePort, allStateStartIndices, stateSizes);

            processModelElement.correctState(correctionBlock);
         }

         for (Runnable runnable : postStateChangeRunnables)
         {
            runnable.run();
         }
      }

      private void updateProcessModel(DenseMatrix64F F, DenseMatrix64F G, DenseMatrix64F Q)
      {
         for (ProcessModelElement processModelElement : processModelElements.values())
         {
            processModelElement.computeMatrixBlocks();
         }

         updateProcessModelBlock(FContinuous, GContinuous, QContinuous, continuousStateStartIndices, continuousStatePorts);
         updateProcessModelBlock(FDiscrete, GDiscrete, QDiscrete, discreteStateStartIndices, discreteStatePorts);

         discretizer.discretize(FContinuous, GContinuous, QContinuous, controlDT);

         assembleProcessModel(F, G, Q, FContinuous, GContinuous, QContinuous, FDiscrete, GDiscrete, QDiscrete);
      }

      private void updateProcessModelBlock(DenseMatrix64F F, DenseMatrix64F G, DenseMatrix64F Q, Map<ControlFlowOutputPort<?>, Integer> stateStartIndices,
            List<ControlFlowOutputPort<?>> statePorts)
      {
         F.zero();
         G.zero();
         Q.zero();

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

      private void assembleProcessModel(DenseMatrix64F F, DenseMatrix64F G, DenseMatrix64F Q, DenseMatrix64F FContinuous, DenseMatrix64F GContinuous,
            DenseMatrix64F QContinuous, DenseMatrix64F FDiscrete, DenseMatrix64F GDiscrete, DenseMatrix64F QDiscrete)
      {
         F.zero();
         G.zero();
         Q.zero();

         CommonOps.insert(FContinuous, F, 0, 0);
         CommonOps.insert(FDiscrete, F, FContinuous.getNumRows(), FContinuous.getNumCols());

         CommonOps.insert(GContinuous, G, 0, 0);
         CommonOps.insert(GDiscrete, G, GContinuous.getNumRows(), 0);

         CommonOps.insert(QContinuous, Q, 0, 0);
         CommonOps.insert(QDiscrete, Q, QContinuous.getNumRows(), QContinuous.getNumCols());
      }

      private void updateMeasurementModel(DenseMatrix64F H, DenseMatrix64F R)
      {
         H.zero();
         R.zero();

         for (ControlFlowInputPort<?> measurementInputPort : measurementInputPorts)
         {
            MeasurementModelElement measurementModelElement = measurementModelElements.get(measurementInputPort);
            measurementModelElement.computeMatrixBlocks();

            for (ControlFlowOutputPort<?> statePort : allStatePorts)
            {
               DenseMatrix64F outputMatrixBlock = measurementModelElement.getOutputMatrixBlock(statePort);
               MatrixTools.insertMatrixBlock(H, outputMatrixBlock, measurementInputPort, measurementStartIndices, statePort, allStateStartIndices);
            }

            DenseMatrix64F measurementCovarianceMatrixBlock = measurementModelElement.getMeasurementCovarianceMatrixBlock();
            MatrixTools.insertMatrixBlock(R, measurementCovarianceMatrixBlock, measurementInputPort, measurementStartIndices, measurementInputPort,
                  measurementStartIndices);
         }
      }
   }
}
