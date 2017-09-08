package us.ihmc.sensorProcessing.stateEstimation;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.sensorProcessing.stateEstimation.measurementModelElements.MeasurementModelElement;

public class MeasurementModel
{
   private final ArrayList<MeasurementModelElement> measurementModelElements;

   private final Map<MeasurementModelElement, Integer> measurementStartIndices = new LinkedHashMap<MeasurementModelElement, Integer>();
   private final Map<ControlFlowOutputPort<?>, Integer> stateStartIndices;

   private final DenseMatrix64F H = new DenseMatrix64F(1, 1); // reshape dynamically
   private final DenseMatrix64F R = new DenseMatrix64F(1, 1); // reshape dynamically
   private final DenseMatrix64F residual = new DenseMatrix64F(1, 1); // reshape dynamically

   private final Map<MeasurementModelElement,Integer> measurementSizes = new LinkedHashMap<MeasurementModelElement, Integer>();
   private final int stateMatrixSize;
   private int measurementOutputMatrixSize;

   public MeasurementModel(ArrayList<MeasurementModelElement> measurementModelElements, Map<ControlFlowOutputPort<?>, Integer> stateStartIndices, int stateMatrixSize)
   {
      this.measurementModelElements = measurementModelElements;
      this.stateStartIndices = stateStartIndices;
      if (this.stateStartIndices == null) throw new RuntimeException("stateStartIndices == null");
      
      this.stateMatrixSize = stateMatrixSize;
      reshapeMatrices(); // TODO: get rid of this
   }

   public void updateMatrices()
   {
      for(int i = 0; i <  measurementModelElements.size(); i++)
      {
         measurementModelElements.get(i).computeMatrixBlocks();
      }

      reshapeMatrices();

      // TODO: check if necessary:
      H.zero();
      R.zero();

      for(int i = 0; i <  measurementModelElements.size(); i++)
      {
         MeasurementModelElement measurementModelElement = measurementModelElements.get(i);
         int measurementStartIndex = measurementStartIndices.get(measurementModelElement);

         for (int j = 0; j < measurementModelElement.getStatePorts().size(); j++)
         {
            ControlFlowOutputPort<?> statePort  = measurementModelElement.getStatePorts().get(j);
            Integer stateStartIndexInteger = stateStartIndices.get(statePort);
            if (stateStartIndexInteger == null) throw new RuntimeException("Cannot find state port " + statePort + " for MeasurementModelElement " + measurementModelElement);

            int stateStartIndex = stateStartIndexInteger;
            
            
            DenseMatrix64F outputMatrixBlock = measurementModelElement.getOutputMatrixBlock(statePort);
            CommonOps.insert(outputMatrixBlock, H, measurementStartIndex, stateStartIndex);
         }
         DenseMatrix64F measurementCovarianceBlock = measurementModelElement.getMeasurementCovarianceMatrixBlock();
         CommonOps.insert(measurementCovarianceBlock, R, measurementStartIndex, measurementStartIndex);
      }
      
//      if (H.getNumRows() == 0) throw new RuntimeException("H.getNumRows() == 0");
   }

   public DenseMatrix64F computeResidual()
   {
      for(int i = 0; i <  measurementModelElements.size(); i++)
      {
         DenseMatrix64F residualBlock = measurementModelElements.get(i).computeResidual();
         int measurementStartIndex = measurementStartIndices.get(measurementModelElements.get(i));
         CommonOps.insert(residualBlock, residual, measurementStartIndex, 0);
      }
      return residual;
   }

   public DenseMatrix64F getOutputMatrix()
   {
      return H;
   }

   public DenseMatrix64F getMeasurementCovarianceMatrix()
   {
      return R;
   }

   private static int computeTotalSize(Iterable<Integer> sizes)
   {
      int ret = 0;
      for (Integer size : sizes)
      {
         ret += size;
      }
      return ret;
   }

   private static void computeMeasurementSizes(ArrayList<MeasurementModelElement> measurementModelElements, Map<MeasurementModelElement, Integer> measurementSizes)
   {
      for(int i = 0; i <  measurementModelElements.size(); i++)
      {
         MeasurementModelElement measurementModelElement = measurementModelElements.get(i);
         List<ControlFlowOutputPort<?>> measurementModelElementStates = measurementModelElement.getStatePorts();
         for (int j = 0; j < measurementModelElementStates.size(); j++)
         {
            DenseMatrix64F outputMatrixBlock = measurementModelElement.getOutputMatrixBlock(measurementModelElementStates.get(j));
            checkOrAddSize(measurementSizes, measurementModelElement, outputMatrixBlock.getNumRows());
         }
      }
   }

   private static <KeyType> void checkOrAddSize(Map<KeyType, Integer> sizes, KeyType key, int newSize)
   {
      Integer stateSize = sizes.get(key);
      if (stateSize == null)
      {
         sizes.put(key, newSize);
      }
      else if (stateSize != newSize)
      {
         throw new RuntimeException("State matrix size is ambiguous");
      }
   }

   public int getOutputMatrixSize()
   {
      return measurementOutputMatrixSize;
   }

   private void reshapeMatrices()
   {
      measurementSizes.clear();
      computeMeasurementSizes(measurementModelElements, measurementSizes);
      MatrixTools.computeIndicesIntoVector(measurementModelElements, measurementStartIndices, measurementSizes);
      measurementOutputMatrixSize = computeTotalSize(measurementSizes.values());
      H.reshape(measurementOutputMatrixSize, stateMatrixSize);
      R.reshape(measurementOutputMatrixSize, measurementOutputMatrixSize);
      residual.reshape(measurementOutputMatrixSize, 1);
   }
}
