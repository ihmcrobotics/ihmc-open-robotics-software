package us.ihmc.sensorProcessing.stateEstimation.measurementModelElements;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.AfterJointReferenceFrameNameMap;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.PointPositionDataObject;

/**
 * @author twan
 *         Date: 4/27/13
 */
public class AggregatePointPositionMeasurementModelElement implements MeasurementModelElement
{
   private YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final List<PointPositionMeasurementModelElement> elementPool = new ArrayList<PointPositionMeasurementModelElement>();
   private final ControlFlowInputPort<List<PointPositionDataObject>> inputPort;
   private final ControlFlowOutputPort<FramePoint> centerOfMassPositionPort;
   private final ControlFlowOutputPort<FrameOrientation> orientationPort;
   private final ReferenceFrame estimationFrame;
   private final AfterJointReferenceFrameNameMap referenceFrameMap;
   private final ArrayList<ControlFlowOutputPort<?>> statePorts = new ArrayList<ControlFlowOutputPort<?>>();

   private final Map<ControlFlowOutputPort<?>, DenseMatrix64F> outputMatrixBlocks = new LinkedHashMap<ControlFlowOutputPort<?>, DenseMatrix64F>();
   private final DenseMatrix64F measurementCovarianceMatrixBlock = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F residual = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F singlePointCovariance = new DenseMatrix64F(PointPositionMeasurementModelElement.SIZE, PointPositionMeasurementModelElement.SIZE);

   private final boolean assumePerfectIMU;
   private int nElementsInUse;


   public AggregatePointPositionMeasurementModelElement(ControlFlowInputPort<List<PointPositionDataObject>> inputPort,
         ControlFlowOutputPort<FramePoint> centerOfMassPositionPort, ControlFlowOutputPort<FrameOrientation> orientationPort, ReferenceFrame estimationFrame,
         AfterJointReferenceFrameNameMap referenceFrameMap, boolean assumePerfectIMU)
   {
      this.inputPort = inputPort;
      this.centerOfMassPositionPort = centerOfMassPositionPort;
      this.orientationPort = orientationPort;
      this.estimationFrame = estimationFrame;
      this.referenceFrameMap = referenceFrameMap;
      this.assumePerfectIMU = assumePerfectIMU;

      statePorts.add(centerOfMassPositionPort);
      
      if(!assumePerfectIMU)
      {
         statePorts.add(orientationPort);
      }
      
      for (ControlFlowOutputPort<?> statePort : statePorts)
      {
         outputMatrixBlocks.put(statePort, new DenseMatrix64F(1, 1));
      }
   }

   public void computeMatrixBlocks()
   {
      List<PointPositionDataObject> pointPositionDataObjects = inputPort.getData();
      nElementsInUse = pointPositionDataObjects.size();

      for (int i = 0; i < pointPositionDataObjects.size(); i++)
      {
         if (!pointPositionDataObjects.get(i).isPointPositionValid())
         {
            nElementsInUse--;
         }
      }
      

      for (int i = elementPool.size(); i < nElementsInUse; i++)
      {
         String name = "pointPosition" + i;
         ControlFlowInputPort<PointPositionDataObject> pointPositionMeasurementInputPort = new ControlFlowInputPort<PointPositionDataObject>("pointPositionMeasurementInputPort", null);
         PointPositionMeasurementModelElement element = new PointPositionMeasurementModelElement(name, pointPositionMeasurementInputPort,
                                                           centerOfMassPositionPort, orientationPort, estimationFrame, referenceFrameMap, assumePerfectIMU, registry);
         element.setNoiseCovariance(singlePointCovariance);
         elementPool.add(element);
      }

      reshapeAndZeroMatrices();

      int rowIndex = 0;
      int elementIndex = 0;
      for (PointPositionDataObject pointPositionDataObject : pointPositionDataObjects)
      {
         if (!pointPositionDataObject.isPointPositionValid())
            continue;
         
         PointPositionMeasurementModelElement element = elementPool.get(elementIndex);
         element.getPointPositionMeasurementInputPort().setData(pointPositionDataObject);
         element.computeMatrixBlocks();

         for (ControlFlowOutputPort<?> statePort : statePorts)
         {
            DenseMatrix64F outputMatrixBlock = element.getOutputMatrixBlock(statePort);
            CommonOps.insert(outputMatrixBlock, outputMatrixBlocks.get(statePort), rowIndex, 0);
         }

         CommonOps.insert(element.getMeasurementCovarianceMatrixBlock(), measurementCovarianceMatrixBlock, rowIndex, rowIndex);

         rowIndex += PointPositionMeasurementModelElement.SIZE;
         elementIndex++;
      }
   }

   private void reshapeAndZeroMatrices()
   {
      int size = nElementsInUse * PointPositionMeasurementModelElement.SIZE;
      for (int i = 0; i < statePorts.size(); i++)
      {
         DenseMatrix64F outputMatrixBlock = outputMatrixBlocks.get(statePorts.get(i));
         outputMatrixBlock.reshape(size, PointPositionMeasurementModelElement.SIZE);
         outputMatrixBlock.zero();
      }
      measurementCovarianceMatrixBlock.reshape(size, size);
      measurementCovarianceMatrixBlock.zero();
      residual.reshape(size, 1);
      residual.zero();
   }

   public DenseMatrix64F getOutputMatrixBlock(ControlFlowOutputPort<?> statePort)
   {
      return outputMatrixBlocks.get(statePort);
   }

   public DenseMatrix64F getMeasurementCovarianceMatrixBlock()
   {
      return measurementCovarianceMatrixBlock;
   }

   public DenseMatrix64F computeResidual()
   {
      int rowStart = 0;
      for (int i = 0; i < nElementsInUse; i++)
      {
         PointPositionMeasurementModelElement element = elementPool.get(i);
         DenseMatrix64F residualBlock = element.computeResidual();
         CommonOps.insert(residualBlock, residual, rowStart, 0);
         rowStart += PointPositionMeasurementModelElement.SIZE;
      }

      return residual;
   }

   public ArrayList<ControlFlowOutputPort<?>> getStatePorts()
   {
      return statePorts;
   }

   public void setNoiseCovariance(DenseMatrix64F covariance)
   {
      singlePointCovariance.set(covariance);
      for (PointPositionMeasurementModelElement pointPositionMeasurementModelElement : elementPool)
      {
         pointPositionMeasurementModelElement.setNoiseCovariance(covariance);
      }
   }
}
