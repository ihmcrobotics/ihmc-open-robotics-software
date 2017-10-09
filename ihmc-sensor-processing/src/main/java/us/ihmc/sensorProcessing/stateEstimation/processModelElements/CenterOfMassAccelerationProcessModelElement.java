package us.ihmc.sensorProcessing.stateEstimation.processModelElements;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.sensorProcessing.stateEstimation.TimeDomain;

public class CenterOfMassAccelerationProcessModelElement extends AbstractProcessModelElement
{
   private static final boolean USE_INPUT_ACCELERATION = false;
   
   private static final int SIZE = 3;
   private final ControlFlowOutputPort<FrameVector3D> centerOfMassAccelerationStatePort;
   private final ControlFlowInputPort<FrameVector3D> centerOfMassAccelerationInputPort;
   private final FrameVector3D comAcceleration = new FrameVector3D(ReferenceFrame.getWorldFrame());
   private final FrameVector3D comAccelerationDelta = new FrameVector3D(ReferenceFrame.getWorldFrame());

   public CenterOfMassAccelerationProcessModelElement(String name, YoVariableRegistry registry, ControlFlowOutputPort<FrameVector3D> centerOfMassAccelerationStatePort, ControlFlowInputPort<FrameVector3D> centerOfMassAccelerationInputPort)
   {
      super(centerOfMassAccelerationStatePort, TimeDomain.DISCRETE, false, SIZE, name, registry);
      this.centerOfMassAccelerationStatePort = centerOfMassAccelerationStatePort;
      this.centerOfMassAccelerationInputPort = centerOfMassAccelerationInputPort;
      
      if (USE_INPUT_ACCELERATION) computeCenterOfMassAccelerationInputMatrixBlock();
      else
      {
         computeCenterOfMassAccelerationStateMatrixBlock();
      }
   }

   private void computeCenterOfMassAccelerationInputMatrixBlock()
   {
      DenseMatrix64F centerOfMassAccelerationInputBlock = new DenseMatrix64F(SIZE, SIZE);
      CommonOps.setIdentity(centerOfMassAccelerationInputBlock);
      inputMatrixBlocks.put(centerOfMassAccelerationInputPort, centerOfMassAccelerationInputBlock);
   }
   
   private void computeCenterOfMassAccelerationStateMatrixBlock()
   {
      stateMatrixBlocks.put(centerOfMassAccelerationStatePort, new DenseMatrix64F(SIZE, SIZE));
      DenseMatrix64F comAccelerationMatrixBlock = stateMatrixBlocks.get(centerOfMassAccelerationStatePort);
      CommonOps.setIdentity(comAccelerationMatrixBlock);
   }

   public void computeMatrixBlocks()
   {
      // empty
   }

   public void propagateState(double dt)
   {
      if (USE_INPUT_ACCELERATION)
      {
         comAcceleration.set(centerOfMassAccelerationInputPort.getData());
         centerOfMassAccelerationStatePort.setData(comAcceleration); 
      }

      else
      {
         comAcceleration.set(centerOfMassAccelerationStatePort.getData());
         centerOfMassAccelerationStatePort.setData(comAcceleration);
      }
   }

   public void correctState(DenseMatrix64F correction)
   {
      MatrixTools.extractTuple3dFromEJMLVector(comAccelerationDelta.getVector(), correction, 0);
      comAcceleration.set(centerOfMassAccelerationStatePort.getData());
      comAcceleration.add(comAccelerationDelta);
      centerOfMassAccelerationStatePort.setData(comAcceleration);
   }
}
