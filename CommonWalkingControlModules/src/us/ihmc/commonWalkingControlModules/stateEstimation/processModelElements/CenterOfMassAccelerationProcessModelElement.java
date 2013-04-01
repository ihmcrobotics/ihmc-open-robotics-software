package us.ihmc.commonWalkingControlModules.stateEstimation.processModelElements;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.stateEstimation.TimeDomain;
import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.utilities.math.MatrixTools;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class CenterOfMassAccelerationProcessModelElement extends AbstractProcessModelElement
{
   private static final int SIZE = 3;
   private final ControlFlowOutputPort<FrameVector> centerOfMassAccelerationStatePort;
   private final ControlFlowInputPort<FrameVector> centerOfMassAccelerationInputPort;
   private final FrameVector comAcceleration = new FrameVector(ReferenceFrame.getWorldFrame());
   private final FrameVector comAccelerationDelta = new FrameVector(ReferenceFrame.getWorldFrame());

   public CenterOfMassAccelerationProcessModelElement(String name, YoVariableRegistry registry, ControlFlowOutputPort<FrameVector> centerOfMassAccelerationStatePort, ControlFlowInputPort<FrameVector> centerOfMassAccelerationInputPort)
   {
      super(TimeDomain.DISCRETE, SIZE, name, registry);
      this.centerOfMassAccelerationStatePort = centerOfMassAccelerationStatePort;
      this.centerOfMassAccelerationInputPort = centerOfMassAccelerationInputPort;
      
      computeCenterOfMassAccelerationInputMatrixBlock();
   }

   private void computeCenterOfMassAccelerationInputMatrixBlock()
   {
      DenseMatrix64F centerOfMassAccelerationInputBlock = new DenseMatrix64F(SIZE, SIZE);
      CommonOps.setIdentity(centerOfMassAccelerationInputBlock);
      inputMatrixBlocks.put(centerOfMassAccelerationInputPort, centerOfMassAccelerationInputBlock);
   }

   public void computeMatrixBlocks()
   {
      // empty
   }

   public void propagateState(double dt)
   {
      comAcceleration.set(centerOfMassAccelerationInputPort.getData());
      centerOfMassAccelerationStatePort.setData(comAcceleration);
   }

   public void correctState(DenseMatrix64F correction)
   {
      MatrixTools.extractTuple3dFromEJMLVector(comAccelerationDelta.getVector(), correction, 0);
      comAcceleration.set(centerOfMassAccelerationStatePort.getData());
      comAcceleration.add(comAccelerationDelta);
      centerOfMassAccelerationStatePort.setData(comAcceleration);
   }
}
