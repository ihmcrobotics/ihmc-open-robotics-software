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

public class AngularAccelerationProcessModelElement extends AbstractProcessModelElement
{
   private static final int SIZE = 3;
   private final ControlFlowOutputPort<FrameVector> angularAccelerationStatePort;
   private final ControlFlowInputPort<FrameVector> angularAccelerationInputPort;
   private final FrameVector angularAcceleration = new FrameVector(ReferenceFrame.getWorldFrame());
   private final FrameVector angularAccelerationDelta = new FrameVector(ReferenceFrame.getWorldFrame());

   public AngularAccelerationProcessModelElement(String name, YoVariableRegistry registry, ControlFlowOutputPort<FrameVector> angularAccelerationStatePort, ControlFlowInputPort<FrameVector> angularAccelerationInputPort)
   {
      super(TimeDomain.DISCRETE, SIZE, name, registry);
      this.angularAccelerationStatePort = angularAccelerationStatePort;
      this.angularAccelerationInputPort = angularAccelerationInputPort;
      
      computeAngularAccelerationInputMatrixBlock();
   }

   private void computeAngularAccelerationInputMatrixBlock()
   {
      DenseMatrix64F angularAccelerationInputBlock = new DenseMatrix64F(SIZE, SIZE);
      CommonOps.setIdentity(angularAccelerationInputBlock);
      inputMatrixBlocks.put(angularAccelerationInputPort, angularAccelerationInputBlock);
   }

   public void computeMatrixBlocks()
   {
      // empty
   }

   public void propagateState(double dt)
   {
      angularAcceleration.set(angularAccelerationInputPort.getData());
      angularAccelerationStatePort.setData(angularAcceleration);
   }

   public void correctState(DenseMatrix64F correction)
   {
      MatrixTools.extractTuple3dFromEJMLVector(angularAccelerationDelta.getVector(), correction, 0);
      angularAcceleration.set(angularAccelerationStatePort.getData());
      angularAcceleration.add(angularAccelerationDelta);
      angularAccelerationStatePort.setData(angularAcceleration);
   }
}
