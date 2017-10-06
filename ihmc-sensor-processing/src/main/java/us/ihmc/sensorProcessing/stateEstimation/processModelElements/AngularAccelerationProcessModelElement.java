package us.ihmc.sensorProcessing.stateEstimation.processModelElements;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.sensorProcessing.stateEstimation.TimeDomain;

public class AngularAccelerationProcessModelElement extends AbstractProcessModelElement
{
   private static final int SIZE = 3;
   private final ControlFlowOutputPort<FrameVector3D> angularAccelerationStatePort;
   private final ControlFlowInputPort<FrameVector3D> angularAccelerationInputPort;
   private final FrameVector3D angularAcceleration;
   private final FrameVector3D angularAccelerationDelta;
   private final ReferenceFrame estimationFrame;
   private final Vector3D angularAccelerationVector3d = new Vector3D();

   public AngularAccelerationProcessModelElement(String name, ReferenceFrame estimationFrame, YoVariableRegistry registry,
           ControlFlowOutputPort<FrameVector3D> angularAccelerationStatePort, ControlFlowInputPort<FrameVector3D> angularAccelerationInputPort)
   {
      super(angularAccelerationStatePort, TimeDomain.DISCRETE, false, SIZE, name, registry);
      this.angularAccelerationStatePort = angularAccelerationStatePort;
      this.angularAccelerationInputPort = angularAccelerationInputPort;
      this.angularAcceleration = new FrameVector3D(estimationFrame);
      this.angularAccelerationDelta = new FrameVector3D(estimationFrame);

      this.estimationFrame = estimationFrame;
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
      FrameVector3D angularAccelerationInputData = angularAccelerationInputPort.getData();

      // TODO: Figure out how to deal best with HumanoidReferenceFrames here.
      // Upon generation, the generator might not know what the estimation frame will
      // be. So here we're just making sure that the frame is null if it's not
      // the estimation frame.

      if (angularAccelerationInputData.getReferenceFrame() != null)
      {
         angularAccelerationInputData.checkReferenceFrameMatch(estimationFrame);
      }

      angularAccelerationInputData.get(angularAccelerationVector3d);
      angularAcceleration.setIncludingFrame(estimationFrame, angularAccelerationVector3d);
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
