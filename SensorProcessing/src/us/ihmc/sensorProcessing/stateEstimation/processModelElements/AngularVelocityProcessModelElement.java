package us.ihmc.sensorProcessing.stateEstimation.processModelElements;


import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.sensorProcessing.stateEstimation.TimeDomain;

public class AngularVelocityProcessModelElement extends AbstractProcessModelElement
{
   private static final int SIZE = 3;
   private final ReferenceFrame estimationFrame;
   private final ControlFlowOutputPort<FrameVector> angularVelocityPort;
   private final ControlFlowInputPort<FrameVector> angularAccelerationPort;

   // temp stuff
   private final FrameVector angularVelocity;
   private final FrameVector angularVelocityDelta;
   private final FrameVector angularAcceleration;
   private final Vector3D angularAccelerationVector3d = new Vector3D();
   
   public AngularVelocityProcessModelElement(ReferenceFrame estimationFrame, ControlFlowOutputPort<FrameVector> angularVelocityPort,
           ControlFlowInputPort<FrameVector> angularAccelerationPort, String name, YoVariableRegistry registry)
   {
      super(angularVelocityPort, TimeDomain.CONTINUOUS, false, SIZE, name, registry);
      this.estimationFrame = estimationFrame;
      this.angularVelocityPort = angularVelocityPort;
      this.angularAccelerationPort = angularAccelerationPort;
      this.angularVelocity = new FrameVector(estimationFrame);
      this.angularVelocityDelta = new FrameVector(estimationFrame);
      this.angularAcceleration = new FrameVector(estimationFrame);
      
      if (angularAccelerationPort != null)
      {
         inputMatrixBlocks.put(angularAccelerationPort, new DenseMatrix64F(SIZE, SIZE));
         computeAngularAccelerationInputMatrixBlock();
      }
   }

   private void computeAngularAccelerationInputMatrixBlock()
   {
      CommonOps.setIdentity(inputMatrixBlocks.get(angularAccelerationPort));
   }

   public void computeMatrixBlocks()
   {
      // empty
   }

   public void propagateState(double dt)
   {
      if (angularAccelerationPort != null)
      {
         FrameVector angularAccelerationData = angularAccelerationPort.getData();
         angularAccelerationData.get(angularAccelerationVector3d);
         
         //TODO: Figure out how to deal best with HumanoidReferenceFrames here.
         // Upon generation, the generator might not know what the estimation frame will
         // be. So here we're just making sure that the frame is null if it's not
         // the estimation frame.
         
         if (angularAccelerationData.getReferenceFrame() != null)
         {
            angularAccelerationData.checkReferenceFrameMatch(estimationFrame);
         }
         
         angularAcceleration.setIncludingFrame(estimationFrame, angularAccelerationVector3d);
//         angularAcceleration.changeFrame(estimationFrame);
         angularVelocityDelta.set(angularAcceleration);
         angularVelocityDelta.scale(dt);

         updateAngularVelocity(angularVelocityDelta);
      }
   }

   public void correctState(DenseMatrix64F correction)
   {
      MatrixTools.extractTuple3dFromEJMLVector(angularVelocityDelta.getVector(), correction, 0);
      updateAngularVelocity(angularVelocityDelta);
   }

   private void updateAngularVelocity(FrameVector angularVelocityDelta)
   {
      angularVelocity.set(angularVelocityPort.getData());
      angularVelocity.add(angularVelocityDelta);
      angularVelocityPort.setData(angularVelocity);
   }
}
