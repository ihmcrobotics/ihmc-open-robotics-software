package us.ihmc.sensorProcessing.stateEstimation.measurementModelElements;


import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;

public class AngularVelocityMeasurementModelElement extends AbstractMeasurementModelElement
{
   static final int SIZE = 3;
   private final ControlFlowOutputPort<FrameVector3D> angularVelocityStatePort;
   private final ControlFlowOutputPort<FrameVector3D> biasStatePort;
   private final ControlFlowInputPort<Vector3D> angularVelocityMeasurementInputPort;

   private final RigidBody orientationEstimationLink;
   private final ReferenceFrame estimationFrame;

   private final RigidBody measurementLink;
   private final ReferenceFrame measurementFrame;

   private final ControlFlowInputPort<FullInverseDynamicsStructure> inverseDynamicsStructureInputPort;
   private final DenseMatrix64F residual = new DenseMatrix64F(SIZE, 1);

   // temp stuff
   private final Twist tempTwist = new Twist();
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final RotationMatrix tempMatrix = new RotationMatrix();
   private final FrameVector3D relativeAngularVelocity = new FrameVector3D(ReferenceFrame.getWorldFrame());
   private final FrameVector3D angularVelocityResidual;

   public AngularVelocityMeasurementModelElement(ControlFlowOutputPort<FrameVector3D> angularVelocityStatePort, ControlFlowOutputPort<FrameVector3D> biasStatePort,
           ControlFlowInputPort<Vector3D> angularVelocityMeasurementInputPort, RigidBody orientationEstimationLink, ReferenceFrame estimationFrame,
           RigidBody measurementLink, ReferenceFrame measurementFrame, ControlFlowInputPort<FullInverseDynamicsStructure> inverseDynamicsStructureInputPort, String name, YoVariableRegistry registry)
   {
      super(SIZE, name, registry);
      this.angularVelocityStatePort = angularVelocityStatePort;
      this.biasStatePort = biasStatePort;
      this.angularVelocityMeasurementInputPort = angularVelocityMeasurementInputPort;
      this.orientationEstimationLink = orientationEstimationLink;
      this.estimationFrame = estimationFrame;
      this.measurementLink = measurementLink;
      this.measurementFrame = measurementFrame;
      this.inverseDynamicsStructureInputPort = inverseDynamicsStructureInputPort;
      this.angularVelocityResidual = new FrameVector3D(measurementFrame);

      initialize(SIZE, angularVelocityStatePort, biasStatePort);

      computeAngularVelocityStateOutputBlock();
      computeBiasStateOutputBlock();
   }

   private void computeAngularVelocityStateOutputBlock()
   {
      estimationFrame.getTransformToDesiredFrame(tempTransform, measurementFrame);
      tempTransform.getRotation(tempMatrix);
      tempMatrix.get(getOutputMatrixBlock(angularVelocityStatePort));
   }

   private void computeBiasStateOutputBlock()
   {
      CommonOps.setIdentity(getOutputMatrixBlock(biasStatePort));
   }

   public void computeMatrixBlocks()
   {
      computeAngularVelocityStateOutputBlock();
   }

   private final FrameVector3D predictedAngularVelocityMeasurementTemp = new FrameVector3D();
   public DenseMatrix64F computeResidual()
   {
      Vector3D measuredAngularVelocityVector3d = angularVelocityMeasurementInputPort.getData();
      
      measurementLink.getBodyFixedFrame().getTwistRelativeToOther(orientationEstimationLink.getBodyFixedFrame(), tempTwist);
      tempTwist.getAngularPart(relativeAngularVelocity);
      relativeAngularVelocity.changeFrame(measurementFrame);

      predictedAngularVelocityMeasurementTemp.setIncludingFrame(angularVelocityStatePort.getData());
      predictedAngularVelocityMeasurementTemp.changeFrame(measurementFrame);
      predictedAngularVelocityMeasurementTemp.add(relativeAngularVelocity);
      predictedAngularVelocityMeasurementTemp.add(biasStatePort.getData());

      angularVelocityResidual.setIncludingFrame(measurementFrame, measuredAngularVelocityVector3d);
      angularVelocityResidual.sub(predictedAngularVelocityMeasurementTemp);
      angularVelocityResidual.getVector().get(residual);

      return residual;
   }

}
