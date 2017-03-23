package us.ihmc.sensorProcessing.stateEstimation.measurementModelElements;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SpatialAccelerationCalculator;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;

public class LinearAccelerationMeasurementModelElement extends AbstractMeasurementModelElement
{
   private static final int SIZE = 3;

   private final ControlFlowOutputPort<FramePoint> centerOfMassPositionPort;
   private final ControlFlowOutputPort<FrameVector> centerOfMassVelocityPort;
   private final ControlFlowOutputPort<FrameVector> centerOfMassAccelerationPort;
   private final ControlFlowOutputPort<FrameOrientation> orientationPort;
   private final ControlFlowOutputPort<FrameVector> angularVelocityPort;
   private final ControlFlowOutputPort<FrameVector> angularAccelerationPort;
   private final ControlFlowOutputPort<FrameVector> biasPort;

   private final ControlFlowInputPort<Vector3D> linearAccelerationMeasurementInputPort;

   private final ControlFlowInputPort<FullInverseDynamicsStructure> inverseDynamicsStructureInputPort;

   private final RigidBody measurementLink;
   private final ReferenceFrame measurementFrame;

   private final RigidBody estimationLink;
   private final ReferenceFrame estimationFrame;

   private final DenseMatrix64F residual = new DenseMatrix64F(SIZE, 1);

   // intermediate result stuff:
   private final RotationMatrix rotationFromEstimationToWorld = new RotationMatrix();
   private final RotationMatrix rotationFromEstimationToMeasurement = new RotationMatrix();
   private final FrameVector omegaEstimationToMeasurement = new FrameVector(ReferenceFrame.getWorldFrame());
   private final FrameVector vEstimationToMeasurement = new FrameVector(ReferenceFrame.getWorldFrame());
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final Matrix3D tempMatrix = new Matrix3D();
   private final Vector3D tempVector = new Vector3D();
   private final FramePoint tempFramePoint = new FramePoint(ReferenceFrame.getWorldFrame());
   private final Twist twistOfEstimationLink = new Twist();
   private final Twist twistOfMeasurementFrameWithRespectToEstimation = new Twist();
   private final FrameVector tempFrameVector = new FrameVector(ReferenceFrame.getWorldFrame());
   private final FrameVector gravitationalAcceleration = new FrameVector(ReferenceFrame.getWorldFrame());

   private final Matrix3D omegaJOmega = new Matrix3D();
   private final Matrix3D omegaJV = new Matrix3D();
   private final Matrix3D omegaJOmegad = new Matrix3D();
   private final Matrix3D omegaJVd = new Matrix3D();

   private final Matrix3D phiJPhi = new Matrix3D();
   private final Matrix3D phiJOmega = new Matrix3D();
   private final Matrix3D phiJV = new Matrix3D();
   private final Matrix3D phiJOmegad = new Matrix3D();
   private final Matrix3D phiJVd = new Matrix3D();
   private final Matrix3D phiJP = new Matrix3D();

   private final LinearAccelerationMeasurementModelJacobianAssembler jacobianAssembler;

   private final FrameVector estimatedMeasurement = new FrameVector();

   private final Twist twistOfMeasurementLink = new Twist();
   private final SpatialAccelerationVector spatialAccelerationOfMeasurementLink = new SpatialAccelerationVector();


   public LinearAccelerationMeasurementModelElement(String name, YoVariableRegistry registry, ControlFlowOutputPort<FramePoint> centerOfMassPositionPort,
           ControlFlowOutputPort<FrameVector> centerOfMassVelocityPort, ControlFlowOutputPort<FrameVector> centerOfMassAccelerationPort,
           ControlFlowOutputPort<FrameOrientation> orientationPort, ControlFlowOutputPort<FrameVector> angularVelocityPort,
           ControlFlowOutputPort<FrameVector> angularAccelerationPort, ControlFlowOutputPort<FrameVector> biasPort, ControlFlowInputPort<Vector3D> linearAccelerationMeasurementInputPort,
           ControlFlowInputPort<FullInverseDynamicsStructure> inverseDynamicsStructureInputPort, 
           RigidBody measurementLink,
           ReferenceFrame measurementFrame, RigidBody estimationLink, ReferenceFrame estimationFrame, double gZ)
   {
      super(SIZE, name, registry);
      MathTools.checkIntervalContains(gZ, Double.NEGATIVE_INFINITY, 0.0);

      this.centerOfMassPositionPort = centerOfMassPositionPort;
      this.centerOfMassVelocityPort = centerOfMassVelocityPort;
      this.centerOfMassAccelerationPort = centerOfMassAccelerationPort;
      this.orientationPort = orientationPort;
      this.angularVelocityPort = angularVelocityPort;
      this.angularAccelerationPort = angularAccelerationPort;
      this.biasPort = biasPort;

      this.linearAccelerationMeasurementInputPort = linearAccelerationMeasurementInputPort;

      this.inverseDynamicsStructureInputPort = inverseDynamicsStructureInputPort;

      this.measurementLink = measurementLink;
      this.measurementFrame = measurementFrame;

      this.estimationLink = estimationLink;
      this.estimationFrame = estimationFrame;
      this.jacobianAssembler = new LinearAccelerationMeasurementModelJacobianAssembler(inverseDynamicsStructureInputPort, measurementLink,
              measurementFrame, estimationFrame);

      gravitationalAcceleration.setZ(gZ);

      initialize(SIZE, centerOfMassVelocityPort, centerOfMassAccelerationPort, orientationPort, angularVelocityPort, angularAccelerationPort, biasPort);

      computeBiasBlock();
   }

   private final FrameVector rdTemp = new FrameVector();
   private final FramePoint rPTemp = new FramePoint();
   private final FrameVector rPdTemp = new FrameVector();
   public void computeMatrixBlocks()
   {
      FullInverseDynamicsStructure inverseDynamicsStructure = inverseDynamicsStructureInputPort.getData();
      TwistCalculator twistCalculator = inverseDynamicsStructure.getTwistCalculator();
      SpatialAccelerationCalculator spatialAccelerationCalculator = inverseDynamicsStructure.getSpatialAccelerationCalculator();
      
      computeUnbiasedEstimatedMeasurement(spatialAccelerationCalculator, estimatedMeasurement);

      // R_{w}^{p}
      estimationFrame.getTransformToDesiredFrame(tempTransform, ReferenceFrame.getWorldFrame());
      tempTransform.getRotation(rotationFromEstimationToWorld);

      // R_{p}^{m}
      estimationFrame.getTransformToDesiredFrame(tempTransform, measurementFrame);
      tempTransform.getRotation(rotationFromEstimationToMeasurement);

      // T_{i}^{p,p}
      twistCalculator.getRelativeTwist(twistOfMeasurementFrameWithRespectToEstimation, estimationLink, measurementLink);

      // r^{p} 
      rPTemp.setIncludingFrame(centerOfMassPositionPort.getData());
      rPTemp.changeFrame(estimationFrame);

      rdTemp.setIncludingFrame(centerOfMassVelocityPort.getData());
      computeRpd(rPdTemp, twistCalculator, rPTemp, rdTemp);
      jacobianAssembler.preCompute(estimatedMeasurement.getVector());

      computeCenterOfMassVelocityBlock();
      computeCenterOfMassAccelerationBlock();
      computeOrientationBlock(twistCalculator, spatialAccelerationCalculator, rotationFromEstimationToWorld, twistOfMeasurementFrameWithRespectToEstimation, rPTemp, rPdTemp);
      computeAngularVelocityBlock(rotationFromEstimationToWorld, twistOfMeasurementFrameWithRespectToEstimation, rPTemp, rdTemp, rPdTemp);
      computeAngularAccelerationBlock(rotationFromEstimationToMeasurement);
   }

   private void computeRpd(FrameVector rPdToPack, TwistCalculator twistCalculator, FramePoint rP, FrameVector rd)
   {
      // T_{p}^{p,w}
      twistCalculator.getTwistOfBody(twistOfEstimationLink, estimationLink);
      twistOfEstimationLink.changeFrame(estimationFrame);
      
      // \dot{r}^{p} = R_{w}^{p} \dot{r} - \tilde{\omega}r^{p} - v_{p}^{p,w}
      rPdToPack.setIncludingFrame(rd);
      rPdToPack.changeFrame(estimationFrame);
      twistOfEstimationLink.getAngularPart(tempFrameVector);
      tempFrameVector.cross(tempFrameVector, rP);
      rPdToPack.sub(tempFrameVector);
      twistOfEstimationLink.getLinearPart(tempFrameVector);
      rPdToPack.sub(tempFrameVector);
   }

   private final FrameVector s = new FrameVector();

   private void computeOrientationBlock(TwistCalculator twistCalculator, SpatialAccelerationCalculator spatialAccelerationCalculator,
         RotationMatrix rotationFromEstimationToWorld, Twist twistOfMeasurementWithRespectToEstimation, FramePoint rP, FrameVector rPd)
   {
      // TODO: code and computation repeated in LinearAccelerationMeasurementModelJacobianAssembler
      RigidBody elevator = twistCalculator.getRootBody();
      ReferenceFrame elevatorFrame = elevator.getBodyFixedFrame();
      twistCalculator.getRelativeTwist(twistOfMeasurementLink, elevator, measurementLink);
      spatialAccelerationCalculator.getRelativeAcceleration(spatialAccelerationOfMeasurementLink, elevator, measurementLink);
      spatialAccelerationOfMeasurementLink.changeFrame(elevatorFrame, twistOfMeasurementLink, twistOfMeasurementLink);

      twistOfMeasurementWithRespectToEstimation.changeFrame(estimationFrame);
      twistOfMeasurementWithRespectToEstimation.getAngularPart(omegaEstimationToMeasurement);
      twistOfMeasurementWithRespectToEstimation.getLinearPart(vEstimationToMeasurement);

      FramePoint r = centerOfMassPositionPort.getData();
      FrameVector rd = centerOfMassVelocityPort.getData();
      FrameVector rdd = centerOfMassAccelerationPort.getData();

      r.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      rd.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      rdd.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());

      FrameVector omega = angularVelocityPort.getData();
      omega.checkReferenceFrameMatch(estimationFrame);

      // dPhidPhi
      phiJPhi.setIdentity();

      // dOmegaWWMdPhi
      twistOfMeasurementLink.changeFrame(elevatorFrame);
      twistOfMeasurementLink.getAngularPart(tempFrameVector);
      tempMatrix.setToTildeForm(tempFrameVector.getVector());
      phiJOmega.set(tempMatrix);
      phiJOmega.multiply(rotationFromEstimationToWorld);
      phiJOmega.scale(-1.0);

      // dVWWMdPhi
      tempFrameVector.setToZero(estimationFrame);
      tempFrameVector.cross(omegaEstimationToMeasurement, rP);
      tempFrameVector.add(vEstimationToMeasurement);
      tempFrameVector.sub(rPd);
      tempMatrix.setToTildeForm(tempFrameVector.getVector());
      phiJV.set(rotationFromEstimationToWorld);
      phiJV.multiply(tempMatrix);
      phiJV.scale(-1.0);

      tempMatrix.setToTildeForm(r.getPoint());
      tempMatrix.multiply(phiJOmega);
      phiJV.add(tempMatrix);

      // dOmegadWWMdPhi
      spatialAccelerationOfMeasurementLink.getAngularPart(tempVector);
      tempMatrix.setToTildeForm(tempVector);
      phiJOmegad.set(tempMatrix);
      phiJOmegad.multiply(rotationFromEstimationToWorld);
      phiJOmegad.scale(-1.0);

      // dVdWWMdPhi
      spatialAccelerationCalculator.getRelativeAcceleration(spatialAccelerationOfMeasurementLink, elevator, measurementLink);
      twistOfMeasurementWithRespectToEstimation.changeFrame(measurementLink.getBodyFixedFrame());
      twistOfMeasurementLink.changeFrame(measurementLink.getBodyFixedFrame());
      spatialAccelerationOfMeasurementLink.changeFrame(estimationLink.getBodyFixedFrame(), twistOfMeasurementFrameWithRespectToEstimation,
              twistOfMeasurementLink);
      spatialAccelerationOfMeasurementLink.changeFrameNoRelativeMotion(estimationFrame);
      twistOfMeasurementLink.changeFrame(estimationFrame);

      s.setToZero(estimationFrame);
      spatialAccelerationOfMeasurementLink.getAngularPart(tempFrameVector);
      tempFrameVector.cross(tempFrameVector, rP);
      s.add(tempFrameVector);

      spatialAccelerationOfMeasurementLink.getLinearPart(tempFrameVector);
      s.add(tempFrameVector);

      twistOfMeasurementLink.getAngularPart(tempFrameVector);
      tempFrameVector.cross(tempFrameVector, rPd);
      s.add(tempFrameVector);

      twistOfMeasurementLink.getLinearPart(tempFrameVector);
      tempFrameVector.cross(omega, tempFrameVector);
      s.add(tempFrameVector);

      twistOfMeasurementLink.getAngularPart(tempFrameVector);
      tempFrameVector.cross(tempFrameVector, rP);
      tempFrameVector.cross(omega, tempFrameVector);
      s.add(tempFrameVector);
      s.changeFrame(ReferenceFrame.getWorldFrame());

      tempFrameVector.setToZero(ReferenceFrame.getWorldFrame());
      tempFrameVector.sub(rdd, s);
      tempMatrix.setToTildeForm(tempFrameVector.getVector());
      phiJVd.set(tempMatrix);
      phiJVd.multiply(rotationFromEstimationToWorld);

      tempMatrix.setToTildeForm(r.getPoint());
      tempMatrix.multiply(phiJOmegad);
      phiJVd.add(tempMatrix);

      tempMatrix.setToTildeForm(rd.getVector());
      tempMatrix.multiply(phiJOmega);
      phiJVd.add(tempMatrix);

      // dPWIdPhi
      tempFrameVector.setIncludingFrame(rP);
      tempFramePoint.setToZero(measurementFrame);
      tempFramePoint.changeFrame(estimationFrame);
      tempFrameVector.sub(tempFramePoint);
      tempMatrix.setToTildeForm(tempFrameVector.getVector());
      phiJP.set(rotationFromEstimationToWorld);
      phiJP.multiply(tempMatrix);

      jacobianAssembler.assembleMeasurementJacobian(tempMatrix, phiJPhi, phiJOmega, phiJV, phiJOmegad, phiJVd, phiJP);
      tempMatrix.get(getOutputMatrixBlock(orientationPort));
   }

   private void computeAngularVelocityBlock(RotationMatrix rotationFromEstimationToWorld, Twist twistOfMeasurementWithRespectToEstimation, FramePoint rP,
           FrameVector rd, FrameVector rPd)
   {
      twistOfMeasurementWithRespectToEstimation.changeFrame(estimationFrame);
      twistOfMeasurementWithRespectToEstimation.getAngularPart(omegaEstimationToMeasurement);
      twistOfMeasurementWithRespectToEstimation.getLinearPart(vEstimationToMeasurement);

      // dOmegaWWMdOmega
      omegaJOmega.set(rotationFromEstimationToWorld);

      // dVWWMdOmega
      tempFramePoint.setIncludingFrame(centerOfMassPositionPort.getData());
      tempFramePoint.changeFrame(ReferenceFrame.getWorldFrame());
      tempMatrix.setToTildeForm(tempFramePoint.getPoint());
      omegaJV.set(tempMatrix);
      omegaJV.multiply(rotationFromEstimationToWorld);

      // dOmegadWWMdOmega
      tempMatrix.setToTildeForm(omegaEstimationToMeasurement.getVector());
      omegaJOmegad.set(rotationFromEstimationToWorld);
      omegaJOmegad.multiply(tempMatrix);
      omegaJOmegad.scale(-1.0);

      // dVdWWMdOmega
      rP.checkReferenceFrameMatch(estimationFrame);
      MatrixTools.setTildeTimesTilde(tempMatrix, rP.getPoint(), omegaEstimationToMeasurement.getVector());
      omegaJVd.set(tempMatrix);

      rPd.checkReferenceFrameMatch(estimationFrame);
      tempMatrix.setToTildeForm(rPd.getVector());
      omegaJVd.add(tempMatrix);

      MatrixTools.setTildeTimesTilde(tempMatrix, omegaEstimationToMeasurement.getVector(), rP.getPoint());
      omegaJVd.sub(tempMatrix);

      tempMatrix.setToTildeForm(vEstimationToMeasurement.getVector());
      omegaJVd.sub(tempMatrix);

      omegaJVd.set(rotationFromEstimationToWorld);
      omegaJVd.multiply(omegaJVd);

      tempMatrix.setToTildeForm(rd.getVector());
      tempMatrix.multiply(rotationFromEstimationToWorld);
      omegaJVd.add(tempMatrix);

      tempMatrix.setToTildeForm(omegaEstimationToMeasurement.getVector());
      tempMatrix.preMultiply(omegaJV);
      omegaJVd.sub(tempMatrix);

      jacobianAssembler.assembleMeasurementJacobian(tempMatrix, null, omegaJOmega, omegaJV, omegaJOmegad, omegaJVd, null);
      tempMatrix.get(getOutputMatrixBlock(angularVelocityPort));
   }

   private void computeCenterOfMassVelocityBlock()
   {
      tempMatrix.setToZero();
      tempMatrix.get(getOutputMatrixBlock(centerOfMassVelocityPort));
   }

   private void computeCenterOfMassAccelerationBlock()
   {
      ReferenceFrame.getWorldFrame().getTransformToDesiredFrame(tempTransform, measurementFrame);
      tempTransform.getRotation(tempMatrix);
      tempMatrix.get(getOutputMatrixBlock(centerOfMassAccelerationPort));
   }

   private void computeAngularAccelerationBlock(RotationMatrix rotationFromEstimationToMeasurement)
   {
      // r
      tempFramePoint.setIncludingFrame(centerOfMassPositionPort.getData());
      tempFramePoint.changeFrame(ReferenceFrame.getWorldFrame());
      tempFrameVector.setIncludingFrame(tempFramePoint);

      // r - p_{i}^{w}
      tempFramePoint.setToZero(measurementFrame);
      tempFramePoint.changeFrame(ReferenceFrame.getWorldFrame());
      tempFrameVector.sub(tempFramePoint);

      // R_{w}^{p} (r - p_{i}^{w})
      tempFrameVector.changeFrame(estimationFrame);

      // \tilde{r^{p} - p_{i}^{p}}
      tempMatrix.setToTildeForm(tempFrameVector.getVector());

      // R_{p}^{i} \tilde{r^{p} - p_{i}^{p}}
      tempMatrix.preMultiply(rotationFromEstimationToMeasurement);

      tempMatrix.get(getOutputMatrixBlock(angularAccelerationPort));
   }

   private void computeBiasBlock()
   {
      CommonOps.setIdentity(getOutputMatrixBlock(biasPort));
   }
   
   public DenseMatrix64F computeResidual()
   {
      FullInverseDynamicsStructure inverseDynamicsStructure = inverseDynamicsStructureInputPort.getData();
      
      computeBiasedEstimatedMeasurement(inverseDynamicsStructure.getSpatialAccelerationCalculator(), estimatedMeasurement);    // TODO: repeated computation
      tempVector.set(linearAccelerationMeasurementInputPort.getData());
      tempVector.sub(estimatedMeasurement.getVector());

      tempVector.get(residual);

      return residual;
   }

   private void computeBiasedEstimatedMeasurement(SpatialAccelerationCalculator spatialAccelerationCalculator, FrameVector estimatedMeasurement)
   {
      computeUnbiasedEstimatedMeasurement(spatialAccelerationCalculator, estimatedMeasurement);
      estimatedMeasurement.add(biasPort.getData());
   }

   private void computeUnbiasedEstimatedMeasurement(SpatialAccelerationCalculator spatialAccelerationCalculator, FrameVector estimatedMeasurement)
   {
      tempFramePoint.setToZero(measurementFrame);
      RigidBody rootBody = spatialAccelerationCalculator.getRootBody();
      spatialAccelerationCalculator.getLinearAccelerationOfBodyFixedPoint(estimatedMeasurement, rootBody, measurementLink, tempFramePoint);
      estimatedMeasurement.changeFrame(gravitationalAcceleration.getReferenceFrame());
      estimatedMeasurement.sub(gravitationalAcceleration);
      estimatedMeasurement.changeFrame(measurementFrame);
   }
}
