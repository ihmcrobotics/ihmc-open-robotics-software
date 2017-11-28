package us.ihmc.sensorProcessing.stateEstimation.measurementModelElements;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.commons.MathTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SpatialAccelerationCalculator;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;

public class LinearAccelerationMeasurementModelElement extends AbstractMeasurementModelElement
{
   private static final int SIZE = 3;

   private final ControlFlowOutputPort<FramePoint3D> centerOfMassPositionPort;
   private final ControlFlowOutputPort<FrameVector3D> centerOfMassVelocityPort;
   private final ControlFlowOutputPort<FrameVector3D> centerOfMassAccelerationPort;
   private final ControlFlowOutputPort<FrameQuaternion> orientationPort;
   private final ControlFlowOutputPort<FrameVector3D> angularVelocityPort;
   private final ControlFlowOutputPort<FrameVector3D> angularAccelerationPort;
   private final ControlFlowOutputPort<FrameVector3D> biasPort;

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
   private final FrameVector3D omegaEstimationToMeasurement = new FrameVector3D(ReferenceFrame.getWorldFrame());
   private final FrameVector3D vEstimationToMeasurement = new FrameVector3D(ReferenceFrame.getWorldFrame());
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final Matrix3D tempMatrix = new Matrix3D();
   private final Vector3D tempVector = new Vector3D();
   private final FramePoint3D tempFramePoint = new FramePoint3D(ReferenceFrame.getWorldFrame());
   private final Twist twistOfEstimationLink = new Twist();
   private final Twist twistOfMeasurementFrameWithRespectToEstimation = new Twist();
   private final FrameVector3D tempFrameVector = new FrameVector3D(ReferenceFrame.getWorldFrame());
   private final FrameVector3D gravitationalAcceleration = new FrameVector3D(ReferenceFrame.getWorldFrame());

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

   private final FrameVector3D estimatedMeasurement = new FrameVector3D();

   private final Twist twistOfMeasurementLink = new Twist();
   private final SpatialAccelerationVector spatialAccelerationOfMeasurementLink = new SpatialAccelerationVector();


   public LinearAccelerationMeasurementModelElement(String name, YoVariableRegistry registry, ControlFlowOutputPort<FramePoint3D> centerOfMassPositionPort,
           ControlFlowOutputPort<FrameVector3D> centerOfMassVelocityPort, ControlFlowOutputPort<FrameVector3D> centerOfMassAccelerationPort,
           ControlFlowOutputPort<FrameQuaternion> orientationPort, ControlFlowOutputPort<FrameVector3D> angularVelocityPort,
           ControlFlowOutputPort<FrameVector3D> angularAccelerationPort, ControlFlowOutputPort<FrameVector3D> biasPort, ControlFlowInputPort<Vector3D> linearAccelerationMeasurementInputPort,
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

   private final FrameVector3D rdTemp = new FrameVector3D();
   private final FramePoint3D rPTemp = new FramePoint3D();
   private final FrameVector3D rPdTemp = new FrameVector3D();
   public void computeMatrixBlocks()
   {
      FullInverseDynamicsStructure inverseDynamicsStructure = inverseDynamicsStructureInputPort.getData();
      SpatialAccelerationCalculator spatialAccelerationCalculator = inverseDynamicsStructure.getSpatialAccelerationCalculator();
      
      computeUnbiasedEstimatedMeasurement(spatialAccelerationCalculator, estimatedMeasurement);

      // R_{w}^{p}
      estimationFrame.getTransformToDesiredFrame(tempTransform, ReferenceFrame.getWorldFrame());
      tempTransform.getRotation(rotationFromEstimationToWorld);

      // R_{p}^{m}
      estimationFrame.getTransformToDesiredFrame(tempTransform, measurementFrame);
      tempTransform.getRotation(rotationFromEstimationToMeasurement);

      // T_{i}^{p,p}
      measurementLink.getBodyFixedFrame().getTwistRelativeToOther(estimationLink.getBodyFixedFrame(), twistOfMeasurementFrameWithRespectToEstimation);

      // r^{p} 
      rPTemp.setIncludingFrame(centerOfMassPositionPort.getData());
      rPTemp.changeFrame(estimationFrame);

      rdTemp.setIncludingFrame(centerOfMassVelocityPort.getData());
      computeRpd(rPdTemp, rPTemp, rdTemp);
      jacobianAssembler.preCompute(estimatedMeasurement.getVector());

      computeCenterOfMassVelocityBlock();
      computeCenterOfMassAccelerationBlock();
      computeOrientationBlock(spatialAccelerationCalculator, rotationFromEstimationToWorld, twistOfMeasurementFrameWithRespectToEstimation, rPTemp, rPdTemp);
      computeAngularVelocityBlock(rotationFromEstimationToWorld, twistOfMeasurementFrameWithRespectToEstimation, rPTemp, rdTemp, rPdTemp);
      computeAngularAccelerationBlock(rotationFromEstimationToMeasurement);
   }

   private void computeRpd(FrameVector3D rPdToPack, FramePoint3D rP, FrameVector3D rd)
   {
      // T_{p}^{p,w}
      estimationLink.getBodyFixedFrame().getTwistOfFrame(twistOfEstimationLink);
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

   private final FrameVector3D s = new FrameVector3D();

   private void computeOrientationBlock(SpatialAccelerationCalculator spatialAccelerationCalculator,
         RotationMatrix rotationFromEstimationToWorld, Twist twistOfMeasurementWithRespectToEstimation, FramePoint3D rP, FrameVector3D rPd)
   {
      // TODO: code and computation repeated in LinearAccelerationMeasurementModelJacobianAssembler
      RigidBody elevator = spatialAccelerationCalculator.getRootBody();
      ReferenceFrame elevatorFrame = elevator.getBodyFixedFrame();
      measurementLink.getBodyFixedFrame().getTwistRelativeToOther(elevatorFrame, twistOfMeasurementLink);
      spatialAccelerationCalculator.getRelativeAcceleration(elevator, measurementLink, spatialAccelerationOfMeasurementLink);
      spatialAccelerationOfMeasurementLink.changeFrame(elevatorFrame, twistOfMeasurementLink, twistOfMeasurementLink);

      twistOfMeasurementWithRespectToEstimation.changeFrame(estimationFrame);
      twistOfMeasurementWithRespectToEstimation.getAngularPart(omegaEstimationToMeasurement);
      twistOfMeasurementWithRespectToEstimation.getLinearPart(vEstimationToMeasurement);

      FramePoint3D r = centerOfMassPositionPort.getData();
      FrameVector3D rd = centerOfMassVelocityPort.getData();
      FrameVector3D rdd = centerOfMassAccelerationPort.getData();

      r.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      rd.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());
      rdd.checkReferenceFrameMatch(ReferenceFrame.getWorldFrame());

      FrameVector3D omega = angularVelocityPort.getData();
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
      spatialAccelerationCalculator.getRelativeAcceleration(elevator, measurementLink, spatialAccelerationOfMeasurementLink);
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

   private void computeAngularVelocityBlock(RotationMatrix rotationFromEstimationToWorld, Twist twistOfMeasurementWithRespectToEstimation, FramePoint3D rP,
           FrameVector3D rd, FrameVector3D rPd)
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

   private void computeBiasedEstimatedMeasurement(SpatialAccelerationCalculator spatialAccelerationCalculator, FrameVector3D estimatedMeasurement)
   {
      computeUnbiasedEstimatedMeasurement(spatialAccelerationCalculator, estimatedMeasurement);
      estimatedMeasurement.add(biasPort.getData());
   }

   private void computeUnbiasedEstimatedMeasurement(SpatialAccelerationCalculator spatialAccelerationCalculator, FrameVector3D estimatedMeasurement)
   {
      tempFramePoint.setToZero(measurementFrame);
      RigidBody rootBody = spatialAccelerationCalculator.getRootBody();
      spatialAccelerationCalculator.getLinearAccelerationOfBodyFixedPoint(rootBody, measurementLink, tempFramePoint, estimatedMeasurement);
      estimatedMeasurement.changeFrame(gravitationalAcceleration.getReferenceFrame());
      estimatedMeasurement.sub(gravitationalAcceleration);
      estimatedMeasurement.changeFrame(measurementFrame);
   }
}
