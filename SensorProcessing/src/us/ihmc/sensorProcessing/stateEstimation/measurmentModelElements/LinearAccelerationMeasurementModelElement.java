package us.ihmc.sensorProcessing.stateEstimation.measurmentModelElements;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.linearAlgebra.MatrixTools;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SpatialAccelerationCalculator;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.utilities.screwTheory.Twist;
import us.ihmc.utilities.screwTheory.TwistCalculator;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

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

   private final ControlFlowInputPort<Vector3d> linearAccelerationMeasurementInputPort;

   private final ControlFlowInputPort<FullInverseDynamicsStructure> inverseDynamicsStructureInputPort;

   private final RigidBody measurementLink;
   private final ReferenceFrame measurementFrame;

   private final RigidBody estimationLink;
   private final ReferenceFrame estimationFrame;

   private final DenseMatrix64F residual = new DenseMatrix64F(SIZE, 1);

   // intermediate result stuff:
   private final Matrix3d rotationFromEstimationToWorld = new Matrix3d();
   private final Matrix3d rotationFromEstimationToMeasurement = new Matrix3d();
   private final FrameVector omegaEstimationToMeasurement = new FrameVector(ReferenceFrame.getWorldFrame());
   private final FrameVector vEstimationToMeasurement = new FrameVector(ReferenceFrame.getWorldFrame());
   private final RigidBodyTransform tempTransform = new RigidBodyTransform();
   private final Matrix3d tempMatrix = new Matrix3d();
   private final Vector3d tempVector = new Vector3d();
   private final FramePoint tempFramePoint = new FramePoint(ReferenceFrame.getWorldFrame());
   private final Twist twistOfEstimationLink = new Twist();
   private final Twist twistOfMeasurementFrameWithRespectToEstimation = new Twist();
   private final FrameVector tempFrameVector = new FrameVector(ReferenceFrame.getWorldFrame());
   private final FrameVector gravitationalAcceleration = new FrameVector(ReferenceFrame.getWorldFrame());

   private final Matrix3d omegaJOmega = new Matrix3d();
   private final Matrix3d omegaJV = new Matrix3d();
   private final Matrix3d omegaJOmegad = new Matrix3d();
   private final Matrix3d omegaJVd = new Matrix3d();

   private final Matrix3d phiJPhi = new Matrix3d();
   private final Matrix3d phiJOmega = new Matrix3d();
   private final Matrix3d phiJV = new Matrix3d();
   private final Matrix3d phiJOmegad = new Matrix3d();
   private final Matrix3d phiJVd = new Matrix3d();
   private final Matrix3d phiJP = new Matrix3d();

   private final LinearAccelerationMeasurementModelJacobianAssembler jacobianAssembler;

   private final FrameVector estimatedMeasurement = new FrameVector();

   private final Twist twistOfMeasurementLink = new Twist();
   private final SpatialAccelerationVector spatialAccelerationOfMeasurementLink = new SpatialAccelerationVector();


   public LinearAccelerationMeasurementModelElement(String name, YoVariableRegistry registry, ControlFlowOutputPort<FramePoint> centerOfMassPositionPort,
           ControlFlowOutputPort<FrameVector> centerOfMassVelocityPort, ControlFlowOutputPort<FrameVector> centerOfMassAccelerationPort,
           ControlFlowOutputPort<FrameOrientation> orientationPort, ControlFlowOutputPort<FrameVector> angularVelocityPort,
           ControlFlowOutputPort<FrameVector> angularAccelerationPort, ControlFlowOutputPort<FrameVector> biasPort, ControlFlowInputPort<Vector3d> linearAccelerationMeasurementInputPort,
           ControlFlowInputPort<FullInverseDynamicsStructure> inverseDynamicsStructureInputPort, 
           RigidBody measurementLink,
           ReferenceFrame measurementFrame, RigidBody estimationLink, ReferenceFrame estimationFrame, double gZ)
   {
      super(SIZE, name, registry);
      MathTools.checkIfInRange(gZ, Double.NEGATIVE_INFINITY, 0.0);

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
      tempTransform.get(rotationFromEstimationToWorld);

      // R_{p}^{m}
      estimationFrame.getTransformToDesiredFrame(tempTransform, measurementFrame);
      tempTransform.get(rotationFromEstimationToMeasurement);

      // T_{i}^{p,p}
      twistCalculator.packRelativeTwist(twistOfMeasurementFrameWithRespectToEstimation, estimationLink, measurementLink);

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
      twistCalculator.packTwistOfBody(twistOfEstimationLink, estimationLink);
      twistOfEstimationLink.changeFrame(estimationFrame);
      
      // \dot{r}^{p} = R_{w}^{p} \dot{r} - \tilde{\omega}r^{p} - v_{p}^{p,w}
      rPdToPack.setIncludingFrame(rd);
      rPdToPack.changeFrame(estimationFrame);
      twistOfEstimationLink.packAngularPart(tempFrameVector);
      tempFrameVector.cross(tempFrameVector, rP);
      rPdToPack.sub(tempFrameVector);
      twistOfEstimationLink.packLinearPart(tempFrameVector);
      rPdToPack.sub(tempFrameVector);
   }

   private final FrameVector s = new FrameVector();

   private void computeOrientationBlock(TwistCalculator twistCalculator, SpatialAccelerationCalculator spatialAccelerationCalculator,
         Matrix3d rotationFromEstimationToWorld, Twist twistOfMeasurementWithRespectToEstimation, FramePoint rP, FrameVector rPd)
   {
      // TODO: code and computation repeated in LinearAccelerationMeasurementModelJacobianAssembler
      RigidBody elevator = twistCalculator.getRootBody();
      ReferenceFrame elevatorFrame = elevator.getBodyFixedFrame();
      twistCalculator.packRelativeTwist(twistOfMeasurementLink, elevator, measurementLink);
      spatialAccelerationCalculator.packRelativeAcceleration(spatialAccelerationOfMeasurementLink, elevator, measurementLink);
      spatialAccelerationOfMeasurementLink.changeFrame(elevatorFrame, twistOfMeasurementLink, twistOfMeasurementLink);

      twistOfMeasurementWithRespectToEstimation.changeFrame(estimationFrame);
      twistOfMeasurementWithRespectToEstimation.packAngularPart(omegaEstimationToMeasurement);
      twistOfMeasurementWithRespectToEstimation.packLinearPart(vEstimationToMeasurement);

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
      twistOfMeasurementLink.packAngularPart(tempFrameVector);
      MatrixTools.toTildeForm(tempMatrix, tempFrameVector.getVector());
      phiJOmega.mul(tempMatrix, rotationFromEstimationToWorld);
      phiJOmega.negate();

      // dVWWMdPhi
      tempFrameVector.setToZero(estimationFrame);
      tempFrameVector.cross(omegaEstimationToMeasurement, rP);
      tempFrameVector.add(vEstimationToMeasurement);
      tempFrameVector.sub(rPd);
      MatrixTools.toTildeForm(tempMatrix, tempFrameVector.getVector());
      phiJV.mul(rotationFromEstimationToWorld, tempMatrix);
      phiJV.negate();

      MatrixTools.toTildeForm(tempMatrix, r.getPoint());
      tempMatrix.mul(tempMatrix, phiJOmega);
      phiJV.add(tempMatrix);

      // dOmegadWWMdPhi
      spatialAccelerationOfMeasurementLink.packAngularPart(tempVector);
      MatrixTools.toTildeForm(tempMatrix, tempVector);
      phiJOmegad.mul(tempMatrix, rotationFromEstimationToWorld);
      phiJOmegad.negate();

      // dVdWWMdPhi
      spatialAccelerationCalculator.packRelativeAcceleration(spatialAccelerationOfMeasurementLink, elevator, measurementLink);
      twistOfMeasurementWithRespectToEstimation.changeFrame(measurementLink.getBodyFixedFrame());
      twistOfMeasurementLink.changeFrame(measurementLink.getBodyFixedFrame());
      spatialAccelerationOfMeasurementLink.changeFrame(estimationLink.getBodyFixedFrame(), twistOfMeasurementFrameWithRespectToEstimation,
              twistOfMeasurementLink);
      spatialAccelerationOfMeasurementLink.changeFrameNoRelativeMotion(estimationFrame);
      twistOfMeasurementLink.changeFrame(estimationFrame);

      s.setToZero(estimationFrame);
      spatialAccelerationOfMeasurementLink.packAngularPart(tempFrameVector);
      tempFrameVector.cross(tempFrameVector, rP);
      s.add(tempFrameVector);

      spatialAccelerationOfMeasurementLink.packLinearPart(tempFrameVector);
      s.add(tempFrameVector);

      twistOfMeasurementLink.packAngularPart(tempFrameVector);
      tempFrameVector.cross(tempFrameVector, rPd);
      s.add(tempFrameVector);

      twistOfMeasurementLink.packLinearPart(tempFrameVector);
      tempFrameVector.cross(omega, tempFrameVector);
      s.add(tempFrameVector);

      twistOfMeasurementLink.packAngularPart(tempFrameVector);
      tempFrameVector.cross(tempFrameVector, rP);
      tempFrameVector.cross(omega, tempFrameVector);
      s.add(tempFrameVector);
      s.changeFrame(ReferenceFrame.getWorldFrame());

      tempFrameVector.setToZero(ReferenceFrame.getWorldFrame());
      tempFrameVector.sub(rdd, s);
      MatrixTools.toTildeForm(tempMatrix, tempFrameVector.getVector());
      phiJVd.mul(tempMatrix, rotationFromEstimationToWorld);

      MatrixTools.toTildeForm(tempMatrix, r.getPoint());
      tempMatrix.mul(phiJOmegad);
      phiJVd.add(tempMatrix);

      MatrixTools.toTildeForm(tempMatrix, rd.getVector());
      tempMatrix.mul(phiJOmega);
      phiJVd.add(tempMatrix);

      // dPWIdPhi
      tempFrameVector.setIncludingFrame(rP);
      tempFramePoint.setToZero(measurementFrame);
      tempFramePoint.changeFrame(estimationFrame);
      tempFrameVector.sub(tempFramePoint);
      MatrixTools.toTildeForm(tempMatrix, tempFrameVector.getVector());
      phiJP.mul(rotationFromEstimationToWorld, tempMatrix);

      jacobianAssembler.assembleMeasurementJacobian(tempMatrix, phiJPhi, phiJOmega, phiJV, phiJOmegad, phiJVd, phiJP);
      MatrixTools.setDenseMatrixFromMatrix3d(0, 0, tempMatrix, getOutputMatrixBlock(orientationPort));
   }

   private void computeAngularVelocityBlock(Matrix3d rotationFromEstimationToWorld, Twist twistOfMeasurementWithRespectToEstimation, FramePoint rP,
           FrameVector rd, FrameVector rPd)
   {
      twistOfMeasurementWithRespectToEstimation.changeFrame(estimationFrame);
      twistOfMeasurementWithRespectToEstimation.packAngularPart(omegaEstimationToMeasurement);
      twistOfMeasurementWithRespectToEstimation.packLinearPart(vEstimationToMeasurement);

      // dOmegaWWMdOmega
      omegaJOmega.set(rotationFromEstimationToWorld);

      // dVWWMdOmega
      tempFramePoint.setIncludingFrame(centerOfMassPositionPort.getData());
      tempFramePoint.changeFrame(ReferenceFrame.getWorldFrame());
      MatrixTools.toTildeForm(tempMatrix, tempFramePoint.getPoint());
      omegaJV.mul(tempMatrix, rotationFromEstimationToWorld);

      // dOmegadWWMdOmega
      MatrixTools.toTildeForm(tempMatrix, omegaEstimationToMeasurement.getVector());
      omegaJOmegad.mul(rotationFromEstimationToWorld, tempMatrix);
      omegaJOmegad.negate();

      // dVdWWMdOmega
      rP.checkReferenceFrameMatch(estimationFrame);
      MatrixTools.setTildeTimesTilde(tempMatrix, rP.getPoint(), omegaEstimationToMeasurement.getVector());
      omegaJVd.set(tempMatrix);

      rPd.checkReferenceFrameMatch(estimationFrame);
      MatrixTools.toTildeForm(tempMatrix, rPd.getVector());
      omegaJVd.add(tempMatrix);

      MatrixTools.setTildeTimesTilde(tempMatrix, omegaEstimationToMeasurement.getVector(), rP.getPoint());
      omegaJVd.sub(tempMatrix);

      MatrixTools.toTildeForm(tempMatrix, vEstimationToMeasurement.getVector());
      omegaJVd.sub(tempMatrix);

      omegaJVd.mul(rotationFromEstimationToWorld, omegaJVd);

      MatrixTools.toTildeForm(tempMatrix, rd.getVector());
      tempMatrix.mul(rotationFromEstimationToWorld);
      omegaJVd.add(tempMatrix);

      MatrixTools.toTildeForm(tempMatrix, omegaEstimationToMeasurement.getVector());
      tempMatrix.mul(omegaJV, tempMatrix);
      omegaJVd.sub(tempMatrix);

      jacobianAssembler.assembleMeasurementJacobian(tempMatrix, null, omegaJOmega, omegaJV, omegaJOmegad, omegaJVd, null);
      MatrixTools.setDenseMatrixFromMatrix3d(0, 0, tempMatrix, getOutputMatrixBlock(angularVelocityPort));
   }

   private void computeCenterOfMassVelocityBlock()
   {
      tempMatrix.setZero();
      MatrixTools.setDenseMatrixFromMatrix3d(0, 0, tempMatrix, getOutputMatrixBlock(centerOfMassVelocityPort));
   }

   private void computeCenterOfMassAccelerationBlock()
   {
      ReferenceFrame.getWorldFrame().getTransformToDesiredFrame(tempTransform, measurementFrame);
      tempTransform.get(tempMatrix);
      MatrixTools.setDenseMatrixFromMatrix3d(0, 0, tempMatrix, getOutputMatrixBlock(centerOfMassAccelerationPort));
   }

   private void computeAngularAccelerationBlock(Matrix3d rotationFromEstimationToMeasurement)
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
      MatrixTools.toTildeForm(tempMatrix, tempFrameVector.getVector());

      // R_{p}^{i} \tilde{r^{p} - p_{i}^{p}}
      tempMatrix.mul(rotationFromEstimationToMeasurement, tempMatrix);

      MatrixTools.setDenseMatrixFromMatrix3d(0, 0, tempMatrix, getOutputMatrixBlock(angularAccelerationPort));
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

      MatrixTools.insertTuple3dIntoEJMLVector(tempVector, residual, 0);

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
      spatialAccelerationCalculator.packLinearAccelerationOfBodyFixedPoint(estimatedMeasurement, measurementLink, tempFramePoint);
      estimatedMeasurement.changeFrame(gravitationalAcceleration.getReferenceFrame());
      estimatedMeasurement.sub(gravitationalAcceleration);
      estimatedMeasurement.changeFrame(measurementFrame);
   }
}
