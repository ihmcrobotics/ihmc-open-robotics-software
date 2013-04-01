package us.ihmc.commonWalkingControlModules.stateEstimation.measurementModelElements;

import javax.media.j3d.Transform3D;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.MatrixTools;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SpatialAccelerationCalculator;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.utilities.screwTheory.Twist;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import com.yobotics.simulationconstructionset.YoVariableRegistry;

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

   private final TwistCalculator twistCalculator;
   private final SpatialAccelerationCalculator spatialAccelerationCalculator;

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
   private final Transform3D tempTransform = new Transform3D();
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
           TwistCalculator twistCalculator, SpatialAccelerationCalculator spatialAccelerationCalculator, RigidBody measurementLink,
           ReferenceFrame measurementFrame, RigidBody estimationLink, ReferenceFrame estimationFrame, double gZ)
   {
      super(SIZE, name, registry);
      MathTools.checkIfInRange(gZ, 0.0, Double.POSITIVE_INFINITY);

      this.centerOfMassPositionPort = centerOfMassPositionPort;
      this.centerOfMassVelocityPort = centerOfMassVelocityPort;
      this.centerOfMassAccelerationPort = centerOfMassAccelerationPort;
      this.orientationPort = orientationPort;
      this.angularVelocityPort = angularVelocityPort;
      this.angularAccelerationPort = angularAccelerationPort;
      this.biasPort = biasPort;

      this.linearAccelerationMeasurementInputPort = linearAccelerationMeasurementInputPort;

      this.twistCalculator = twistCalculator;
      this.spatialAccelerationCalculator = spatialAccelerationCalculator;

      this.measurementLink = measurementLink;
      this.measurementFrame = measurementFrame;

      this.estimationLink = estimationLink;
      this.estimationFrame = estimationFrame;
      this.jacobianAssembler = new LinearAccelerationMeasurementModelJacobianAssembler(twistCalculator, spatialAccelerationCalculator, measurementLink,
              measurementFrame, estimationFrame);

      gravitationalAcceleration.setZ(-gZ);

      outputMatrixBlocks.put(centerOfMassVelocityPort, new DenseMatrix64F(SIZE, SIZE));
      outputMatrixBlocks.put(centerOfMassAccelerationPort, new DenseMatrix64F(SIZE, SIZE));
      outputMatrixBlocks.put(orientationPort, new DenseMatrix64F(SIZE, SIZE));
      outputMatrixBlocks.put(angularVelocityPort, new DenseMatrix64F(SIZE, SIZE));
      outputMatrixBlocks.put(angularAccelerationPort, new DenseMatrix64F(SIZE, SIZE));
      outputMatrixBlocks.put(biasPort, new DenseMatrix64F(SIZE, SIZE));

      computeBiasBlock();
   }

   public void computeMatrixBlocks()
   {
      computeUnbiasedEstimatedMeasurement(estimatedMeasurement);

      // R_{w}^{p}
      estimationFrame.getTransformToDesiredFrame(tempTransform, ReferenceFrame.getWorldFrame());
      tempTransform.get(rotationFromEstimationToWorld);

      // R_{p}^{m}
      estimationFrame.getTransformToDesiredFrame(tempTransform, measurementFrame);
      tempTransform.get(rotationFromEstimationToMeasurement);

      // T_{i}^{p,p}
      twistCalculator.packRelativeTwist(twistOfMeasurementFrameWithRespectToEstimation, estimationLink, measurementLink);

      // r^{p} TODO: garbage
      FramePoint rP = new FramePoint(centerOfMassPositionPort.getData());
      rP.changeFrame(estimationFrame);

      FrameVector rd = new FrameVector(centerOfMassVelocityPort.getData());
      FrameVector rPd = computeRpd(rP, rd);
      jacobianAssembler.preCompute(estimatedMeasurement.getVector());

      computeCenterOfMassVelocityBlock();
      computeCenterOfMassAccelerationBlock();
      computeOrientationBlock(rotationFromEstimationToWorld, twistOfMeasurementFrameWithRespectToEstimation, rP, rPd);
      computeAngularVelocityBlock(rotationFromEstimationToWorld, twistOfMeasurementFrameWithRespectToEstimation, rP, rd, rPd);
      computeAngularAccelerationBlock(rotationFromEstimationToMeasurement);
   }

   private FrameVector computeRpd(FramePoint rP, FrameVector rd)
   {
      // T_{p}^{p,w}
      twistCalculator.packTwistOfBody(twistOfEstimationLink, estimationLink);
      twistOfEstimationLink.changeFrame(estimationFrame);
      
      // \dot{r}^{p} = R_{w}^{p} \dot{r} - \tilde{\omega}r^{p} - v_{p}^{p,w}
      FrameVector rPd = new FrameVector(rd);
      rPd.changeFrame(estimationFrame);
      twistOfEstimationLink.packAngularPart(tempFrameVector);
      tempFrameVector.cross(tempFrameVector, rP);
      rPd.sub(tempFrameVector);
      twistOfEstimationLink.packLinearPart(tempFrameVector);
      rPd.sub(tempFrameVector);

      return rPd;
   }

   private final FrameVector s = new FrameVector();

   private void computeOrientationBlock(Matrix3d rotationFromEstimationToWorld, Twist twistOfMeasurementWithRespectToEstimation, FramePoint rP, FrameVector rPd)
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
      tempFrameVector.setAndChangeFrame(rP);
      tempFramePoint.setToZero(measurementFrame);
      tempFramePoint.changeFrame(estimationFrame);
      tempFrameVector.sub(tempFramePoint);
      MatrixTools.toTildeForm(tempMatrix, tempFrameVector.getVector());
      phiJP.mul(rotationFromEstimationToWorld, tempMatrix);

      jacobianAssembler.assembleMeasurementJacobian(tempMatrix, phiJPhi, phiJOmega, phiJV, phiJOmegad, phiJVd, phiJP);
      MatrixTools.setDenseMatrixFromMatrix3d(0, 0, tempMatrix, outputMatrixBlocks.get(orientationPort));
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
      tempFramePoint.setAndChangeFrame(centerOfMassPositionPort.getData());
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
      MatrixTools.setDenseMatrixFromMatrix3d(0, 0, tempMatrix, outputMatrixBlocks.get(angularVelocityPort));
   }

   private void computeCenterOfMassVelocityBlock()
   {
      tempMatrix.setZero();
      MatrixTools.setDenseMatrixFromMatrix3d(0, 0, tempMatrix, outputMatrixBlocks.get(centerOfMassVelocityPort));
   }

   private void computeCenterOfMassAccelerationBlock()
   {
      ReferenceFrame.getWorldFrame().getTransformToDesiredFrame(tempTransform, measurementFrame);
      tempTransform.get(tempMatrix);
      MatrixTools.setDenseMatrixFromMatrix3d(0, 0, tempMatrix, outputMatrixBlocks.get(centerOfMassAccelerationPort));
   }

   private void computeAngularAccelerationBlock(Matrix3d rotationFromEstimationToMeasurement)
   {
      // r
      tempFramePoint.setAndChangeFrame(centerOfMassPositionPort.getData());
      tempFramePoint.changeFrame(ReferenceFrame.getWorldFrame());
      tempFrameVector.setAndChangeFrame(tempFramePoint);

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

      MatrixTools.setDenseMatrixFromMatrix3d(0, 0, tempMatrix, outputMatrixBlocks.get(angularAccelerationPort));
   }

   private void computeBiasBlock()
   {
      CommonOps.setIdentity(outputMatrixBlocks.get(biasPort));
   }
   
   public DenseMatrix64F computeResidual()
   {
      computeBiasedEstimatedMeasurement(estimatedMeasurement);    // TODO: repeated computation
      tempVector.set(linearAccelerationMeasurementInputPort.getData());
      tempVector.sub(estimatedMeasurement.getVector());

      MatrixTools.insertTuple3dIntoEJMLVector(tempVector, residual, 0);

      return residual;
   }

   private void computeBiasedEstimatedMeasurement(FrameVector estimatedMeasurement)
   {
      computeUnbiasedEstimatedMeasurement(estimatedMeasurement);
      estimatedMeasurement.add(biasPort.getData());
   }

   private void computeUnbiasedEstimatedMeasurement(FrameVector estimatedMeasurement)
   {
      tempFramePoint.setToZero(measurementFrame);
      spatialAccelerationCalculator.packLinearAccelerationOfBodyFixedPoint(estimatedMeasurement, measurementLink, tempFramePoint);
      estimatedMeasurement.changeFrame(gravitationalAcceleration.getReferenceFrame());
      estimatedMeasurement.add(gravitationalAcceleration);
      estimatedMeasurement.changeFrame(measurementFrame);
   }
}
