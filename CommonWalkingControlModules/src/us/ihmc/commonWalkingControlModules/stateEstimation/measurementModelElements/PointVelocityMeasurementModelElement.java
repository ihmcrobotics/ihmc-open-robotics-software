package us.ihmc.commonWalkingControlModules.stateEstimation.measurementModelElements;

import javax.media.j3d.Transform3D;
import javax.vecmath.Matrix3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.utilities.math.MatrixTools;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.Twist;
import us.ihmc.utilities.screwTheory.TwistCalculator;

import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class PointVelocityMeasurementModelElement extends AbstractMeasurementModelElement
{
   private static final int SIZE = 3;

   private final ControlFlowOutputPort<FramePoint> centerOfMassPositionPort;
   private final ControlFlowOutputPort<FrameVector> centerOfMassVelocityPort;
   private final ControlFlowOutputPort<FrameOrientation> orientationPort;
   private final ControlFlowOutputPort<FrameVector> angularVelocityPort;

   private final ControlFlowInputPort<FrameVector> pointVelocityMeasurementInputPort;

   private final ReferenceFrame estimationFrame;
   private final TwistCalculator twistCalculator;
   private final FramePoint stationaryPoint;

   private final RigidBody stationaryPointLink;
   private final DenseMatrix64F residual = new DenseMatrix64F(SIZE, 1);

   private final Matrix3d rotationFromPelvisToWorld = new Matrix3d();
   private final Transform3D tempTransform = new Transform3D();
   private final Matrix3d tempMatrix = new Matrix3d();
   private final FramePoint tempFramePoint = new FramePoint(ReferenceFrame.getWorldFrame());
   private final Twist tempTwist = new Twist();
   private final FrameVector tempFrameVector = new FrameVector(ReferenceFrame.getWorldFrame());
   private final FrameVector residualVector = new FrameVector(ReferenceFrame.getWorldFrame());


   public PointVelocityMeasurementModelElement(String name, ControlFlowInputPort<FrameVector> pointVelocityMeasurementInputPort,
           ControlFlowOutputPort<FramePoint> centerOfMassPositionPort, ControlFlowOutputPort<FrameVector> centerOfMassVelocityPort,
           ControlFlowOutputPort<FrameOrientation> orientationPort, ControlFlowOutputPort<FrameVector> angularVelocityPort, ReferenceFrame estimationFrame,
           RigidBody stationaryPointLink, FramePoint stationaryPoint, TwistCalculator twistCalculator, YoVariableRegistry registry)
   {
      super(SIZE, 3, name, registry);

      this.centerOfMassPositionPort = centerOfMassPositionPort;
      this.centerOfMassVelocityPort = centerOfMassVelocityPort;
      this.orientationPort = orientationPort;
      this.angularVelocityPort = angularVelocityPort;

      this.pointVelocityMeasurementInputPort = pointVelocityMeasurementInputPort;

      this.estimationFrame = estimationFrame;
      this.stationaryPointLink = stationaryPointLink;
      this.stationaryPoint = stationaryPoint;
      this.twistCalculator = twistCalculator;

      outputMatrixBlocks.put(centerOfMassVelocityPort, new DenseMatrix64F(SIZE, SIZE));
      outputMatrixBlocks.put(orientationPort, new DenseMatrix64F(SIZE, SIZE));
      outputMatrixBlocks.put(angularVelocityPort, new DenseMatrix64F(SIZE, SIZE));

      computeCenterOfMassVelocityStateOutputBlock();
   }

   public void computeMatrixBlocks()
   {
      estimationFrame.getTransformToDesiredFrame(tempTransform, ReferenceFrame.getWorldFrame());
      tempTransform.get(rotationFromPelvisToWorld);
      computeOrientationStateOutputBlock(rotationFromPelvisToWorld);
      computeAngularVelocityStateOutputBlock(rotationFromPelvisToWorld);
   }

   private void computeCenterOfMassVelocityStateOutputBlock()
   {
      CommonOps.setIdentity(outputMatrixBlocks.get(centerOfMassVelocityPort));
   }

   private void computeAngularVelocityStateOutputBlock(Matrix3d rotationFromPelvisToWorld)
   {
      computeVelocityOfStationaryPoint(tempFrameVector);
      tempFrameVector.changeFrame(estimationFrame);

      // TODO: garbage
      FrameVector centerOfMassVelocity = new FrameVector(centerOfMassVelocityPort.getData());
      tempFrameVector.sub(centerOfMassVelocity);
      tempFrameVector.scale(-1.0);

      MatrixTools.toTildeForm(tempMatrix, tempFramePoint.getPoint());
      tempMatrix.mul(rotationFromPelvisToWorld, tempMatrix);
      MatrixTools.setDenseMatrixFromMatrix3d(0, 0, tempMatrix, outputMatrixBlocks.get(angularVelocityPort));
   }

   private void computeOrientationStateOutputBlock(Matrix3d rotationFromPelvisToWorld)
   {
      // TODO: garbage
      FramePoint centerOfMassPosition = new FramePoint(centerOfMassPositionPort.getData());
      centerOfMassPosition.changeFrame(estimationFrame);

      tempFramePoint.set(stationaryPoint);
      tempFramePoint.changeFrame(estimationFrame);
      tempFramePoint.sub(centerOfMassPosition);
      tempFramePoint.scale(-1.0);

      MatrixTools.toTildeForm(tempMatrix, tempFramePoint.getPoint());
      tempMatrix.mul(rotationFromPelvisToWorld, tempMatrix);
      MatrixTools.setDenseMatrixFromMatrix3d(0, 0, tempMatrix, outputMatrixBlocks.get(orientationPort));
   }

   public DenseMatrix64F computeResidual()
   {
      computeVelocityOfStationaryPoint(tempFrameVector);
      tempFrameVector.changeFrame(ReferenceFrame.getWorldFrame());

      residualVector.set(pointVelocityMeasurementInputPort.getData());
      residualVector.sub(tempFrameVector);

      MatrixTools.insertTuple3dIntoEJMLVector(residualVector.getVector(), residual, 0);

      return residual;
   }

   private void computeVelocityOfStationaryPoint(FrameVector stationaryPointVelocityToPack)
   {
      twistCalculator.packTwistOfBody(tempTwist, stationaryPointLink);
      tempTwist.changeFrame(tempTwist.getBaseFrame());
      tempFramePoint.setAndChangeFrame(stationaryPoint);
      tempFramePoint.changeFrame(tempTwist.getBaseFrame());
      tempTwist.packVelocityOfPointFixedInBodyFrame(stationaryPointVelocityToPack, tempFramePoint);
   }

}
