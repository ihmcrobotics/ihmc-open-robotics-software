package us.ihmc.sensorProcessing.stateEstimation.measurmentModelElements;

import javax.media.j3d.Transform3D;
import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.PointVelocityDataObject;
import us.ihmc.utilities.math.MatrixTools;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.Twist;

import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class PointVelocityMeasurementModelElement extends AbstractMeasurementModelElement
{
   private static final int SIZE = 3;

   private final ControlFlowOutputPort<FramePoint> centerOfMassPositionPort;
   private final ControlFlowOutputPort<FrameVector> centerOfMassVelocityPort;
   private final ControlFlowOutputPort<FrameOrientation> orientationPort;
   private final ControlFlowOutputPort<FrameVector> angularVelocityPort;

   private final ControlFlowInputPort<PointVelocityDataObject> pointVelocityMeasurementInputPort;

   private final ReferenceFrame estimationFrame;
   private final ControlFlowInputPort<FullInverseDynamicsStructure> inverseDynamicsStructureInputPort;
   private final FramePoint stationaryPoint;

   private final RigidBody stationaryPointLink;
   private final DenseMatrix64F residual = new DenseMatrix64F(SIZE, 1);

   private final Matrix3d rotationFromEstimationToWorld = new Matrix3d();
   private final Transform3D tempTransform = new Transform3D();
   private final Matrix3d tempMatrix = new Matrix3d();
   private final FramePoint tempFramePoint = new FramePoint(ReferenceFrame.getWorldFrame());
   private final Twist tempTwist = new Twist();
   private final FrameVector tempFrameVector = new FrameVector(ReferenceFrame.getWorldFrame());
   private final Vector3d tempVector = new Vector3d();
   private final FrameVector residualVector = new FrameVector(ReferenceFrame.getWorldFrame());


   public PointVelocityMeasurementModelElement(String name, ControlFlowInputPort<PointVelocityDataObject> pointVelocityMeasurementInputPort,
           ControlFlowOutputPort<FramePoint> centerOfMassPositionPort, ControlFlowOutputPort<FrameVector> centerOfMassVelocityPort,
           ControlFlowOutputPort<FrameOrientation> orientationPort, ControlFlowOutputPort<FrameVector> angularVelocityPort, ReferenceFrame estimationFrame,
           RigidBody stationaryPointLink, FramePoint stationaryPoint, ControlFlowInputPort<FullInverseDynamicsStructure> inverseDynamicsStructureInputPort,
           YoVariableRegistry registry)
   {
      super(SIZE, name, registry);

      this.centerOfMassPositionPort = centerOfMassPositionPort;
      this.centerOfMassVelocityPort = centerOfMassVelocityPort;
      this.orientationPort = orientationPort;
      this.angularVelocityPort = angularVelocityPort;

      this.pointVelocityMeasurementInputPort = pointVelocityMeasurementInputPort;
      this.pointVelocityMeasurementInputPort.setData(new PointVelocityDataObject());

      this.estimationFrame = estimationFrame;
      this.stationaryPointLink = stationaryPointLink;
      this.stationaryPoint = stationaryPoint;
      this.inverseDynamicsStructureInputPort = inverseDynamicsStructureInputPort;

      outputMatrixBlocks.put(centerOfMassVelocityPort, new DenseMatrix64F(SIZE, SIZE));
      outputMatrixBlocks.put(orientationPort, new DenseMatrix64F(SIZE, SIZE));
      outputMatrixBlocks.put(angularVelocityPort, new DenseMatrix64F(SIZE, SIZE));

      computeCenterOfMassVelocityStateOutputBlock();
   }

   public void computeMatrixBlocks()
   {
      estimationFrame.getTransformToDesiredFrame(tempTransform, ReferenceFrame.getWorldFrame());
      tempTransform.get(rotationFromEstimationToWorld);
      computeOrientationStateOutputBlock(rotationFromEstimationToWorld);
      computeAngularVelocityStateOutputBlock(rotationFromEstimationToWorld);
   }

   private void computeCenterOfMassVelocityStateOutputBlock()
   {
      CommonOps.setIdentity(outputMatrixBlocks.get(centerOfMassVelocityPort));
   }

   private void computeAngularVelocityStateOutputBlock(Matrix3d rotationFromPelvisToWorld)
   {
      // TODO: garbage
      FramePoint centerOfMassPosition = new FramePoint(centerOfMassPositionPort.getData());
      centerOfMassPosition.changeFrame(estimationFrame);

      tempFramePoint.setAndChangeFrame(stationaryPoint);
      tempFramePoint.changeFrame(estimationFrame);
      tempFramePoint.sub(centerOfMassPosition);
      tempFramePoint.scale(-1.0);

      MatrixTools.toTildeForm(tempMatrix, tempFramePoint.getPoint());
      tempMatrix.mul(rotationFromPelvisToWorld, tempMatrix);
      MatrixTools.setDenseMatrixFromMatrix3d(0, 0, tempMatrix, outputMatrixBlocks.get(angularVelocityPort));
   }

   private void computeOrientationStateOutputBlock(Matrix3d rotationFromPelvisToWorld)
   {
      computeVelocityOfStationaryPoint(tempFrameVector);
      tempFrameVector.changeFrame(estimationFrame);

      // TODO: garbage
      FrameVector centerOfMassVelocity = new FrameVector(centerOfMassVelocityPort.getData());
      centerOfMassVelocity.changeFrame(estimationFrame);
      tempFrameVector.sub(centerOfMassVelocity);
      tempFrameVector.scale(-1.0);

      MatrixTools.toTildeForm(tempMatrix, tempFrameVector.getVector());
      tempMatrix.mul(rotationFromPelvisToWorld, tempMatrix);
      MatrixTools.setDenseMatrixFromMatrix3d(0, 0, tempMatrix, outputMatrixBlocks.get(orientationPort));
   }

   @Override
   public DenseMatrix64F getMeasurementCovarianceMatrixBlock()
   {
      this.setCovarianceMatrixScaling(pointVelocityMeasurementInputPort.getData().getCovarianceScaling());
      return super.getMeasurementCovarianceMatrixBlock();
   }
   
   public DenseMatrix64F computeResidual()
   {
      computeVelocityOfStationaryPoint(tempFrameVector);
      tempFrameVector.changeFrame(ReferenceFrame.getWorldFrame());

      pointVelocityMeasurementInputPort.getData().getVelocity(tempVector);
      residualVector.set(tempVector);
      residualVector.sub(tempFrameVector);

      MatrixTools.insertTuple3dIntoEJMLVector(residualVector.getVector(), residual, 0);

      return residual;
   }

   private void computeVelocityOfStationaryPoint(FrameVector stationaryPointVelocityToPack)
   {
      FullInverseDynamicsStructure inverseDynamicsStructure = inverseDynamicsStructureInputPort.getData();

      inverseDynamicsStructure.getTwistCalculator().packTwistOfBody(tempTwist, stationaryPointLink);
      tempTwist.changeFrame(tempTwist.getBaseFrame());
      tempFramePoint.setAndChangeFrame(stationaryPoint);
      tempFramePoint.changeFrame(tempTwist.getBaseFrame());
      tempTwist.packVelocityOfPointFixedInBodyFrame(stationaryPointVelocityToPack, tempFramePoint);
   }

}
