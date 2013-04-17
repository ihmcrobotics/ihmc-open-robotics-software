package us.ihmc.sensorProcessing.stateEstimation.measurmentModelElements;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.sensorProcessing.stateEstimation.evaluation.FullInverseDynamicsStructure;
import us.ihmc.utilities.math.MatrixTools;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import javax.media.j3d.Transform3D;
import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

public class PointPositionMeasurementModelElement extends AbstractMeasurementModelElement
{
   private static final int SIZE = 3;

   private final ControlFlowOutputPort<FramePoint> centerOfMassPositionPort;
   private final ControlFlowOutputPort<FrameOrientation> orientationPort;

   private final ControlFlowInputPort<Point3d> pointPositionMeasurementInputPort;

   private final ReferenceFrame estimationFrame;
   private final ControlFlowInputPort<FullInverseDynamicsStructure> inverseDynamicsStructureInputPort;
   private final FramePoint stationaryPoint;

   private final DenseMatrix64F residual = new DenseMatrix64F(SIZE, 1);

   private final Matrix3d rotationFromEstimationToWorld = new Matrix3d();
   private final Transform3D tempTransform = new Transform3D();
   private final Matrix3d tempMatrix = new Matrix3d();
   private final FramePoint tempFramePoint = new FramePoint(ReferenceFrame.getWorldFrame());
   private final Vector3d residualVector = new Vector3d();


   public PointPositionMeasurementModelElement(String name, ControlFlowInputPort<Point3d> pointPositionMeasurementInputPort,
                                               ControlFlowOutputPort<FramePoint> centerOfMassPositionPort,
                                               ControlFlowOutputPort<FrameOrientation> orientationPort, ReferenceFrame estimationFrame,
                                               FramePoint stationaryPoint, ControlFlowInputPort<FullInverseDynamicsStructure> inverseDynamicsStructureInputPort,
                                               YoVariableRegistry registry)
   {
      super(pointPositionMeasurementInputPort, SIZE, name, registry);

      this.centerOfMassPositionPort = centerOfMassPositionPort;
      this.orientationPort = orientationPort;

      this.pointPositionMeasurementInputPort = pointPositionMeasurementInputPort;

      this.estimationFrame = estimationFrame;
      this.stationaryPoint = stationaryPoint;
      this.inverseDynamicsStructureInputPort = inverseDynamicsStructureInputPort;

      outputMatrixBlocks.put(centerOfMassPositionPort, new DenseMatrix64F(SIZE, SIZE));
      outputMatrixBlocks.put(orientationPort, new DenseMatrix64F(SIZE, SIZE));

      computeCenterOfMassPositionStateOutputBlock();
   }

   public void computeMatrixBlocks()
   {
      computeOrientationStateOutputBlock();
   }

   private void computeCenterOfMassPositionStateOutputBlock()
   {
      CommonOps.setIdentity(outputMatrixBlocks.get(centerOfMassPositionPort));
   }

   private void computeOrientationStateOutputBlock()
   {
      estimationFrame.getTransformToDesiredFrame(tempTransform, ReferenceFrame.getWorldFrame());
      tempTransform.get(rotationFromEstimationToWorld);

      // TODO: garbage
      FramePoint centerOfMassPosition = new FramePoint(centerOfMassPositionPort.getData());
      centerOfMassPosition.changeFrame(estimationFrame);

      tempFramePoint.setAndChangeFrame(stationaryPoint);
      tempFramePoint.changeFrame(estimationFrame);
      tempFramePoint.sub(centerOfMassPosition);
      tempFramePoint.scale(-1.0);

      MatrixTools.toTildeForm(tempMatrix, tempFramePoint.getPoint());
      tempMatrix.mul(rotationFromEstimationToWorld, tempMatrix);
      MatrixTools.setDenseMatrixFromMatrix3d(0, 0, tempMatrix, outputMatrixBlocks.get(orientationPort));
   }

   public DenseMatrix64F computeResidual()
   {
      tempFramePoint.setAndChangeFrame(stationaryPoint);
      tempFramePoint.changeFrame(ReferenceFrame.getWorldFrame());

      residualVector.set(pointPositionMeasurementInputPort.getData());
      residualVector.sub(tempFramePoint.getPoint());

      MatrixTools.insertTuple3dIntoEJMLVector(residualVector, residual, 0);

      return residual;
   }
}
