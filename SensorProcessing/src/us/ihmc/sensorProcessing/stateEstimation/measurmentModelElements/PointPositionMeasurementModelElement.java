package us.ihmc.sensorProcessing.stateEstimation.measurmentModelElements;

import javax.media.j3d.Transform3D;
import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.sensorProcessing.stateEstimation.sensorConfiguration.PointPositionDataObject;
import us.ihmc.utilities.math.MatrixTools;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class PointPositionMeasurementModelElement extends AbstractMeasurementModelElement
{
   private static final int SIZE = 3;

   private final ControlFlowOutputPort<FramePoint> centerOfMassPositionPort;
   private final ControlFlowOutputPort<FrameOrientation> orientationPort;

   private final ControlFlowInputPort<PointPositionDataObject> pointPositionMeasurementInputPort;

   private final ReferenceFrame estimationFrame;
   private final FramePoint stationaryPoint;

   private final DenseMatrix64F residual = new DenseMatrix64F(SIZE, 1);

   private final Matrix3d rotationFromEstimationToWorld = new Matrix3d();
   private final Transform3D tempTransform = new Transform3D();
   private final Matrix3d tempMatrix = new Matrix3d();
   private final FramePoint tempFramePoint = new FramePoint(ReferenceFrame.getWorldFrame());
   private final Point3d tempPosition = new Point3d();
   private final Vector3d residualVector = new Vector3d();


   public PointPositionMeasurementModelElement(String name, ControlFlowInputPort<PointPositionDataObject> pointPositionMeasurementInputPort,
                                               ControlFlowOutputPort<FramePoint> centerOfMassPositionPort,
                                               ControlFlowOutputPort<FrameOrientation> orientationPort, ReferenceFrame estimationFrame,
                                               FramePoint stationaryPoint,
                                               YoVariableRegistry registry)
   {
      super(pointPositionMeasurementInputPort, SIZE, name, registry);

      this.centerOfMassPositionPort = centerOfMassPositionPort;
      this.orientationPort = orientationPort;

      this.pointPositionMeasurementInputPort = pointPositionMeasurementInputPort;
      this.pointPositionMeasurementInputPort.setData(new PointPositionDataObject());
      
      this.estimationFrame = estimationFrame;
      this.stationaryPoint = stationaryPoint;

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

   @Override
   public DenseMatrix64F getMeasurementCovarianceMatrixBlock()
   {
      this.setCovarianceMatrixScaling(pointPositionMeasurementInputPort.getData().getCovarianceScaling());
      return super.getMeasurementCovarianceMatrixBlock();
   }
   
   public DenseMatrix64F computeResidual()
   {
      tempFramePoint.setAndChangeFrame(stationaryPoint);
      tempFramePoint.changeFrame(ReferenceFrame.getWorldFrame());

      pointPositionMeasurementInputPort.getData().getPosition(tempPosition);
      residualVector.set(tempPosition);
      residualVector.sub(tempFramePoint.getPoint());

      MatrixTools.insertTuple3dIntoEJMLVector(residualVector, residual, 0);

      return residual;
   }
}
