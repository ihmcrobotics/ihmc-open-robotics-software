package us.ihmc.sensorProcessing.stateEstimation.measurmentModelElements;

import javax.media.j3d.Transform3D;
import javax.vecmath.Matrix3d;
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
import us.ihmc.utilities.screwTheory.AfterJointReferenceFrameNameMap;

import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class PointPositionMeasurementModelElement extends AbstractMeasurementModelElement
{
   public static final int SIZE = 3;

   private final ControlFlowOutputPort<FramePoint> centerOfMassPositionPort;
   private final ControlFlowOutputPort<FrameOrientation> orientationPort;

   private final ControlFlowInputPort<PointPositionDataObject> pointPositionMeasurementInputPort;

   private final ReferenceFrame estimationFrame;

   private final DenseMatrix64F residual = new DenseMatrix64F(SIZE, 1);

   private final Matrix3d rotationFromEstimationToWorld = new Matrix3d();
   private final Transform3D tempTransform = new Transform3D();
   private final Matrix3d tempMatrix = new Matrix3d();
   private final FramePoint tempFramePoint = new FramePoint(ReferenceFrame.getWorldFrame());
   private final Vector3d residualVector = new Vector3d();

   private final AfterJointReferenceFrameNameMap referenceFrameMap;

   public PointPositionMeasurementModelElement(String name, ControlFlowInputPort<PointPositionDataObject> pointPositionMeasurementInputPort,
         ControlFlowOutputPort<FramePoint> centerOfMassPositionPort, ControlFlowOutputPort<FrameOrientation> orientationPort, ReferenceFrame estimationFrame,
         AfterJointReferenceFrameNameMap referenceFrameMap, YoVariableRegistry registry)
   {
      super(SIZE, name, registry);

      this.centerOfMassPositionPort = centerOfMassPositionPort;
      this.orientationPort = orientationPort;

      this.pointPositionMeasurementInputPort = pointPositionMeasurementInputPort;

      this.estimationFrame = estimationFrame;
      this.referenceFrameMap = referenceFrameMap;

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

      ReferenceFrame referenceFrame = referenceFrameMap.getFrameByName(pointPositionMeasurementInputPort.getData().getBodyFixedReferenceFrameName());
      tempFramePoint.setAndChangeFrame(referenceFrame, pointPositionMeasurementInputPort.getData().getMeasurementPointInBodyFrame());

      tempFramePoint.changeFrame(estimationFrame);
      tempFramePoint.sub(centerOfMassPosition);
      tempFramePoint.scale(-1.0);

      MatrixTools.toTildeForm(tempMatrix, tempFramePoint.getPoint());
      tempMatrix.mul(rotationFromEstimationToWorld, tempMatrix);
      MatrixTools.setDenseMatrixFromMatrix3d(0, 0, tempMatrix, outputMatrixBlocks.get(orientationPort));
   }

   public DenseMatrix64F computeResidual()
   {
      PointPositionDataObject data = pointPositionMeasurementInputPort.getData();
      ReferenceFrame referenceFrame = referenceFrameMap.getFrameByName(pointPositionMeasurementInputPort.getData().getBodyFixedReferenceFrameName());
      tempFramePoint.setAndChangeFrame(referenceFrame, data.getMeasurementPointInBodyFrame());
      tempFramePoint.changeFrame(ReferenceFrame.getWorldFrame());

      residualVector.set(data.getMeasurementPointInWorldFrame());
      residualVector.sub(tempFramePoint.getPoint());

      MatrixTools.insertTuple3dIntoEJMLVector(residualVector, residual, 0);

      return residual;
   }

   public ControlFlowInputPort<PointPositionDataObject> getPointPositionMeasurementInputPort()
   {
      return pointPositionMeasurementInputPort;
   }
}
