package us.ihmc.commonWalkingControlModules.stateEstimation.measurementModelElements;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.utilities.math.MatrixTools;
import us.ihmc.utilities.math.geometry.AngleTools;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class OrientationMeasurementModelElement extends AbstractMeasurementModelElement
{
   private static final int SIZE = 3;
   private final ControlFlowOutputPort<FrameOrientation> orientationStatePort;
   private final ControlFlowInputPort<Matrix3d> orientationMeasurementInputPort;

   private final ReferenceFrame estimationFrame;
   private final ReferenceFrame measurementFrame;

   private final DenseMatrix64F residual = new DenseMatrix64F(SIZE, 1);

   // temp stuff:
   private final FrameOrientation orientationOfEstimationInMeasurementFrame = new FrameOrientation(ReferenceFrame.getWorldFrame());
   private final Quat4d quaternion = new Quat4d();
   private final Quat4d estimationFrameToMeasurementFrame = new Quat4d();
   private final Quat4d orientationResidual = new Quat4d();
   private final AxisAngle4d tempAxisAngle = new AxisAngle4d();
   private final Vector3d rotationVectorResidual = new Vector3d();

   public OrientationMeasurementModelElement(ControlFlowOutputPort<FrameOrientation> orientationStatePort,
           ControlFlowInputPort<Matrix3d> orientationMeasurementInputPort, ReferenceFrame estimationFrame, ReferenceFrame measurementFrame, String name,
           YoVariableRegistry registry)
   {
      super(SIZE, 1, name, registry);
      this.orientationStatePort = orientationStatePort;
      this.orientationMeasurementInputPort = orientationMeasurementInputPort;
      this.estimationFrame = estimationFrame;
      this.measurementFrame = measurementFrame;

      outputMatrixBlocks.put(orientationStatePort, new DenseMatrix64F(SIZE, SIZE));
      computeOrientationStateOutputBlock();
   }

   private void computeOrientationStateOutputBlock()
   {
      DenseMatrix64F orientationStateOutputBlock = outputMatrixBlocks.get(orientationStatePort);
      CommonOps.setIdentity(orientationStateOutputBlock);
      CommonOps.scale(-1.0, orientationStateOutputBlock);
   }

   public void computeMatrixBlocks()
   {
      // empty
   }

   public DenseMatrix64F computeResidual()
   {
      orientationStatePort.getData().getQuaternion(quaternion);

      orientationOfEstimationInMeasurementFrame.set(estimationFrame);
      orientationOfEstimationInMeasurementFrame.changeFrame(measurementFrame);
      orientationOfEstimationInMeasurementFrame.getQuaternion(estimationFrameToMeasurementFrame);

      // TODO: garbage generation
      FrameOrientation measuredOrientation = new FrameOrientation(ReferenceFrame.getWorldFrame(), orientationMeasurementInputPort.getData());
      measuredOrientation.getQuaternion(orientationResidual);
      orientationResidual.mul(estimationFrameToMeasurementFrame);
      orientationResidual.inverse();    // conjugate();
      orientationResidual.mul(quaternion);
      tempAxisAngle.set(0.0, 0.0, 0.0, 0.0);    // necessary because set(Quat4d) may not actually set anything if magnitude is small!
      tempAxisAngle.set(orientationResidual);
      rotationVectorResidual.set(tempAxisAngle.getX(), tempAxisAngle.getY(), tempAxisAngle.getZ());
      double angle = AngleTools.trimAngleMinusPiToPi(tempAxisAngle.getAngle());
      rotationVectorResidual.scale(angle);
      MatrixTools.insertTuple3dIntoEJMLVector(rotationVectorResidual, residual, 0);

      return residual;
   }
}
