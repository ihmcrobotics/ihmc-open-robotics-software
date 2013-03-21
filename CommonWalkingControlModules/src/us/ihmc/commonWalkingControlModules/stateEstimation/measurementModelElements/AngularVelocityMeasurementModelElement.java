package us.ihmc.commonWalkingControlModules.stateEstimation.measurementModelElements;


import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import com.yobotics.simulationconstructionset.YoVariableRegistry;

import us.ihmc.controlFlow.ControlFlowInputPort;
import us.ihmc.controlFlow.ControlFlowOutputPort;
import us.ihmc.utilities.math.MatrixTools;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.Twist;
import us.ihmc.utilities.screwTheory.TwistCalculator;

public class AngularVelocityMeasurementModelElement extends AbstractMeasurementModelElement
{
   static final int SIZE = 3;
   private final ControlFlowOutputPort<FrameVector> angularVelocityStatePort;
   private final ControlFlowOutputPort<FrameVector> biasStatePort;
   private final ControlFlowInputPort<Vector3d> angularVelocityMeasurementInputPort;
   private final RigidBody orientationEstimationLink;
   private final RigidBody measurementLink;
   private final ReferenceFrame measurementFrame;

   private final TwistCalculator twistCalculator;
   private final DenseMatrix64F residual = new DenseMatrix64F(SIZE, 1);

   // temp stuff
   private final Twist tempTwist = new Twist();
   private final FrameVector relativeAngularVelocity = new FrameVector(ReferenceFrame.getWorldFrame());
   private final Vector3d angularVelocityResidual = new Vector3d();

   public AngularVelocityMeasurementModelElement(ControlFlowOutputPort<FrameVector> angularVelocityStatePort, ControlFlowOutputPort<FrameVector> biasStatePort,
           ControlFlowInputPort<Vector3d> angularVelocityMeasurementInputPort, RigidBody orientationEstimationLink, RigidBody measurementLink,
           ReferenceFrame measurementFrame, TwistCalculator twistCalculator, String name, YoVariableRegistry registry)
   {
      super(SIZE, 2, name, registry);
      this.angularVelocityStatePort = angularVelocityStatePort;
      this.biasStatePort = biasStatePort;
      this.angularVelocityMeasurementInputPort = angularVelocityMeasurementInputPort;
      this.orientationEstimationLink = orientationEstimationLink;
      this.measurementLink = measurementLink;
      this.measurementFrame = measurementFrame;
      this.twistCalculator = twistCalculator;

      outputMatrixBlocks.put(angularVelocityStatePort, new DenseMatrix64F(SIZE, SIZE));
      outputMatrixBlocks.put(biasStatePort, new DenseMatrix64F(SIZE, SIZE));

      computeAngularVelocityStateOutputBlock();
      computeBiasStateOutputBlock();
   }


   private void computeAngularVelocityStateOutputBlock()
   {
      CommonOps.setIdentity(outputMatrixBlocks.get(angularVelocityStatePort));
   }

   private void computeBiasStateOutputBlock()
   {
      CommonOps.setIdentity(outputMatrixBlocks.get(biasStatePort));
   }

   public void computeMatrixBlocks()
   {
      // empty
   }

   public DenseMatrix64F computeResidual()
   {
      Vector3d measuredAngularVelocityVector3d = angularVelocityMeasurementInputPort.getData();
      twistCalculator.packRelativeTwist(tempTwist, orientationEstimationLink, measurementLink);
      tempTwist.packAngularPart(relativeAngularVelocity);
      relativeAngularVelocity.changeFrame(measurementFrame);

      // TODO: garbage generation
      FrameVector predictedAngularVelocityMeasurement = new FrameVector(angularVelocityStatePort.getData());
      predictedAngularVelocityMeasurement.changeFrame(measurementFrame);
      predictedAngularVelocityMeasurement.add(relativeAngularVelocity);
      predictedAngularVelocityMeasurement.add(biasStatePort.getData());

      angularVelocityResidual.set(measuredAngularVelocityVector3d);
      angularVelocityResidual.sub(predictedAngularVelocityMeasurement.getVector());
      MatrixTools.insertTuple3dIntoEJMLVector(measuredAngularVelocityVector3d, residual, 0);

      return residual;
   }

}
