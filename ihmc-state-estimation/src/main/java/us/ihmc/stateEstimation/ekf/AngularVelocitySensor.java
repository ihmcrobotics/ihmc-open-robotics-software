package us.ihmc.stateEstimation.ekf;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class AngularVelocitySensor extends BodyVelocitySensor
{
   public AngularVelocitySensor(String sensorName, double dt, RigidBody body, ReferenceFrame measurementFrame, boolean estimateBias, YoVariableRegistry registry)
   {
      super(sensorName, dt, body, measurementFrame, estimateBias, registry);
   }

   @Override
   protected void packRelevantJacobianPart(DenseMatrix64F relevantPartToPack, DenseMatrix64F fullJacobian)
   {
      CommonOps.extract(fullJacobian, 0, 3, 0, fullJacobian.getNumCols(), relevantPartToPack, 0, 0);
   }
}
