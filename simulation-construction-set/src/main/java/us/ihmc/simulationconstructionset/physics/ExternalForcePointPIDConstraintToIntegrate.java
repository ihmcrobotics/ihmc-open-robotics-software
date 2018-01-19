package us.ihmc.simulationconstructionset.physics;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.simulationconstructionset.ExternalForcePoint;

public class ExternalForcePointPIDConstraintToIntegrate extends ExternalForcePointPDConstraintToIntegrate
{
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoDouble integralStiffness;
   private final YoFrameVector yoConnectionPositionIntegratedError;
   private final FrameVector3D integralForce;

   private final Vector3D tempForce = new Vector3D();

   public ExternalForcePointPIDConstraintToIntegrate(String name, ExternalForcePoint connectionPointA, ExternalForcePoint connectionPointB,
         YoVariableRegistry parentRegistry)
   {
      super(name, connectionPointA, connectionPointB, parentRegistry);

      integralStiffness = new YoDouble(name + "_IntegralStiffness", registry);
      yoConnectionPositionIntegratedError = new YoFrameVector(name + "_ConnectionPositionIntegratedError", worldFrame, registry);

      integralForce = new FrameVector3D(worldFrame);
   }

   public void setIntegralStiffness(double integralStiffness)
   {
      this.integralStiffness.set(integralStiffness);
   }

   @Override
   protected void updateClosedJoint()
   {
      super.updateClosedJoint();

      integralForce.setAndScale(integralStiffness.getDoubleValue(), yoConnectionPositionIntegratedError);

      connectionPointA.getForce(tempForce);
      tempForce.add(integralForce);

      connectionPointA.setForce(tempForce);
      tempForce.scale(-1.0);
      connectionPointB.setForce(tempForce);
   }

   @Override
   public double[] computeDerivativeVector()
   {
      updateClosedJoint();
      return new double[] {yoConnectionPositionError.getX(), yoConnectionPositionError.getY(), yoConnectionPositionError.getZ()};
   }

   @Override
   public int getVectorSize()
   {
      return 3;
   }

   @Override
   public YoDouble[] getOutputVariables()
   {
      return new YoDouble[] {yoConnectionPositionIntegratedError.getYoX(), yoConnectionPositionIntegratedError.getYoY(),
            yoConnectionPositionIntegratedError.getYoZ()};
   }

}
