package us.ihmc.simulationconstructionset.physics;

import javax.vecmath.Vector3d;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.simulationconstructionset.ExternalForcePoint;

public class ExternalForcePointPIDConstraintToIntegrate extends ExternalForcePointPDConstraintToIntegrate
{
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final DoubleYoVariable integralStiffness;
   private final YoFrameVector yoConnectionPositionIntegratedError;
   private final FrameVector integralForce;

   private final Vector3d tempForce = new Vector3d();

   public ExternalForcePointPIDConstraintToIntegrate(String name, ExternalForcePoint connectionPointA, ExternalForcePoint connectionPointB,
         YoVariableRegistry parentRegistry)
   {
      super(name, connectionPointA, connectionPointB, parentRegistry);

      integralStiffness = new DoubleYoVariable(name + "_IntegralStiffness", registry);
      yoConnectionPositionIntegratedError = new YoFrameVector(name + "_ConnectionPositionIntegratedError", worldFrame, registry);

      integralForce = new FrameVector(worldFrame);
   }

   public void setIntegralStiffness(double integralStiffness)
   {
      this.integralStiffness.set(integralStiffness);
   }

   @Override
   protected void updateClosedJoint()
   {
      super.updateClosedJoint();

      integralForce.scale(integralStiffness.getDoubleValue(), yoConnectionPositionIntegratedError.getFrameTuple());

      connectionPointA.getForce(tempForce);
      tempForce.add(integralForce.getVector());

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
   public DoubleYoVariable[] getOutputVariables()
   {
      return new DoubleYoVariable[] {yoConnectionPositionIntegratedError.getYoX(), yoConnectionPositionIntegratedError.getYoY(),
            yoConnectionPositionIntegratedError.getYoZ()};
   }

}
