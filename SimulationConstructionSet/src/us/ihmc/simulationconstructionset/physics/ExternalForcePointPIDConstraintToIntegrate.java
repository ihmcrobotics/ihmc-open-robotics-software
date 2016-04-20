package us.ihmc.simulationconstructionset.physics;

import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.FunctionToIntegrate;

public class ExternalForcePointPIDConstraintToIntegrate implements FunctionToIntegrate
{
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry;
   private final PIDController pidController;

   private final ExternalForcePoint efpA;
   private final ExternalForcePoint efpB;

   private final YoFramePoint yoPointAPosition;
   private final YoFramePoint yoPointBPosition;

   private final YoFrameVector yoConnectionAVelocity;
   private final YoFrameVector yoConnectionBVelocity;

   private final double deltaT;

   // Temporary variables:
   private final FramePoint pointAPosition = new FramePoint(worldFrame);
   private final FramePoint pointBPosition = new FramePoint(worldFrame);
   private final FrameVector pointAVelocity = new FrameVector(worldFrame);
   private final FrameVector pointBVelocity = new FrameVector(worldFrame);
   private final FrameVector positionError = new FrameVector(worldFrame);
   private final FrameVector velocityError = new FrameVector(worldFrame);
   private final FrameVector atoBUnitVector = new FrameVector(worldFrame);
   private final FrameVector totalForce = new FrameVector(worldFrame);

   public ExternalForcePointPIDConstraintToIntegrate(String name, ExternalForcePoint efpA, ExternalForcePoint efpB, YoVariableRegistry parentRegistry, double deltaT)
   {
      this(name, efpA, efpB, parentRegistry, 0.0, 0.0, 0.0, 0.0, Double.MAX_VALUE, 0.0, deltaT);
   }

   public ExternalForcePointPIDConstraintToIntegrate(String name, ExternalForcePoint efpA, ExternalForcePoint efpB, YoVariableRegistry parentRegistry,
         double proportionalGain, double derivativeGain, double integralGain, double maxIntegralError, double maxOutputLimit, double integralLeakRatio, double deltaT)
   {
      registry = new YoVariableRegistry(name);
      pidController = new PIDController(name, registry);

      this.efpA = efpA;
      this.efpB = efpB;

      this.deltaT = deltaT;

      yoPointAPosition = efpA.getYoPosition();
      yoPointBPosition = efpB.getYoPosition();

      yoConnectionAVelocity = efpA.getYoVelocity();
      yoConnectionBVelocity = efpB.getYoVelocity();

      pidController.setProportionalGain(proportionalGain);
      pidController.setDerivativeGain(derivativeGain);
      pidController.setIntegralGain(integralGain);
      pidController.setMaxIntegralError(maxIntegralError);
      pidController.setMaximumOutputLimit(maxOutputLimit);
      pidController.setIntegralLeakRatio(integralLeakRatio);

      parentRegistry.addChild(registry);
   }

   public void setProportionalGain(double proportionalGain)
   {
      pidController.setProportionalGain(proportionalGain);
   }

   public void setDerivativeGain(double derivativeGain)
   {
      pidController.setDerivativeGain(derivativeGain);
   }

   public void setIntegralGain(double integralGain)
   {
      pidController.setIntegralGain(integralGain);
   }

   public void setMaxIntegralError(double maxIntegralError)
   {
      pidController.setMaxIntegralError(maxIntegralError);
   }

   public void setMaxOutputLimit(double maxOutputLimit)
   {
      pidController.setMaximumOutputLimit(maxOutputLimit);
   }

   public void setIntegralLeakRatio(double integralLeakRatio)
   {
      pidController.setIntegralLeakRatio(integralLeakRatio);
   }

   @Override public double[] computeDerivativeVector()
   {
      updatePositionsAndVelocities();
      computeErrors();
      double totalForceMagnitude = pidController.compute(positionError.length(), 0.0, velocityError.length(), 0.0, deltaT);
      totalForce.set(atoBUnitVector);
      totalForce.scale(totalForceMagnitude);

      efpB.setForce(totalForce.getVector());
      totalForce.negate();
      efpA.setForce(totalForce.getVector());

      return null;
   }

   private void updatePositionsAndVelocities()
   {
      yoPointAPosition.getFrameTupleIncludingFrame(pointAPosition);
      yoPointBPosition.getFrameTupleIncludingFrame(pointBPosition);

      yoConnectionAVelocity.getFrameTupleIncludingFrame(pointAVelocity);
      yoConnectionBVelocity.getFrameTupleIncludingFrame(pointBVelocity);
   }

   private void computeErrors()
   {
      positionError.sub(pointBPosition, pointAPosition);
      velocityError.sub(pointBVelocity, pointAVelocity);

      if(positionError.length() > 1e-9)
      {
         atoBUnitVector.set(positionError);
         atoBUnitVector.normalize();
      }
   }


   @Override public int getVectorSize()
   {
      return 0;
   }

   @Override public DoubleYoVariable[] getOutputVariables()
   {
      return new DoubleYoVariable[0];
   }
}
