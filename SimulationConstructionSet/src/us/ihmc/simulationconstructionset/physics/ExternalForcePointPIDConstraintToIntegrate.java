package us.ihmc.simulationconstructionset.physics;

import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.FunctionToIntegrate;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

public class ExternalForcePointPIDConstraintToIntegrate implements FunctionToIntegrate
{
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry;
   private final PIDController pidController;
   private final DoubleYoVariable radialDerivativeGain;

   private final DoubleYoVariable radialDerivativeAction;
   private final DoubleYoVariable radialVelocityErrorMagnitude;

   private final ExternalForcePoint efpA;
   private final ExternalForcePoint efpB;

   private final YoFramePoint yoPointAPosition;
   private final YoFramePoint yoPointBPosition;

   private final YoFrameVector yoConnectionAVelocity;
   private final YoFrameVector yoConnectionBVelocity;

   private final double deltaT;

   // Temporary variables:
   private final Point3d pointAPosition = new Point3d();
   private final Point3d pointBPosition = new Point3d();
   private final Vector3d pointAVelocity = new Vector3d();
   private final Vector3d pointBVelocity = new Vector3d();
   private final Vector3d positionError = new Vector3d();
   private final Vector3d velocityError = new Vector3d();
   private final Vector3d axialVelocityError = new Vector3d();
   private final Vector3d radialVelocityError = new Vector3d();
   private final Vector3d atoBUnitVector = new Vector3d();
   private final Vector3d radialForce = new Vector3d();
   private final Vector3d axialForce = new Vector3d();
   private final Vector3d totalForce = new Vector3d();

   public ExternalForcePointPIDConstraintToIntegrate(String name, ExternalForcePoint efpA, ExternalForcePoint efpB, YoVariableRegistry parentRegistry, double deltaT)
   {
      this(name, efpA, efpB, parentRegistry, 0.0, 0.0, 0.0, 0.0, 0.0, Double.MAX_VALUE, 0.0, deltaT);
   }

   /**
    * PID Function to Integrate to connect to external force points. A PID controller sets a force in the axial direction, i.e. the line connecting the
    * two points, and operates on the separation distance and axial velocity component of the velocity difference. Along the radial plane, there is
    * only damping, set by radialDerivativeGain, since there is no position offset in this direction
    */
   public ExternalForcePointPIDConstraintToIntegrate(String name, ExternalForcePoint efpA, ExternalForcePoint efpB, YoVariableRegistry parentRegistry,
         double proportionalGain, double axialDerivativeGain, double radialDerivativeGain, double integralGain, double maxIntegralError, double maxOutputLimit,
         double integralLeakRatio, double deltaT)
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
      pidController.setDerivativeGain(axialDerivativeGain);
      pidController.setIntegralGain(integralGain);
      pidController.setMaxIntegralError(maxIntegralError);
      pidController.setMaximumOutputLimit(maxOutputLimit);
      pidController.setIntegralLeakRatio(integralLeakRatio);

      this.radialDerivativeGain = new DoubleYoVariable(name + "RadialKd", registry);
      this.radialDerivativeAction = new DoubleYoVariable(name + "Action_D", registry);
      this.radialDerivativeGain.set(radialDerivativeGain);
      this.radialVelocityErrorMagnitude = new DoubleYoVariable(name + "RadialRateErrorMagnitude", registry);

      parentRegistry.addChild(registry);
   }

   public void setProportionalGain(double proportionalGain)
   {
      pidController.setProportionalGain(proportionalGain);
   }

   public void setAxialDerivativeGain(double axialDerivativeGain)
   {
      pidController.setDerivativeGain(axialDerivativeGain);
   }

   public void setRadialDerivativeGain(double radialDerivativeGain)
   {
      this.radialDerivativeGain.set(radialDerivativeGain);
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

      double axialVelocityErrorMagnitude = velocityError.dot(atoBUnitVector);
      axialVelocityError.set(atoBUnitVector);
      axialVelocityError.scale(axialVelocityErrorMagnitude);
      radialVelocityError.set(velocityError);
      radialVelocityError.sub(axialVelocityError);

      double axialForceMagnitude = pidController.compute(positionError.length(), 0.0, axialVelocityErrorMagnitude, 0.0, deltaT);
      axialForce.set(atoBUnitVector);
      axialForce.scale(axialForceMagnitude);

      radialForce.set(radialVelocityError);
      radialForce.scale(- radialDerivativeGain.getDoubleValue());
      radialDerivativeAction.set(radialForce.length());
      radialVelocityErrorMagnitude.set(radialVelocityError.length());

      totalForce.set(axialForce);
      totalForce.add(radialForce);

      efpB.setForce(totalForce);
      totalForce.negate();
      efpA.setForce(totalForce);

      return null;
   }

   private void updatePositionsAndVelocities()
   {
      yoPointAPosition.getPoint(pointAPosition);
      yoPointBPosition.getPoint(pointBPosition);

      yoConnectionAVelocity.get(pointAVelocity);
      yoConnectionBVelocity.get(pointBVelocity);
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
