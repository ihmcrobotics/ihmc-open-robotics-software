package us.ihmc.commonWalkingControlModules.controllerCore.parameters;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class YoJointAccelerationIntegrationParameters
{
   private final YoDouble alphaPosition;
   private final YoDouble alphaVelocity;
   private final YoDouble maxPositionError;
   private final YoDouble masVelocityError;
   private final YoDouble velocityReferenceAlpha;
   
   private final YoDouble velocityReference;
   private final YoDouble positionReference;
   private final YoBoolean resetIntegrators;
   private final YoDouble desiredPosition;
   private final YoDouble desiredPositionNotClamped;
   private final YoDouble desiredPositionClamped;
   private final YoDouble desriedPositionClampedJointLimits;
   private final YoDouble desiredVelocity;
   private final YoDouble desiredVelocityNotClamped;
   private final YoDouble desiredVelocityClamped;
   
   
   public YoJointAccelerationIntegrationParameters(String jointName, YoRegistry registry)
   {
      String prefix = "JointAccelerationIntegrationParameters_";
      alphaPosition = new YoDouble(prefix + jointName + "_AlphaPosition", registry);
      alphaVelocity = new YoDouble(prefix + jointName + "_AlphaVelocity", registry);
      maxPositionError = new YoDouble(prefix + jointName + "_MaxPositionError", registry);
      masVelocityError = new YoDouble(prefix + jointName + "_MaxVelocityError", registry);
      velocityReferenceAlpha = new YoDouble(prefix + jointName + "_VelocityReferenceAlpha", registry);

      velocityReference = new YoDouble(prefix + jointName + "_VelocityReference", registry);
      positionReference = new YoDouble(prefix + jointName + "_PositionReference", registry);
      resetIntegrators = new YoBoolean(prefix + jointName + "_ResetIntegrators", registry);

      desiredPosition = new YoDouble(prefix + jointName + "_DesiredPosition", registry);
      desiredPositionNotClamped = new YoDouble(prefix + jointName + "_DesiredPositionNotClamped", registry);
      desiredPositionClamped = new YoDouble(prefix + jointName + "_DesiredPositionClamped", registry);
      desriedPositionClampedJointLimits = new YoDouble(prefix + jointName + "_DesiredPositionClampedJointLimits", registry);
      desiredVelocity = new YoDouble(prefix + jointName +"_DesiredVelocity", registry);
      desiredVelocityNotClamped = new YoDouble(prefix + jointName + "_DesiredVelocityNotClamped", registry);
      desiredVelocityClamped = new YoDouble(prefix + jointName + "_DesiredVelocityClamped", registry);
   }
   
   public void setParameters(double alphaPosition, double alphaVelocity, double maxPositionError, double maxVelocityError,
                             double velocityReferenceAlpha, double velocityReference, double positionReference, boolean resetIntegrators)
   {
      this.alphaPosition.set(alphaPosition);
      this.alphaVelocity.set(alphaVelocity);
      this.maxPositionError.set(maxPositionError);
      this.masVelocityError.set(maxVelocityError);
      this.velocityReferenceAlpha.set(velocityReferenceAlpha);
      this.velocityReference.set(velocityReference);
      this.positionReference.set(positionReference);
      this.resetIntegrators.set(resetIntegrators);
   }
   public void setMaxPositionError(double maxPositionError)
   {
      this.maxPositionError.set(maxPositionError);
   }
   public void setMaxVelocityError(double maxVelocityError)
   {
      this.masVelocityError.set(maxVelocityError);
   }  
   public void setAlphaPosition(double alphaPosition)
   {
      this.alphaPosition.set(alphaPosition);
   }
   public void setAlphaVelocity(double alphaVelocity)
   {
      this.alphaVelocity.set(alphaVelocity);
   }
   public void setVelocityReferenceAlpha(double velocityReferenceAlpha)
   {
      this.velocityReferenceAlpha.set(velocityReferenceAlpha);
   }
   public void setVelocityReference(double velocityReference)
   {
      this.velocityReference.set(velocityReference);
   }
   public void setPositionReference(double positionReference)
   {
      this.positionReference.set(positionReference);
   }
   public void setResetIntegrators(boolean resetIntegrators)
   {
      this.resetIntegrators.set(resetIntegrators);
   }
   public void setDesiredPosition(double desiredPosition)
{
   this.desiredPosition.set(desiredPosition);
}
   public void setDesiredPositionNotClamped(double desiredPositionNotClamped)
   {
      this.desiredPositionNotClamped.set(desiredPositionNotClamped);
   }
   public void setDesiredPositionClamped(double desiredPositionClamped)
   {
      this.desiredPositionClamped.set(desiredPositionClamped);
   }
   public void setDesiredPositionClampedJointLimits(double desiredPositionClampedJointLimits)
   {
      this.desriedPositionClampedJointLimits.set(desiredPositionClampedJointLimits);
   }
   public void setDesiredVelocity (double desiredVelocity)
   {
      this.desiredVelocity.set(desiredVelocity);
   }
   public void setDesiredVelocityNotClamped(double desiredVelocityNotClamped)
   {
      this.desiredVelocityNotClamped.set(desiredVelocityNotClamped);
   }
   public void setDesiredVelocityClamped(double desiredVelocityClamped)
   {
      this.desiredVelocityClamped.set(desiredVelocityClamped);
   }
   /**
    * Getters for the parameters.
    */
      
   public YoDouble getAlphaPosition()
   {
      return alphaPosition;
   }
   public YoDouble getAlphaVelocity()
   {
      return alphaVelocity;
   }
   public YoDouble getMaxPositionError()
   {
      return maxPositionError;
   }
   public YoDouble getMaxVelocityError()
   {
      return masVelocityError;
   }
   public YoDouble getVelocityReferenceAlpha()
   {
      return velocityReferenceAlpha;
   }
   public YoDouble getVelocityReference()
   {
      return velocityReference;
   }
   public YoDouble getPositionReference()
   {
      return positionReference;
   }
   public YoBoolean getResetIntegrators()
   {
      return resetIntegrators;
   }
   public YoDouble getDesiredPositionNotClamped()
   {
      return desiredPositionNotClamped;
   }
   public YoDouble getDesiredPositionClamped()
   {
      return desiredPositionClamped;
   }
   public YoDouble getDesiredVelocityNotClamped()
   {
      return desiredVelocityNotClamped;
   }
   public YoDouble getDesiredVelocityClamped()
   {
      return desiredVelocityClamped;
   }  
   
}
