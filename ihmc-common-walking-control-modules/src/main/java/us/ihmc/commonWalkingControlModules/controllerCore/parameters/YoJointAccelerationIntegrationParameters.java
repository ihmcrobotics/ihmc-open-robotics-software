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
   private final YoDouble desiredPositionNotClamped;
   private final YoDouble desiredPositionClamped;
   private final YoDouble desiredVelocityNotClamped;
   private final YoDouble desiredVelocityClamped;
   
   public YoJointAccelerationIntegrationParameters(String jointName, YoRegistry registry)
   {
      alphaPosition = new YoDouble(jointName + "AlphaPosition", registry);
      alphaVelocity = new YoDouble(jointName + "AlphaVelocity", registry);
      maxPositionError = new YoDouble(jointName + "MaxPositionError", registry);
      masVelocityError = new YoDouble(jointName + "MaxVelocityError", registry);
      velocityReferenceAlpha = new YoDouble(jointName + "VelocityReferenceAlpha", registry);
      
      velocityReference = new YoDouble(jointName + "VelocityReference", registry);
      positionReference = new YoDouble(jointName + "PositionReference", registry);
      resetIntegrators = new YoBoolean(jointName + "ResetIntegrators", registry);
      
      desiredPositionNotClamped = new YoDouble(jointName + "DesiredPositionNotClamped", registry);
      desiredPositionClamped = new YoDouble(jointName + "DesiredPositionClamped", registry);
      desiredVelocityNotClamped = new YoDouble(jointName + "DesiredVelocityNotClamped", registry);
      desiredVelocityClamped = new YoDouble(jointName + "DesiredVelocityClamped", registry);
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
