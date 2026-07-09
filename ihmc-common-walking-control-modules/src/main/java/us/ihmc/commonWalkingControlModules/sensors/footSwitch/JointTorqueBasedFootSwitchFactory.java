package us.ihmc.commonWalkingControlModules.sensors.footSwitch;

import java.util.Collection;

import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.sensors.FootSwitchFactory;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.robotics.sensors.ForceSensorDataReadOnly;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;

public class JointTorqueBasedFootSwitchFactory implements FootSwitchFactory
{
   private double defaultContactThresholdTorque = 50.0;
   private double defaultHigherContactThresholdTorque = 100.0;
   private double defaultContactThresholdForceLow = 50.0;
   private double defaultContactThresholdForceHigh = 100;
   private double defaultContactCoPThreshold = 0.01;
   private double defaultContactWindowDuration = 0.05; // previous WBCC-specific window size = 0.03s, estimator-specific window size = 0.025s
   private boolean defaultUseJacobianTranspose = false;
   private double defaultHorizontalVelocityThreshold = 0.5;
   private double defaultVerticalVelocityThreshold = 0.125;
   private double defaultVerticalVelocityHighThreshold = 0.3;
   private double defaultJacobianDeterminantSingularityThreshold = 2e-3;

   private DoubleProvider contactThresholdTorque;
   private DoubleProvider higherContactThresholdTorque;
   private DoubleProvider contactForceThresholdLow;
   private DoubleProvider contactForceThresholdHigh;
   private DoubleProvider contactCoPThreshold;
   private BooleanProvider compensateGravity;
   private DoubleProvider horizontalVelocityThreshold;
   private DoubleProvider verticalVelocityThreshold;
   private DoubleProvider verticalVelocityHighThreshold;
   private DoubleProvider jacobianDeterminantSingularityThreshold;
   private BooleanProvider useJacobianTranspose;

   private final String jointDescriptionToCheck;

   public JointTorqueBasedFootSwitchFactory(String jointDescriptionToCheck)
   {
      this.jointDescriptionToCheck = jointDescriptionToCheck;
   }

   /**
    * When determining whether a foot has hit the ground the controller can look at the knee torque.
    * This value is then glitch filtered to verify.
    */
   public void setDefaultContactThresholdTorque(double defaultContactThresholdTorque)
   {
      this.defaultContactThresholdTorque = defaultContactThresholdTorque;
   }

   /**
    * When determining whether a foot has hit the ground the controller can look at the knee torque.
    * This value is a higher threshold which instantaneously triggers the touchdown event.
    */
   public void setDefaultHigherContactThresholdTorque(double defaultHigherContactThresholdTorque)
   {
      this.defaultHigherContactThresholdTorque = defaultHigherContactThresholdTorque;
   }

   public void setDefaultContactWindowDuration(double window)
   {
      this.defaultContactWindowDuration = window;
   }

   /**
    * Call either {@link #setDefaultContactThresholdForceLow} or {@link #setDefaultContactThresholdForceHigh(double)}. This now links to
    * {@link #setDefaultContactThresholdForceLow(double)}
    * @param defaultContactThresholdForce
    */
   @Deprecated
   public void setDefaultContactThresholdForce(double defaultContactThresholdForce)
   {
      setDefaultContactThresholdForceLow(defaultContactThresholdForce);
   }

   public void setDefaultContactThresholdForceLow(double defaultContactThresholdForce)
   {
      this.defaultContactThresholdForceLow = defaultContactThresholdForce;
   }

   public void setDefaultContactThresholdForceHigh(double defaultContactThresholdForce)
   {
      this.defaultContactThresholdForceHigh = defaultContactThresholdForce;
   }

   public void setDefaultCoPThresholdDistance(double defaultContactCoPThreshold)
   {
      this.defaultContactCoPThreshold = defaultContactCoPThreshold;
   }

   public void setDefaultUseJacobianTranspose(boolean defaultUseJacobianTranspose)
   {
      this.defaultUseJacobianTranspose = defaultUseJacobianTranspose;
   }

   public void setDefaultHorizontalVelocityThreshold(double defaultHorizontalVelocityThreshold)
   {
      this.defaultHorizontalVelocityThreshold = defaultHorizontalVelocityThreshold;
   }

   public void setDefaultVerticalVelocityThreshold(double defaultVerticalVelocityThreshold)
   {
      this.defaultVerticalVelocityThreshold = defaultVerticalVelocityThreshold;
   }

   public void setDefaultJacobianDeterminantSingularityThreshold(double defaultJacobianDeterminantSingularityThreshold)
   {
      this.defaultJacobianDeterminantSingularityThreshold = defaultJacobianDeterminantSingularityThreshold;
   }

   @Override
   public FootSwitchInterface newFootSwitch(String namePrefix,
                                            ContactablePlaneBody foot,
                                            Collection<? extends ContactablePlaneBody> otherFeet,
                                            RigidBodyBasics rootBody,
                                            ForceSensorDataReadOnly footForceSensor,
                                            double totalRobotWeight,
                                            DoubleProvider switchDT,
                                            YoRegistry registry)
   {
      if (contactThresholdTorque == null)
      {
         contactThresholdTorque = new DoubleParameter(namePrefix + "ContactThresholdJointTorque", registry, defaultContactThresholdTorque);
         higherContactThresholdTorque = new DoubleParameter(namePrefix + "HigherContactThresholdJointTorque", registry, defaultHigherContactThresholdTorque);
         contactForceThresholdLow = new DoubleParameter(namePrefix + "JacobianTThresholdForceLow", registry, defaultContactThresholdForceLow);
         contactForceThresholdHigh = new DoubleParameter(namePrefix + "JacobianTThresholdForceHigh", registry, defaultContactThresholdForceHigh);
         contactCoPThreshold = new DoubleParameter(namePrefix + "JacobianTThresholdContactCoP", registry, defaultContactCoPThreshold);
         compensateGravity = new BooleanParameter(namePrefix + "JacobianTCompensateGravity", registry, Boolean.parseBoolean(System.getProperty("alex.footSwitch.jtCompensateGravity", "true")));
         useJacobianTranspose = new BooleanParameter(namePrefix + "UseJacobianTranspose", registry, defaultUseJacobianTranspose);
         verticalVelocityThreshold = new DoubleParameter(namePrefix + "VerticalVelocityThreshold", registry, defaultVerticalVelocityThreshold);
         verticalVelocityHighThreshold = new DoubleParameter(namePrefix + "VerticalVelocityHighThreshold", registry, defaultVerticalVelocityHighThreshold);
         horizontalVelocityThreshold = new DoubleParameter(namePrefix + "HorizontalVelocityThreshold", registry, defaultHorizontalVelocityThreshold);
         jacobianDeterminantSingularityThreshold = new DoubleParameter(namePrefix + "JacobianDeterminantSingularityThreshold", registry,
                                                                       defaultJacobianDeterminantSingularityThreshold);
      }

      return new JointTorqueBasedFootSwitch(namePrefix,
                                            jointDescriptionToCheck,
                                            rootBody,
                                            foot,
                                            contactThresholdTorque,
                                            higherContactThresholdTorque,
                                            contactForceThresholdLow,
                                            contactForceThresholdHigh,
                                            contactCoPThreshold,
                                            defaultContactWindowDuration,
                                            switchDT,
                                            compensateGravity,
                                            horizontalVelocityThreshold,
                                            verticalVelocityThreshold,
                                            verticalVelocityHighThreshold,
                                            jacobianDeterminantSingularityThreshold,
                                            useJacobianTranspose,
                                            registry);
   }
}