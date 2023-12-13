package us.ihmc.commonWalkingControlModules.sensors.footSwitch;

import java.util.Collection;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.frames.MovingReferenceFrame;
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
import us.ihmc.yoVariables.variable.YoInteger;

public class JointTorqueBasedFootSwitchFactory implements FootSwitchFactory
{
   private double defaultContactThresholdTorque = 50.0;
   private double defaultHigherContactThresholdTorque = 100.0;
   private double defaultContactThresholdForce = 50.0;
   private int defaultContactWindowSize = 5;
   private boolean defaultUseJacobianTranspose = false;
   private double defaultHorizontalVelocityThreshold = 0.25;
   private double defaultVerticalVelocityThreshold = 0.1;

   private DoubleProvider contactThresholdTorque;
   private DoubleProvider higherContactThresholdTorque;
   private DoubleProvider contactForceThreshold;
   private BooleanProvider compensateGravity;
   private DoubleProvider horizontalVelocityThreshold;
   private DoubleProvider verticalVelocityThreshold;
   private YoInteger contactWindowSize;
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

   public void setDefaultContactWindowSize(int defaultContactWindowSize)
   {
      this.defaultContactWindowSize = defaultContactWindowSize;
   }

   public void setDefaultContactThresholdForce(double defaultContactThresholdForce)
   {
      this.defaultContactThresholdForce = defaultContactThresholdForce;
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

   @Override
   public FootSwitchInterface newFootSwitch(String namePrefix,
                                            ContactablePlaneBody foot,
                                            Collection<? extends ContactablePlaneBody> otherFeet,
                                            RigidBodyBasics rootBody,
                                            ForceSensorDataReadOnly footForceSensor,
                                            double totalRobotWeight,
                                            YoGraphicsListRegistry yoGraphicsListRegistry,
                                            YoRegistry registry)
   {
      if (contactThresholdTorque == null)
      {
         contactThresholdTorque = new DoubleParameter(namePrefix + "ContactThresholdJointTorque", registry, defaultContactThresholdTorque);
         higherContactThresholdTorque = new DoubleParameter(namePrefix + "HigherContactThresholdJointTorque", registry, defaultHigherContactThresholdTorque);
         contactForceThreshold = new DoubleParameter(namePrefix + "JacobianTThresholdForce", registry, defaultContactThresholdForce);
         contactWindowSize = new YoInteger(namePrefix + "ContactThresholdJointTorqueWindowSize", registry);
         contactWindowSize.set(defaultContactWindowSize);
         compensateGravity = new BooleanParameter(namePrefix + "JacobianTCompensateGravity", registry, true);
         useJacobianTranspose = new BooleanParameter(namePrefix + "UseJacobianTranspose", registry, defaultUseJacobianTranspose);
         verticalVelocityThreshold = new DoubleParameter(namePrefix + "VerticalVelocityThreshold", registry, defaultVerticalVelocityThreshold);
         horizontalVelocityThreshold = new DoubleParameter(namePrefix + "HorizontalVelocityThreshold", registry, defaultHorizontalVelocityThreshold);
      }

      return new JointTorqueBasedFootSwitch(namePrefix,
                                            jointDescriptionToCheck,
                                            foot.getRigidBody(),
                                            rootBody,
                                            (MovingReferenceFrame) foot.getSoleFrame(),
                                            contactThresholdTorque,
                                            higherContactThresholdTorque,
                                            contactForceThreshold,
                                            contactWindowSize,
                                            compensateGravity,
                                            horizontalVelocityThreshold,
                                            verticalVelocityThreshold,
                                            useJacobianTranspose,
                                            registry);
   }
}
