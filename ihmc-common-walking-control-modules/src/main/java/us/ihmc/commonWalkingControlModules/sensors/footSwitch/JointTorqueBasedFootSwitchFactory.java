package us.ihmc.commonWalkingControlModules.sensors.footSwitch;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.sensors.FootSwitchFactory;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.robotics.sensors.ForceSensorDataReadOnly;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

import java.util.Collection;

public class JointTorqueBasedFootSwitchFactory implements FootSwitchFactory
{
   private double defaultContactThresholdTorque = 50.0;
   private double defaultHigherContactThresholdTorque = 100.0;
   private int defaultContactWindowSize = 5;
   private DoubleProvider contactThresholdTorque;
   private DoubleProvider higherContactThresholdTorque;
   private YoInteger contactWindowSize;

   private final String jointDescriptionToCheck;

   public JointTorqueBasedFootSwitchFactory(String jointDescriptionToCheck)
   {
      this.jointDescriptionToCheck= jointDescriptionToCheck;
   }

   /**
    * When determining whether a foot has hit the ground the controller can look at the knee torque. This value is then glitch filtered to verify.
    */
   public void setDefaultContactThresholdTorque(double defaultContactThresholdTorque)
   {
      this.defaultContactThresholdTorque = defaultContactThresholdTorque;
   }

   /**
    * When determining whether a foot has hit the ground the controller can look at the knee torque. This value is a higher threshold which instantaneously
    * triggers the touchdown event.
    */
   public void setDefaultHigherContactThresholdTorque(double defaultHigherContactThresholdTorque)
   {
      this.defaultHigherContactThresholdTorque = defaultHigherContactThresholdTorque;
   }

   public void setDefaultContactWindowSize(int defaultContactWindowSize)
   {
      this.defaultContactWindowSize = defaultContactWindowSize;
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
         contactWindowSize = new YoInteger(namePrefix + "ContactThresholdJointTorqueWindowSize", registry);
         contactWindowSize.set(defaultContactWindowSize);
      }

      return new JointTorqueBasedFootSwitch(namePrefix, jointDescriptionToCheck, foot.getRigidBody(), rootBody, foot.getSoleFrame(),
                                            contactThresholdTorque, higherContactThresholdTorque, contactWindowSize,
                                            registry);
   }
}
