package us.ihmc.simulationconstructionset.simulatedSensors;

import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.robotics.sensors.ContactBasedFootSwitch;
import us.ihmc.simulationconstructionset.ExternalForcePoint;

public class SimulatedContactBasedFootSwitch implements ContactBasedFootSwitch
{
   private final double DEFAULT_CONTACT_FORCE_THRESHOLD = 0.5;

   private final String name;
   private final ExternalForcePoint forcePoint;
   private final YoBoolean isInContact;
   private final YoDouble contactForceThreshold;

   public SimulatedContactBasedFootSwitch(String name, ExternalForcePoint forcePoint, YoVariableRegistry registry)
   {
      this.name = name;
      this.forcePoint = forcePoint;

      isInContact = new YoBoolean(name, registry);

      contactForceThreshold = new YoDouble(name + "ContactForceThreshold", registry);
      contactForceThreshold.set(DEFAULT_CONTACT_FORCE_THRESHOLD);
   }

   @Override
   public boolean isInContact()
   {
      double force = forcePoint.getYoForce().length();
      isInContact.set(force >= contactForceThreshold.getDoubleValue());

      return isInContact.getBooleanValue();
   }

   @Override
   public void setIsInContact(boolean isInContact)
   {
      this.isInContact.set(isInContact);
   }

   @Override
   public boolean switchFoot()
   {
      return isInContact();
   }

   @Override
   public void reset()
   {
      isInContact.set(false);
   }
}
