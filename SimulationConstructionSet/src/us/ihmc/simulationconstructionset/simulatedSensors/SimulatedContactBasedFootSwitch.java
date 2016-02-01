package us.ihmc.simulationconstructionset.simulatedSensors;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.sensors.ContactBasedFootSwitch;
import us.ihmc.simulationconstructionset.ExternalForcePoint;

public class SimulatedContactBasedFootSwitch implements ContactBasedFootSwitch
{
   private final double DEFAULT_CONTACT_FORCE_THRESHOLD = 0.5;

   private final String name;
   private final ExternalForcePoint forcePoint;
   private final BooleanYoVariable isInContact;
   private final DoubleYoVariable contactForceThreshold;

   public SimulatedContactBasedFootSwitch(String name, ExternalForcePoint forcePoint, YoVariableRegistry registry)
   {
      this.name = name;
      this.forcePoint = forcePoint;

      isInContact = new BooleanYoVariable(name, registry);

      contactForceThreshold = new DoubleYoVariable(name + "ContactForceThreshold", registry);
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
