package us.ihmc.robotics.sensors;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class ContactSensorHolder
{
   private final ArrayList<ContactSensorDefinition> contactSensorDefinitions = new ArrayList<ContactSensorDefinition>();
   private final HashMap<ContactSensorDefinition, ContactSensor> contactSensors = new HashMap<ContactSensorDefinition, ContactSensor>();
   
   public ContactSensorHolder(List<ContactSensorDefinition> contactSensorDefinitions)
   {
      for(int i = 0; i<contactSensorDefinitions.size(); i++)      
      {
         ContactSensorDefinition contactSensorDefinition = contactSensorDefinitions.get(i);
         this.contactSensorDefinitions.add(contactSensorDefinition);
         contactSensors.put(contactSensorDefinition, new RigidBodyContactSensor(contactSensorDefinition.getSensorName(),
               contactSensorDefinition.getRigidBody(),contactSensorDefinition.getSensorType()));
      }
   }
   
   public ContactSensorType getType(ContactSensorDefinition definition)
   {
      return contactSensors.get(definition).getType();
   }
   
   public void setIsInContact(ContactSensorDefinition definition, boolean isInContact)
   {
      contactSensors.get(definition).setIsInContact(isInContact);
   }
   
   public ContactSensor getByName(String name)
   {
      for(ContactSensorDefinition definition : contactSensors.keySet())
      {
         if(name.equals(definition.getSensorName()))
         {
            return contactSensors.get(definition);
         }
      }
      throw new RuntimeException("Contact sensor not found " + name);
   }
   
   public ContactSensor getByDefinition(ContactSensorDefinition definition)
   {
      return contactSensors.get(definition);
   }
   
   public ArrayList<ContactSensorDefinition> getContactSensorDefinitions()
   {
      return contactSensorDefinitions;
   }
   
   public void set(ContactSensorHolder otherContactSensorHolder)
   {  
      for(int i = 0; i < contactSensorDefinitions.size(); i++)
      {
         final ContactSensorDefinition contactSensorDefinition = contactSensorDefinitions.get(i);
         ContactSensor otherContactSensor = otherContactSensorHolder.getByDefinition(contactSensorDefinition);
         
         contactSensors.get(contactSensorDefinition).setIsInContact(otherContactSensor.isInContact());
      }
   }
}
