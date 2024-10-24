package us.ihmc.exampleSimulations.exampleContact;

import java.util.ArrayList;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.simulationconstructionset.util.ground.Contactable;
import us.ihmc.simulationconstructionset.util.ground.SimpleStickSlipContactModel;

public class ContactController implements RobotController
{
   private final YoRegistry registry = new YoRegistry("ContactController");
   
   private final SimpleStickSlipContactModel contactModel;
   
   public ContactController()
   {
      contactModel = new SimpleStickSlipContactModel("simpleContact", registry);
      initialize();
   }
   
   public void addContactPoints(ArrayList<ExternalForcePoint> contactPoints)
   {
      for (ExternalForcePoint contactPoint : contactPoints)
      {
         addContactPoint(contactPoint);
      }   
   }
   
   public void addContactPoint(ExternalForcePoint contactPoint)
   {
      contactModel.addContactPoint(contactPoint);
   }
   
   public void addContactables(ArrayList<Contactable> contactables)
   {
      for (Contactable contactable : contactables)
      {
         addContactable(contactable);
      }
   }
   
   public void addContactable(Contactable contactable)
   {
      contactModel.addContactable(contactable);
   }
   
   public void initialize()
   {
      contactModel.setKContact(200.0);
      contactModel.setBContact(10.0);
      contactModel.setFrictionCoefficients(0.5, 0.3);
   }
  
   
   public void doControl()
   {
      contactModel.doContact();
   }

   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   public String getName()
   {
      return registry.getName();
   }

   public String getDescription()
   {
     return getName();
   }

   

   

}
