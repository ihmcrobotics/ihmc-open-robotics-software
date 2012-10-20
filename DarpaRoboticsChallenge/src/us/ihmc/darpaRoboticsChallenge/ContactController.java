package us.ihmc.darpaRoboticsChallenge;

import java.util.ArrayList;

import com.yobotics.simulationconstructionset.ExternalForcePoint;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.robotController.RobotController;
import com.yobotics.simulationconstructionset.util.ground.Contactable;
import com.yobotics.simulationconstructionset.util.ground.SimpleStickSlipContactModel;

public class ContactController implements RobotController
{
   private static final long serialVersionUID = 1463656383530512645L;

   private final YoVariableRegistry registry = new YoVariableRegistry("ContactController");

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
      contactModel.setKContact(10000.0);
      contactModel.setBContact(1000.0);
      contactModel.setFrictionCoefficients(0.5, 0.3);
   }


   public void doControl()
   {
      contactModel.doContact();
   }

   public YoVariableRegistry getYoVariableRegistry()
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
