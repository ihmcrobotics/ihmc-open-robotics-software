package us.ihmc.darpaRoboticsChallenge;

import java.util.ArrayList;

import us.ihmc.yoUtilities.YoVariableRegistry;

import com.yobotics.simulationconstructionset.ExternalForcePoint;
import com.yobotics.simulationconstructionset.robotController.RobotController;
import com.yobotics.simulationconstructionset.util.ground.Contactable;
import com.yobotics.simulationconstructionset.util.ground.SimpleStickSlipContactModel;

public class ContactController implements RobotController
{
   private final YoVariableRegistry registry = new YoVariableRegistry("ContactController");

   private final SimpleStickSlipContactModel contactModel;

   public ContactController()
   {
      contactModel = new SimpleStickSlipContactModel("simpleContact", registry);
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

   public void setContactParameters(double kContact, double bContact, double alphaStick, double alphaSlip)
   {
      contactModel.setKContact(kContact);
      contactModel.setBContact(bContact);
      contactModel.setFrictionCoefficients(alphaStick, alphaSlip);
   }

   public void initialize()
   {
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
