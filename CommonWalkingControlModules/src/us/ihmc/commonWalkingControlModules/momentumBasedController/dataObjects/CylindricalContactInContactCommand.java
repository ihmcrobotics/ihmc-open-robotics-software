package us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects;

import us.ihmc.graveYard.commonWalkingControlModules.cylindricalGrasping.bipedSupportPolygons.ContactableCylinderBody;

public class CylindricalContactInContactCommand
{
   private final ContactableCylinderBody contactableCylinderBody;
   private final boolean setInContact;

   public CylindricalContactInContactCommand(ContactableCylinderBody contactableCylinderBody, boolean setInContact)
   {
      this.contactableCylinderBody = contactableCylinderBody;
      this.setInContact = setInContact;
   }

   public String toString()
   {
      return "CylindricalContactInContactCommand: contactableCylinderBody = " + contactableCylinderBody.getName() + " setInContact = " + setInContact;
   }
}
