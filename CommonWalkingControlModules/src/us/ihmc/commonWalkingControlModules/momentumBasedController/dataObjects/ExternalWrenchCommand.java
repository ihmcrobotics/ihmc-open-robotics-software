package us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects;

import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.Wrench;

public class ExternalWrenchCommand
{
   private final RigidBody rigidBody;
   private final Wrench wrench;

   public ExternalWrenchCommand(RigidBody rigidBody, Wrench wrench)
   {
      this.rigidBody = rigidBody;
      this.wrench = wrench;
   }

   
   
   public RigidBody getRigidBody()
   {
      return rigidBody;
   }



   public Wrench getWrench()
   {
      return wrench;
   }



   public String toString()
   {
      return "ExternalWrenchCommand: " + wrench;
   }

}
