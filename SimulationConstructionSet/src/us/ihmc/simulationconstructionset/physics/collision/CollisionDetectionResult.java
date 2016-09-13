package us.ihmc.simulationconstructionset.physics.collision;

import java.util.ArrayList;

import us.ihmc.simulationconstructionset.physics.Contacts;

public class CollisionDetectionResult
{
   private final ArrayList<Contacts> results = new ArrayList<>();

   public int getNumberOfCollisions()
   {
      return results.size();
   }

   public void clear()
   {
      results.clear();
   }

   public Contacts getCollision(int i)
   {
      return results.get(i);
   }

   public void addContact(Contacts contacts)
   {
      this.results.add(contacts);
   }
}
