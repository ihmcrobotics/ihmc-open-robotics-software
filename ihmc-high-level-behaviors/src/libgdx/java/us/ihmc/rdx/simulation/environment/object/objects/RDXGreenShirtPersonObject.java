package us.ihmc.rdx.simulation.environment.object.objects;

import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObjectFactory;

/**
 * Same person mesh as {@link RDXPersonObject}, wearing the green-shirt texture so they can be
 * told apart from the grey-shirt original and the orange-shirt lead in a crowd.
 */
public class RDXGreenShirtPersonObject extends RDXPersonObject
{
   public static final String NAME = "Person (green shirt)";
   public static final RDXEnvironmentObjectFactory FACTORY = new RDXEnvironmentObjectFactory(NAME, RDXGreenShirtPersonObject.class);

   public RDXGreenShirtPersonObject()
   {
      super(NAME, FACTORY);
   }
}
