package us.ihmc.rdx.simulation.environment.object.objects;

import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObjectFactory;

/**
 * Same person mesh as {@link RDXPersonObject}, wearing the orange-shirt texture so they can be
 * told apart from the grey-shirt original in a crowd.
 */
public class RDXOrangeShirtPersonObject extends RDXPersonObject
{
   public static final String NAME = "Person (orange shirt)";
   public static final RDXEnvironmentObjectFactory FACTORY = new RDXEnvironmentObjectFactory(NAME, RDXOrangeShirtPersonObject.class);

   public RDXOrangeShirtPersonObject()
   {
      super(NAME, FACTORY);
   }
}
