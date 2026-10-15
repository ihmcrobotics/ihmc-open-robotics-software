package us.ihmc.rdx.simulation.environment.object.objects;

import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObjectFactory;

/**
 * Same person mesh as {@link RDXPersonObject}, wearing the original grey shirt and black pants
 * so they can be told apart from the blue-jeans originals in a crowd.
 */
public class RDXBlackPantsPersonObject extends RDXPersonObject
{
   public static final String NAME = "Person (black pants)";
   public static final RDXEnvironmentObjectFactory FACTORY = new RDXEnvironmentObjectFactory(NAME, RDXBlackPantsPersonObject.class);

   public RDXBlackPantsPersonObject()
   {
      super(NAME, FACTORY);
   }
}
