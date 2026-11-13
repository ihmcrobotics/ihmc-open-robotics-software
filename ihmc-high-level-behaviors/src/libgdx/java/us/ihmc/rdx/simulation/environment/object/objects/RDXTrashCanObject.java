package us.ihmc.rdx.simulation.environment.object.objects;

import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObjectFactory;
import us.ihmc.rdx.simulation.environment.object.RDXURDFArticulatedObject;

public class RDXTrashCanObject extends RDXURDFArticulatedObject
{
   public static final String NAME = "TrashCan";
   public static final RDXEnvironmentObjectFactory FACTORY = new RDXEnvironmentObjectFactory(NAME, RDXTrashCanObject.class);

   public RDXTrashCanObject()
   {
      super(NAME, FACTORY, "environmentObjects/trash_can/trash_can.urdf");
   }
}
