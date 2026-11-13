package us.ihmc.rdx.simulation.environment.object.objects;

import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObjectFactory;
import us.ihmc.rdx.simulation.environment.object.RDXURDFArticulatedObject;

public class RDXSektionCabinetObject extends RDXURDFArticulatedObject
{
   public static final String NAME = "Sektion Cabinet";
   public static final RDXEnvironmentObjectFactory FACTORY = new RDXEnvironmentObjectFactory(NAME, RDXSektionCabinetObject.class);

   public RDXSektionCabinetObject()
   {
      super(NAME, FACTORY, "environmentObjects/sektion_cabinet/sektion_cabinet.urdf");
   }
}
