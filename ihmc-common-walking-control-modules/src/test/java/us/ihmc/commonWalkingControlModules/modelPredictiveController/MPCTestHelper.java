package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;

public class MPCTestHelper
{
   public static ConvexPolygon2DReadOnly createDefaultContact()
   {
      return createContacts(0.22, 0.12);
   }

   public static ConvexPolygon2DReadOnly createContacts(double footLength, double footWidth)
   {
      ConvexPolygon2D contactPolygon = new ConvexPolygon2D();
      contactPolygon.addVertex(footLength / 2.0, footWidth / 2.0);
      contactPolygon.addVertex(footLength / 2.0, -footWidth / 2.0);
      contactPolygon.addVertex(-footLength / 2.0, footWidth / 2.0);
      contactPolygon.addVertex(-footLength / 2.0, -footWidth / 2.0);
      contactPolygon.update();

      return contactPolygon;
   }
}
