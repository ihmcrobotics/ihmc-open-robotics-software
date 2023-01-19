package us.ihmc.perception.objects;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tuple3D.Point3D;

import static org.junit.Assert.assertTrue;

public class ArUcoObjectInfoTest
{
   @Test
   public void infoArUcoObjectTest()
   {
      ArUcoMarkerObjectInfo arucoInfo = new ArUcoMarkerObjectInfo();
      String name = arucoInfo.getObjectName(0);
      System.out.println("Name: " + name);
      assertTrue(!name.isEmpty());
      double markerSize = arucoInfo.getMarkerSize(0);
      System.out.println("Size: " + markerSize);
      assertTrue(markerSize>0.0);
      Point3D translation = arucoInfo.getObjectTranslation(0);
      System.out.println("Translation: " + translation);
      assertTrue(translation!=null);
   }
}
