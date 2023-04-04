package us.ihmc.perception.objects;

import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tuple3D.Point3D;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertTrue;

public class ArUcoMarkerInfoTest
{
   @Test
   public void markerInfoTest()
   {
      ArUcoMarkerObjectsInfo objectInfo = new ArUcoMarkerObjectsInfo();
      objectInfo.load();
      String name = objectInfo.getObjectName(0);
      System.out.println("Name: " + name);
      assertTrue(!name.isEmpty());
      List<Integer> markerIds = objectInfo.getMarkersId();
      System.out.println("Markers: " + markerIds);
      assertTrue(markerIds.size()>0);
      double markerSize = objectInfo.getMarkerSize(0);
      System.out.println("Size: " + markerSize);
      assertTrue(markerSize>0.0);
      Point3D translation = objectInfo.getMarkerTranslation(0);
      System.out.println("Translation: " + translation);
      assertTrue(translation!=null);
   }
}
