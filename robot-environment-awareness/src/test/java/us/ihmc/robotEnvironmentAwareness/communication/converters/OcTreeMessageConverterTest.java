package us.ihmc.robotEnvironmentAwareness.communication.converters;

import perception_msgs.msg.dds.OcTreeKeyListMessage;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.jOctoMap.key.OcTreeKey;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.jOctoMap.pointCloud.PointCloud;
import us.ihmc.jOctoMap.pointCloud.Scan;

import java.util.ArrayList;
import java.util.List;

public class OcTreeMessageConverterTest
{
   @Test
   public void testOcTreeKeyListMessageConverter()
   {
      NormalOcTree ocTree = new NormalOcTree(0.02, 16);

      List<Point3DReadOnly> pointList = new ArrayList<>();
      pointList.add(new Point3D());
      pointList.add(new Point3D(0.2, 0.0, 0.0));
      pointList.add(new Point3D(0.0, 0.2, 0.0));
      pointList.add(new Point3D(0.5, -0.2, -0.4));

      Scan scan = new Scan(new Point3D(-1.0, 0.0, 1.0), new PointCloud(pointList));
      ocTree.insertScan(scan);

      OcTreeKeyListMessage ocTreeDataMessage = OcTreeMessageConverter.createOcTreeDataMessage(ocTree);
      List<OcTreeKey> ocTreeKeys = OcTreeMessageConverter.decompressMessage(ocTreeDataMessage.getKeys(), ocTreeDataMessage.getNumberOfKeys());

      Assertions.assertEquals(pointList.size(), ocTreeKeys.size());
   }
}
