package us.ihmc.ihmcPerception.depthData;

import static org.junit.Assert.*;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.communication.packets.sensing.PointCloudWorldPacket;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;

public class PointCloudWorldPacketGeneratorTest
{
   @Test
   public void testGeneratePointCloudWorldPacket()
   {
      DepthDataFilter depthDataFilter = new DepthDataFilter(ReferenceFrame.getWorldFrame());
      PointCloudWorldPacketGenerator generator = new PointCloudWorldPacketGenerator(depthDataFilter);
      

      RigidBodyTransform sensorOrigin = new RigidBodyTransform(new AxisAngle4d(), new Vector3d(0.0,0.0,1.6));
      for(double x=-10;x<10;x+=0.01)
      {
         for(double y=-10;y<10;y+=0.01)
         {
               Point3d point = new Point3d(x,y,Math.max(x+y,0.0));
               depthDataFilter.addPoint(point, sensorOrigin);
         }
      }
      
      PointCloudWorldPacket packet=generator.getPointCloudWorldPacket();
      
      System.out.println(packet.getGroundQuadTreeSupport().length);
      System.out.println(packet.getGroundQuadTreeSupport().length);
      assertTrue(packet.getGroundQuadTreeSupport().length>0);
      assertTrue(packet.getDecayingWorldScan().length>0);

   }

}
