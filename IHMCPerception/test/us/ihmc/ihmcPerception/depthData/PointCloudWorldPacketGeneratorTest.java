package us.ihmc.ihmcPerception.depthData;

import static org.junit.Assert.assertTrue;

import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;

import javax.vecmath.Point3d;

import org.junit.Test;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.LocalPacketCommunicator;
import us.ihmc.communication.packetCommunicator.interfaces.PacketCommunicator;
import us.ihmc.communication.packets.sensing.PointCloudWorldPacket;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations.AverageDuration;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

public class PointCloudWorldPacketGeneratorTest
{
   
   @AverageDuration
   @Test(timeout = 10000)
   public void testGeneratePointCloudWorldPacket() throws InterruptedException
   {
      final CountDownLatch latch = new CountDownLatch(3);
      PacketCommunicator sender= new LocalPacketCommunicator(1, "sender");
      LocalPacketCommunicator receiver = new LocalPacketCommunicator(2, "receiver");
      sender.attacthGlobalSendListener(receiver);
      DepthDataFilter depthDataFilter = new DepthDataFilter(ReferenceFrame.getWorldFrame());
      PointCloudWorldPacketGenerator generator = new PointCloudWorldPacketGenerator(depthDataFilter,sender);
      

      Point3d sensorOrigin = new Point3d(0,0,1.0); 
      for(double x=-10;x<10;x+=0.01)
      {
         for(double y=-10;y<10;y+=0.01)
         {
               Point3d point = new Point3d(x,y,Math.max(x+y,0.0));
               depthDataFilter.addPoint(point, sensorOrigin);
         }
      }
      
      PointCloudWorldPacket packet=generator.getPointCloudWorldPacket();
      
      assertTrue(packet.getGroundQuadTreeSupport().length>0);
      assertTrue(packet.getDecayingWorldScan().length>0);
      
      
      
      receiver.attachListener(PointCloudWorldPacket.class, new PacketConsumer<PointCloudWorldPacket>()
      {
         @Override
         public void receivedPacket(PointCloudWorldPacket packet)
         {
            latch.countDown();
            //System.out.println(packet.timestamp);
         }
      });

      
      latch.await(10, TimeUnit.SECONDS);
   }

}
