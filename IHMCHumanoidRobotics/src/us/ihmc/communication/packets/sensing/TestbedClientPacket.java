package us.ihmc.communication.packets.sensing;

import java.util.Random;

import us.ihmc.communication.packets.Packet;

/**
 * @author Peter Abeles
 */
public class TestbedClientPacket extends Packet<TestbedClientPacket>
{
   // 0 = start 1 = stop, 2 = just collect data
   public int request;

   public TestbedClientPacket()
   {
      
   }
   
   public TestbedClientPacket(int request)
   {
      this.request = request;
   }

   public int getRequest()
   {
      return request;
   }

   public void setRequest(int request)
   {
      this.request = request;
   }

   @Override
   public boolean epsilonEquals(TestbedClientPacket other, double epsilon)
   {
      return request == other.request;
   }

   public TestbedClientPacket(Random random)
   {
      this(random.nextInt(2));
   }
}
