package us.ihmc.humanoidRobotics.communication.packets.sensing;

import java.util.Random;

import us.ihmc.communication.packets.Packet;

/**
 * Packet sending the results of testbed alignment to the user interface from the server
 *
 * @author Peter Abeles
 */
public class TestbedServerPacket extends Packet<TestbedServerPacket>
{
   public static final int START_COLLECTING = 0;
   public static final int START_PROCESSING = 1;
   public static final int SUCCESS = 2;
   public static final int FAILED = 3;
   public static final int FAILED_NORESOURCE = 4;
   public static final int DONE_COLLECTING_DATA = 5;

   public int result;

   // transform from model to world
   // quaternion
   public float quat[] = new float[4];
   public float translation[] = new float[3];


   public TestbedServerPacket()
   {
   }

   public TestbedServerPacket(int result)
   {
      this.result = result;
   }

   @Override
   public boolean epsilonEquals(TestbedServerPacket other, double epsilon)
   {
      if (result != other.result)
         return false;

      for (int i = 0; i < 4; i++)
      {
         if (Math.abs(quat[i] - other.quat[i]) > epsilon)
            return false;
      }

      for (int i = 0; i < 3; i++)
      {
         if (Math.abs(translation[i] - other.translation[i]) > epsilon)
            return false;
      }

      return true;
   }

   public String resultToString()
   {
      switch (result)
      {
         case START_COLLECTING :
            return "Collecting";

         case START_PROCESSING :
            return "Processing";

         case SUCCESS :
            return "Found Testbed";

         case FAILED :
            return "Failed";

         case FAILED_NORESOURCE :
            return "Failed No Resource";

         case DONE_COLLECTING_DATA :
            return "Done Colleting Data";
      }

      return "Unknown";
   }

   public int getResult()
   {
      return result;
   }

   public void setResult(int result)
   {
      this.result = result;
   }

   public float[] getQuat()
   {
      return quat;
   }

   public void setQuat(float[] quat)
   {
      this.quat = quat;
   }

   public float[] getTranslation()
   {
      return translation;
   }

   public void setTranslation(float[] translation)
   {
      this.translation = translation;
   }

   public TestbedServerPacket(Random random)
   {
      this.setResult(random.nextInt(2));
      float quat[] = this.getQuat();
      for (int i = 0; i < quat.length; i++)
      {
         quat[i] = random.nextFloat();
      }

      float translation[] = this.getTranslation();
      for (int i = 0; i < translation.length; i++)
      {
         translation[i] = random.nextFloat();
      }
   }
}
