package optiTrack;

import java.io.IOException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class MarkerSetPacket
{
   public int numberOfMarkers;
   public float posX;
   public float posY;
   public float posZ;

   private static boolean DEBUG = true;

   public static MarkerSetPacket createFromBytes(byte[] bytes) throws IOException
   {
      final MarkerSetPacket data = new MarkerSetPacket();
      final ByteBuffer buf = ByteBuffer.wrap(bytes);

      buf.order(ByteOrder.LITTLE_ENDIAN);

      data.numberOfMarkers = buf.getInt();

      for (int i = 0; i < data.numberOfMarkers; i++)
      {
         data.posX = buf.getFloat();
         data.posY = buf.getFloat();
         data.posZ = buf.getFloat();
      }

      if (DEBUG)
      {
         System.out.println("\n\nMarker Set Info:");
         System.out.println("# of markers in set: " + data.numberOfMarkers);
         System.out.println("X: " + data.posX + " - Y: " + data.posY + " - Z: " + data.posZ);
      }

      return data;
   }
}