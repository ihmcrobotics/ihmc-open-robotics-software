package us.ihmc.tools.io;

import java.io.IOException;
import java.io.InputStream;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.SocketException;

/**
 * DatagramInputStream.
 * Each time you read, it'll give you data from the last packet if there is some, or wait to receive a new packet
 * to replenish the buffer. If the buffer has data, it'll just ignore the incoming DataGrams.
 * May receive data out of order, but probably not a problem on a direct connection.
 *
 * Shouldn't concatenate this with a BufferedInputStream, like you would with TCP, if you want to
 * ignore packets until you are ready for the next one.
 */
public class DatagramInputStream extends InputStream
{
   private static final boolean VERBOSE = false;
   public static final int BUFFER_SIZE = 32000;
   private final byte[] bytebuffer = new byte[BUFFER_SIZE];


   private final ResettableByteArrayInputStream resettableByteArrayInputStream;
   private DatagramSocket datagramSocketToListenOn;
   private final int port;
   private boolean throwOutStalePackets = false;


   public DatagramInputStream(int port) throws SocketException
   {
      this.port = port;

      resettableByteArrayInputStream = new ResettableByteArrayInputStream();

      datagramSocketToListenOn = new DatagramSocket(port);

      datagramSocketToListenOn.setReceiveBufferSize(BUFFER_SIZE);

      int actualBufferSize = datagramSocketToListenOn.getReceiveBufferSize();
      if (VERBOSE)
         System.out.println("DatagramInputStream actualBufferSize = " + actualBufferSize);
   }
   
   public void setThrowOutStalePackets(boolean throwOutStalePackets)
   {
      this.throwOutStalePackets = throwOutStalePackets;
   }

   /**
    * This will block once out of data and need to wait for a new DatagramPacket.
    */
   @Override
   public int read() throws IOException
   {
      refreshByteInputStreamIfNecessaryByReadingNewDatagram();

      return resettableByteArrayInputStream.read();
   }

   /**
    * This will block once out of data and need to wait for a new DatagramPacket.
    */
   @Override
   public int read(byte b[]) throws IOException
   {
      refreshByteInputStreamIfNecessaryByReadingNewDatagram();

      return resettableByteArrayInputStream.read(b);
   }

   /**
    * This will block once out of data and need to wait for a new DatagramPacket.
    */
   @Override
   public int read(byte b[], int offset, int length) throws IOException
   {
      refreshByteInputStreamIfNecessaryByReadingNewDatagram();

      return resettableByteArrayInputStream.read(b, offset, length);
   }

   private byte expectedIndexByte = 0;
   private void refreshByteInputStreamIfNecessaryByReadingNewDatagram() throws IOException
   {
      while (resettableByteArrayInputStream.available() <= 0)
      {
         if (throwOutStalePackets)
         {
            throwOutStalePackets();
         }

         if (VERBOSE)
         {
            System.out.println("DatagramInputStream Reading new DatagramPacket");
         }

//         byte[] bytebuffer = new byte[BUFFER_SIZE];
         DatagramPacket datagramPacket = new DatagramPacket(bytebuffer, bytebuffer.length);

         datagramSocketToListenOn.receive(datagramPacket);
         byte[] dataBytes = datagramPacket.getData();
         int length = datagramPacket.getLength();

         byte indexByte = dataBytes[length-1];
         if (indexByte != expectedIndexByte)
         {
            System.out.println("DatagramInputStream: indexByte != expectedIndexByte! indexByte = " + indexByte + ", expectedIndexByte = " + expectedIndexByte);
            expectedIndexByte = indexByte;
         }
         expectedIndexByte++;
         
         if (VERBOSE)
         {
            System.out.println("DatagramInputStream Read a DatagramPacket with length = " + length + "dataBytes.length = " + dataBytes.length);
         }
         
         resettableByteArrayInputStream.setBuffer(dataBytes, length - 1);
      }
   }

   private void throwOutStalePackets() throws SocketException
   {
      if (datagramSocketToListenOn != null)
      {
         datagramSocketToListenOn.close();
      }

      datagramSocketToListenOn = new DatagramSocket(port);

   }
   
   @Override
   public void close()
   {
      if (datagramSocketToListenOn != null)
      {
         datagramSocketToListenOn.close();
      } 
   }

}
