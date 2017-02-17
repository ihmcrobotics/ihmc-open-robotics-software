package us.ihmc.tools.io;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.net.DatagramPacket;
import java.net.DatagramSocket;
import java.net.InetAddress;
import java.net.SocketException;
import java.net.UnknownHostException;

/**
 * DatagramOutputStream
 *
 * Each time you write, it buffers the data. Once you call flush, it sends off a DatagramPacket
 * to the ipAddress and port specified on construction.
 */
public class DatagramOutputStream extends OutputStream
{
   private final ByteArrayOutputStream byteArrayOutputStream;

   private final DatagramSocket datagramSocketToWriteTo;
   private final int port;
   private final InetAddress inetAddress;

   private final boolean VERBOSE = false;
   
   public DatagramOutputStream(int port, String ipAddress) throws SocketException, UnknownHostException
   {
      datagramSocketToWriteTo = new DatagramSocket();

      this.port = port;
      this.inetAddress = InetAddress.getByName(ipAddress);

      byteArrayOutputStream = new ByteArrayOutputStream(DatagramInputStream.BUFFER_SIZE);
   }

   @Override
   public void write(int b) throws IOException
   {
      byteArrayOutputStream.write(b);
   }

   @Override
   public void write(byte b[]) throws IOException
   {
      byteArrayOutputStream.write(b);
   }

   @Override
   public void write(byte[] b, int offset, int length)
   {
      byteArrayOutputStream.write(b, offset, length);
   }

   byte sendIndex = 0;
   @Override
   public void flush()
   {
      byteArrayOutputStream.write(sendIndex); // Add a send index to the end of the datagram before it is sent. Just a single byte that rolls over...
      sendIndex++;
      
      byte[] byteArray = byteArrayOutputStream.toByteArray();

      DatagramPacket datagramPacket = new DatagramPacket(byteArray, byteArray.length, inetAddress, port);

      if (VERBOSE)
      {
         System.out.println("Sending DatagramPacket of length " + datagramPacket.getLength() + " to inetAddress " + datagramPacket.getAddress());
      }
      try
      {
         datagramSocketToWriteTo.send(datagramPacket);
      }
      catch (IOException e)
      {
         System.err.println("IOException in DatagramOutputStream.flush()");
         // UDP is unrealiable, so just drop if an exception...
      }

      byteArrayOutputStream.reset();
   }
   
   @Override
   public void close()
   {
      if (datagramSocketToWriteTo != null)
         datagramSocketToWriteTo.close(); 
   }

}
