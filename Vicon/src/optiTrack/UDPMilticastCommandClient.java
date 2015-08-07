package optiTrack;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.InetAddress;
import java.net.MulticastSocket;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;

import us.ihmc.tools.thread.ThreadTools;

public class UDPMilticastCommandClient
{
   MulticastSocket socket;
   ArrayList<MocapRigidbodiesListener> listOfMocapRigidBodiesListeners = new ArrayList<>();
   
   final int NAT_PING = 4;

   public UDPMilticastCommandClient()
   {
      try
      {
         System.out.println("Starting Multicast..");
         socket = new MulticastSocket(1510);
         InetAddress group = InetAddress.getByName("239.255.42.99");
         socket.joinGroup(group);

         for(int i = 0; i < 3; i++)
         {
            
            ByteBuffer bbuf = ByteBuffer.allocate(4); 
            bbuf.order(ByteOrder.LITTLE_ENDIAN);

            bbuf.putInt(0);
            
            DatagramPacket packet;
            packet = new DatagramPacket(bbuf.array(), 4, group, 1510);
            socket.send(packet);
            ThreadTools.sleep(100);
            System.out.println("Sending...");
         }
         
         System.out.println("Starting UDP listener thread");
         Thread udpReceivingThread = new Thread(new UdpClientReceivingThread());
         udpReceivingThread.start();
         System.out.println("UDP listener started");

         //         socket.leaveGroup(group);
         //         socket.close();
      }
      catch (IOException e)
      {
         // TODO Auto-generated catch block
         e.printStackTrace();
      }
   }
   
   private class UdpClientReceivingThread implements Runnable
   {
      public void run()
      {
         while (true)
         {
            //          ThreadTools.sleep(100); 
            try
            {
               DatagramPacket packet;
               byte[] buf = new byte[2000];
               packet = new DatagramPacket(buf, buf.length);
               socket.receive(packet);

               ArrayList<MocapRigidBody> lisftOfRigidbodies = MocapFrameDataPacket.createFromBytes(buf);
               updateRigidBodiesListeners(lisftOfRigidbodies);
            }
            catch (IOException e)
            {
               // TODO Auto-generated catch block
               e.printStackTrace();
            }
         }
      }
   }

   private void updateRigidBodiesListeners(ArrayList<MocapRigidBody> listOfRigidbodies)
   {
      for (MocapRigidbodiesListener listener : listOfMocapRigidBodiesListeners)
      {
         listener.updateRigidbodies(listOfRigidbodies);
      }
   }

   public void registerRigidBodiesListener(MocapRigidbodiesListener listener)
   {
      listOfMocapRigidBodiesListeners.add(listener);
   }

   public static void main(String args[])
   {
      UDPMilticastCommandClient udpMulticastClient = new UDPMilticastCommandClient();
   }
}
