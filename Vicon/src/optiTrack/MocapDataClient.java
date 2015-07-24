package optiTrack;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.InetAddress;
import java.net.MulticastSocket;
import java.net.NetworkInterface;
import java.util.ArrayList;
import java.util.Enumeration;

import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.time.CallFrequencyCalculator;

public class MocapDataClient
{
   private MulticastSocket socket;
   private ArrayList<RigidBodyListener> listOfMocapRigidBodyListeners = new ArrayList<>();
   private ArrayList<MocapRigidbodiesListener> listOfMocapRigidBodiesListeners = new ArrayList<>();
   public static int NETWORK_IF_TO_USE = -1;

   private CallFrequencyCalculator callFrequencyCalculator;
   private double frequency;
   private long lastTime = 0;
   
   //Do not change these
   private int port = 1511;
   private String mocapIP = "239.255.42.99";

   public MocapDataClient()
   {
      try
      {
         callFrequencyCalculator = new CallFrequencyCalculator(new YoVariableRegistry("Mocap"), "MOCAP_");
         Enumeration<NetworkInterface> enumeration = NetworkInterface.getNetworkInterfaces();

         while (enumeration.hasMoreElements())
         {
            NetworkInterface networkInterface = enumeration.nextElement();

            System.out.println("Name: " + networkInterface.getDisplayName() + " Index: " + networkInterface.getIndex());
         }

         System.out.println("Starting Multicast using Interface " + NETWORK_IF_TO_USE + "...");
         socket = new MulticastSocket(port);
         socket.setSoTimeout(100);
         if (NETWORK_IF_TO_USE >= 0)
            socket.setNetworkInterface(NetworkInterface.getByIndex(NETWORK_IF_TO_USE));

         InetAddress group = InetAddress.getByName(mocapIP);
         socket.joinGroup(group);

         lastTime = System.currentTimeMillis();

         System.out.println("Starting UDP listener thread");
         Thread udpReceivingThread = new Thread(new UdpClientReceivingThread());
         udpReceivingThread.start();
         System.out.println("UDP listener started on port " + port);

         // socket.leaveGroup(group);
         // socket.close();
      }
      catch (IOException e)
      {
          // TODO Auto-generated catch block
          e.printStackTrace();
      }
   }

   protected class UdpClientReceivingThread implements Runnable
   {
      public void run()
      {
         while (true)
         {
//          ThreadTools.sleep(10);

            try
            {
               DatagramPacket packet;
               byte[] buf = new byte[2000];
               packet = new DatagramPacket(buf, buf.length);
               socket.receive(packet);

               ArrayList<MocapRigidBody> lisftOfRigidbodies = MocapFrameDataPacket.createFromBytes(buf);
               updateListeners(lisftOfRigidbodies);
            }
            catch (IOException e)
            {
               System.err.println("**MOCAP WARNING** - Socket Timeout - No Rigibodies are being transmitted from MOCAP SERVER. Make sure streaming is enabled!");
            }
         }
      }
   }


   public double getMocapDataReceivingFrequency()
   {
      return frequency;
   }

   protected void updateListeners(ArrayList<MocapRigidBody> lisftOfRigidbodies)
   {
      updateRigidBodiesListeners(lisftOfRigidbodies);
   }

   protected void updateRigidBodiesListeners(ArrayList<MocapRigidBody> listOfRigidbodies)
   {
      frequency = callFrequencyCalculator.determineCallFrequency();

      if (System.currentTimeMillis() - lastTime > 5000)
      {
         if (frequency < 95)    // Should always be around 99
         {
            System.err.println("**MOCAP WARNING** - Receiving data rate is less than 95Hz");
         }
      }

      for (RigidBodyListener listener : listOfMocapRigidBodyListeners)
      {
         for (MocapRigidBody rb : listOfRigidbodies)
         {
            if (listener.updateId() == rb.getId())
            {
               listener.updateMocapRigidBody(rb);
            }
         }
      }

      for (MocapRigidbodiesListener listener : listOfMocapRigidBodiesListeners)
      {
         listener.updateRigidbodies(listOfRigidbodies);
      }
   }

   public void registerRigidBodyListener(RigidBodyListener listener)
   {
      listOfMocapRigidBodyListeners.add(listener);
   }

   public void registerRigidBodiesListener(MocapRigidbodiesListener listener)
   {
      listOfMocapRigidBodiesListeners.add(listener);
   }

   public static void main(String args[])
   {
      MocapDataClient udpMulticastClient = new MocapDataClient();
   }
}
