package optiTrack;

import java.io.IOException;
import java.net.DatagramPacket;
import java.net.InetAddress;
import java.net.MulticastSocket;
import java.net.NetworkInterface;
import java.util.ArrayList;
import java.util.Enumeration;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.time.CallFrequencyCalculator;

public class MocapDataClient
{
   private static MocapDataClient mocapDataClientSingleton;
   private MulticastSocket socket;
   protected ArrayList<MocapRigidbodiesListener> listOfMocapRigidBodiesListeners = new ArrayList<>();
   public static int NETWORK_IF_TO_USE = -1;

   private CallFrequencyCalculator callFrequencyCalculator;
   protected double frequency;
   protected long lastTime = 0;

   // Do not change these
   private int port = 1511;
   private String mocapIP = "239.255.42.99";

   public MocapDataClient()
   {
      try
      {
         callFrequencyCalculator = new CallFrequencyCalculator(new YoVariableRegistry("Mocap"), "MOCAP_");
         Enumeration<NetworkInterface> enumeration = NetworkInterface.getNetworkInterfaces();

         System.out.println("\n\nNetwork adapters found:\n-----------------------------------");

         while (enumeration.hasMoreElements())
         {
            NetworkInterface networkInterface = enumeration.nextElement();

            System.out.println("Name: " + networkInterface.getDisplayName() + " Index: " + networkInterface.getIndex());
         }

         System.out.println("\n>> Starting Multicast client on interface " + NETWORK_IF_TO_USE + "\n");
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

   public static MocapDataClient getInstance() throws Exception
   {
      if (mocapDataClientSingleton == null)
      {
         mocapDataClientSingleton = new MocapDataClient();
      }

      return mocapDataClientSingleton;
   }

   boolean firstTime = true;
   ArrayList<String> listOfModels = new ArrayList<>();

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

               if (firstTime)
               {
                  firstTime = false;
                  listOfModels = new ArrayList<>();

                  for (MocapRigidBody rb : lisftOfRigidbodies)
                  {
                     listOfModels.add("" + rb.getId());
                  }
               }

               updateListeners(lisftOfRigidbodies);
            }
            catch (IOException e)
            {
//               System.err
//                     .println("**MOCAP WARNING** - Socket Timeout - No Rigibodies are being transmitted from MOCAP SERVER. Make sure streaming is enabled!");
            }
         }
      }
   }

   public ArrayList<String> getAvailableModels()
   {
      return listOfModels;
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
         if (frequency < 95) // Should always be around 99
         {
            System.err.println("**MOCAP WARNING** - Receiving data rate is less than 95Hz >>>> " + frequency);
         }
      }

      ArrayList<MocapRigidbodiesListener> list = (ArrayList<MocapRigidbodiesListener>) listOfMocapRigidBodiesListeners.clone();
      for (MocapRigidbodiesListener listener : list)
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
      MocapDataClient udpMulticastClient = new MocapDataClient();
   }
}
