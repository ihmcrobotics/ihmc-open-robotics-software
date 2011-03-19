package us.ihmc.vicon;

import us.ihmc.utilities.remote.RemoteConnection;
import us.ihmc.utilities.remote.RemoteRequest;

import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.Serializable;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Vector;

public class ViconClient
{
   private boolean DEBUG = false;
   protected RemoteConnection viconServer;
   private HashMap<String, Pose> mapModelToPose = new HashMap<String, Pose>();
   private String requestedModel;
   private String myIP = "10.2.36.1";
   private int myPort = 4444;
   private long desiredUpdateRateInMillis = 10;

   public ViconClient(String ip) throws Exception
   {
      viconServer = new RemoteConnection();
      viconServer.connect(ip);

      ArrayList<String> availableModels = getAvailableModels();
      System.out.println("available models = " + availableModels);

      for (String modelName : availableModels)
      {
         mapModelToPose.put(modelName, null);
      }

      ViconPoseListener poseListener = new ViconPoseListener();
      Thread thread = new Thread(poseListener);
      thread.start();

      registerPoseListener(myIP, new Integer(myPort), availableModels.get(0), new Long(desiredUpdateRateInMillis));
      System.out.println(" should be listening for " + availableModels.get(0));
   }

   public ArrayList<String> getAvailableModels()
   {
      RemoteRequest remoteRequest = new RemoteRequest("getAvailableModels", null);
      try
      {
         return (ArrayList<String>) viconServer.SendRequest(remoteRequest);
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }

      return null;
   }

   public Pose getPose(String modelName)
   {
      // This is really slow (3 Hz instead of 80 Hz)
//    Vector<Serializable> parameters = new Vector<Serializable>();
//    parameters.add(modelName);
//    RemoteRequest remoteRequest = new RemoteRequest("getPose", parameters);
//    try
//    {
//       return (Pose) viconServer.SendRequest(remoteRequest);
//    }
//    catch (Exception e)
//    {
//       e.printStackTrace();
//    }
//    return null;

      return mapModelToPose.get(modelName);
   }

   public void registerPoseListener(String host, Integer port, String modelName, Long updateRateInMillis)
   {
      requestedModel = modelName;
      Vector<Serializable> parameters = new Vector<Serializable>();
      parameters.add(host);
      parameters.add(port);
      parameters.add(modelName);
      parameters.add(updateRateInMillis);
      RemoteRequest remoteRequest = new RemoteRequest("registerPoseListener", parameters);
      try
      {
         viconServer.SendObject(remoteRequest);
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }

   public class ViconPoseListener implements Runnable
   {
      protected ServerSocket serverSocket;
      protected Socket client;

      public void run()
      {
         try
         {
            serverSocket = new ServerSocket(4444);
            client = serverSocket.accept();
            ObjectInputStream _ois = new ObjectInputStream(client.getInputStream());

            long startTime = System.currentTimeMillis();
            int updateCount = 0;
            while ((client != null) &&!client.isClosed())
            {
               if (DEBUG)
                  System.out.println("waiting to read...");
               Object obj = _ois.readObject();

               if (obj instanceof Pose)
               {
                  Pose pose = (Pose) obj;
                  mapModelToPose.put(requestedModel, pose);

                  if (DEBUG)
                  {
                     updateCount++;
                     long endTime = System.currentTimeMillis();
                     if ((endTime - startTime) > 1000)
                     {
                        System.out.println("PoseListener updating at " + (int) ((double) updateCount / ((double) (endTime - startTime) / 1000.0)) + " Hz");
                        startTime = endTime;
                        updateCount = 0;
                     }
                  }
               }
               else
               {
                  System.out.println("ViconPoseListenerThread_: received obj which is not a Pose: " + obj.getClass());
               }
            }
         }
         catch (Exception xcp)
         {
            if (DEBUG)
               System.out.println("ViconPoseListenerThread_ done");
            xcp.printStackTrace();
         }

         try
         {
            client.close();
            serverSocket.close();
            if (DEBUG)
               System.out.println(".ViconPoseListenerThread_: ServerAccept Thread has Stopped.");
         }
         catch (IOException xcp)
         {
            xcp.printStackTrace();
         }
      }
   }


   public static void main(String[] args)
   {
      String ip = "10.4.1.100";

//    String ip = "10.2.36.1";

      for (int i = 0; i < args.length - 1; i++)
      {
         if (args[i].equalsIgnoreCase("-ip"))
         {
            ip = args[i + 1];

            break;
         }
      }

      try
      {
         ViconClient client = new ViconClient(ip);
         ArrayList<String> availableModels = client.getAvailableModels();
         long startTime = System.currentTimeMillis();
         int updateCount = 0;
         while (true)
         {
            Pose pose = client.getPose(availableModels.get(0));
            updateCount++;
            long endTime = System.currentTimeMillis();
            if ((endTime - startTime) > 1000)
            {
               double dt = (endTime - startTime) / 1000.0;
               System.out.println("rate = " + (int) (updateCount / dt) + ": " + pose);
               startTime = endTime;
               updateCount = 0;
            }

            Thread.sleep(10);
         }
      }
      catch (Exception ex1)
      {
         ex1.printStackTrace();
      }

      // wait around until terminated
      synchronized (Thread.currentThread())
      {
         try
         {
            Thread.currentThread().wait();
         }
         catch (InterruptedException ex)
         {
            ex.printStackTrace();
         }
      }
   }

   public boolean isConnected()
   {
      return viconServer.isConnected();
   }
}
