package us.ihmc.vicon;

import us.ihmc.utilities.remote.ReflectiveTCPServer;

import java.util.ArrayList;
import java.util.HashMap;

public class ViconServer extends ViconJavaInterface
{
   private boolean DEBUG = false;
   protected ReflectiveTCPServer tcpServer;
   protected ArrayList<String> availableModels = new ArrayList<String>();

   private HashMap<String, PoseListener> listeners = new HashMap<String, PoseListener>();

   public ViconServer(String ip) throws Exception
   {
      if (!ViconConnect(ip))
         throw new Exception("unable to connect to Vicon at " + ip);

      // get list of available bodies
      ViconGetFrame();
      int numBodies = ViconGetNumBodies();
      System.out.println(numBodies + " available models:");

      for (int i = 0; i < numBodies; i++)
      {
         String name = ViconGetBodyName(i);
         availableModels.add(name);
         System.out.println("  " + name);
      }

      // start tcp server
      tcpServer = new ReflectiveTCPServer(this);
      tcpServer.init(7777);

      System.out.println("Vicon server started on " + tcpServer.getHost() + ": " + tcpServer.getPort());
   }

   public ArrayList<String> getAvailableModels()
   {
      return availableModels;
   }

   public Pose getPose(String modelName)
   {
      long startTime = System.currentTimeMillis();
      ViconGetFrame();
      Pose pose = ViconGetBodyAngleAxis(modelName);
      long endTime = System.currentTimeMillis();
      if(DEBUG) System.out.println("getPose took " + (endTime-startTime) + " ms");

      return pose;
   }

   public void registerPoseListener(String host, Integer port, String modelName, Long updatePeriodInMillis)
   {
      PoseListener poseListener = new PoseListener(host, port, modelName, updatePeriodInMillis, this);
      Thread thread = new Thread(poseListener);
      thread.start();

      listeners.put(modelName, poseListener);
   }

   public void stopListener(String modelName)
   {
      PoseListener poseListener = listeners.get(modelName);
      poseListener.stopListening();
   }

   public void stopAllListeners()
   {
      for (PoseListener poseListener : listeners.values())
      {
         poseListener.stopListening();
      }
   }

   public static void main(String[] args)
   {
      String ip = "10.4.1.100";
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
         ViconServer viconserver = new ViconServer(ip);

         ArrayList<String> availableModels = viconserver.getAvailableModels();
         System.out.println("available models:\n" + availableModels);

         for (String modelName : availableModels)
         {
            Pose pose = viconserver.getPose(modelName);
            System.out.println("\t" + modelName + " \tat\t" + pose);
         }

         // test PoseListener
//         viconserver.registerPoseListener(availableModels.get(0), 100);

         // test update rate
//       String modelName = availableModels.get(0);
//       while (true)
//       {
//          long startTime = System.currentTimeMillis();
//          Pose pose = viconserver.getPose(modelName);
//          long endTime = System.currentTimeMillis();
//          System.out.println(modelName + ": " + pose + " in " + (endTime - startTime) + " ms");
//       }

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
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }
}
