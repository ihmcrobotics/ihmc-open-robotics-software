package us.ihmc.vicon;

import us.ihmc.utilities.remote.ReflectiveTCPServer;

import java.util.ArrayList;
import java.util.HashMap;

/**
 * ViconServer can be run anywhere the Vicon dll is located. The Vicon dll is only available for Windows 32-bit machines.
 * Normally the Vicon Server is run on the Vicon host machine with the Vicon IQ software. The ViconServer has an
 * update rate of approximately 100Hz (this has only been tested for a single model).
 *
 * The basic architecture is to run a local thread at 100Hz and save all model poses. When remote clients request pose
 * information they get the most recently stored information. This ensures the maximum update rate and should allow
 * multiple clients to access the data quickly.
 */
public class ViconServer extends ViconJavaInterface
{
   protected ReflectiveTCPServer tcpServer;
   protected ArrayList<String> availableModels = new ArrayList<String>();
   private HashMap<String, Pose> modelPoses = new HashMap<String, Pose>();
   private HashMap<String, PoseListener> listeners = new HashMap<String, PoseListener>();

   private ViconReader viconReader;

   public ViconServer(String ip) throws Exception
   {
      if (!ViconConnect(ip))
         throw new Exception("unable to connect to Vicon at " + ip);

      // start a thread reading Vicon
      viconReader = new ViconReader();
      Thread thread = new Thread(viconReader);
      thread.start();

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
      return modelPoses.get(modelName);
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

   class ViconReader implements Runnable
   {
      private boolean DONE = false;

      public void stopViconReader()
      {
         this.DONE = true;
      }

      public void run()
      {
         // get list of available models
         ViconGetFrame();
         int numBodies = ViconGetNumBodies();
         System.out.println("Vicon has " + numBodies + " available models:");

         for (int i = 0; i < numBodies; i++)
         {
            String name = ViconGetBodyName(i);
            availableModels.add(name);
            System.out.println("\t" + name);
         }

         // reader loop
         boolean displayedUpdateRate = false;
         long startTime = System.currentTimeMillis();
         int updateCount = 0;
         while (!DONE)
         {
            // update frame
            ViconGetFrame();

            // update each model
            for (String modelName : availableModels)
            {
               Pose pose = ViconGetBodyAngleAxis(modelName);

               if (modelPoses.get(modelName) == null)
               {
                  modelPoses.put(modelName, pose);
               }
               else
               {
                  if ((pose.xPosition != modelPoses.get(modelName).xPosition) || (pose.yPosition != modelPoses.get(modelName).yPosition))
                  {
                     modelPoses.put(modelName, pose);
                     updateCount++;
                  }
               }
            }

            // display update rate
            if (!displayedUpdateRate && (System.currentTimeMillis() - startTime) > 3000)
            {
               System.out.println("Vicon server updating at " + (int)((double) updateCount / ((double) (System.currentTimeMillis() - startTime) / 1000.0)) + " Hz");
               displayedUpdateRate = true;
            }

            // don't bother going faster than Vicon
            try
            {
               Thread.sleep(5);
            }
            catch (InterruptedException e)
            {
               e.printStackTrace();
            }
         }
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

         // wait around until terminated
         synchronized (Thread.currentThread())
         {
            Thread.currentThread().wait();
         }
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }
}
