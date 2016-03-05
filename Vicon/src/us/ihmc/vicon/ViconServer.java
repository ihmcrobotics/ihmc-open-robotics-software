package us.ihmc.vicon;

import java.util.ArrayList;
import java.util.HashMap;

import us.ihmc.communication.remote.ReflectiveTCPServer;

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
   private final HashMap<String, ViconModelReading> modelReadings = new HashMap<String, ViconModelReading>();
   private HashMap<String, ViconServerListener> listeners = new HashMap<String, ViconServerListener>();

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
   
   public QuaternionPose getQuaternionPose(String modelName)
   {
      QuaternionPose pose;
      synchronized (modelReadings)
      {
         ViconModelReading poseReading = modelReadings.get(modelName);
         if (poseReading == null)
         {
            return null;
         }
         else
         {
            pose = modelReadings.get(modelName).getQuaternionPose();
         }
      }

      return pose;
   }

   public ViconModelReading getReading(String modelName)
   {
      ViconModelReading pose;
      synchronized (modelReadings)
      {
         pose = modelReadings.get(modelName);
      }

      return pose;
   }

   public void registerPoseListener(String host, Integer port)
   {
      System.out.println("client connecting from " + host + ":" + port);
      ViconServerListener poseListener = new ViconServerListener(host, port, this);
      Thread thread = new Thread(poseListener);
      thread.start();

      listeners.put(host + ":" + port, poseListener);
   }

   public void stopListener(String hostColonPort)
   {
      ViconServerListener poseListener = listeners.get(hostColonPort);
      poseListener.stopListening();
   }

   public void stopAllListeners()
   {
      for (ViconServerListener poseListener : listeners.values())
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
         long startTime = System.currentTimeMillis();
         int updateCount = 0;

         while (!DONE)
         {
            // update frame
            ViconGetFrame();

            // get timestamp
            long timestamp = System.nanoTime();

            // update each model
            boolean updated = false;
            for (String modelName : availableModels)
            {
               QuaternionPose viconPose = ViconGetBodyQuaternion(modelName);
               
               QuaternionPose pose = new QuaternionPose(viconPose);
               pose.scaleTranslation(0.001f);

               synchronized (modelReadings)
               {
                  ViconModelReading lastViconReading = modelReadings.get(modelName);
                  ViconModelReading viconReading = new ViconModelReading(modelName, timestamp, pose);
                  

                  if (lastViconReading == null)
                  {
                     modelReadings.put(modelName, viconReading);
                  }
                  else
                  {
                     if (pose.equals(lastViconReading.getQuaternionPose()))
                     {
                        pose.invalidate();  
                     }
                     
                     if(!viconReading.equals(lastViconReading))
                     {
                        modelReadings.put(modelName, viconReading);
                        updated = true;
                     }
                  }
               }
            }

            if (updated)
               updateCount++;

            // display update rate
            long endTime = System.currentTimeMillis();
            if ((endTime - startTime) > 3000)
            {
               System.out.println("Vicon server updating at " + (int) ((double) updateCount / ((double) (endTime - startTime) / 1000.0)) + " Hz");
               startTime = endTime;
               updateCount = 0;
            }

            // don't bother going faster than Vicon
            try
            {
               Thread.sleep(10);
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
      String ip = "192.168.0.3";
      
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
         new ViconServer(ip);


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
