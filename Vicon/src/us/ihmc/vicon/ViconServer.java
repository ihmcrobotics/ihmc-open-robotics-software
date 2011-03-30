package us.ihmc.vicon;

import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.remote.ReflectiveTCPServer;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;
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
   private final HashMap<String, PoseReading> modelPoses = new HashMap<String, PoseReading>();
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
      Pose pose;
      synchronized (modelPoses)
      {
         PoseReading poseReading = modelPoses.get(modelName);
         if (poseReading == null)
         {
            return null;
         }
         else
         {
            pose = modelPoses.get(modelName).getPose();
         }
      }

      return pose;
   }

   public PoseReading getReading(String modelName)
   {
      PoseReading pose;
      synchronized (modelPoses)
      {
         pose = modelPoses.get(modelName);
      }

      return pose;
   }

   public void registerPoseListener(String host, Integer port)
   {
      System.out.println("client connecting from " + host + ":" + port);
      PoseListener poseListener = new PoseListener(host, port, this);
      Thread thread = new Thread(poseListener);
      thread.start();

      listeners.put(host + ":" + port, poseListener);
   }

   public void stopListener(String hostColonPort)
   {
      PoseListener poseListener = listeners.get(hostColonPort);
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
      private AxisAngle4d axisAngle4d = new AxisAngle4d();
      private Quat4d rotation = new Quat4d();
      private double[] yawPitchRoll = {0, 0, 0};

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
               Pose viconPose = ViconGetBodyEulerAngles(modelName);
               Pose pose = new Pose(viconPose.xPosition / 1000.0f, viconPose.yPosition / 1000.0f, viconPose.zPosition / 1000.0f, viconPose.yAxisRotation,
                                    -viconPose.xAxisRotation, (float) MathTools.trimAngleMinusPiToPi(viconPose.zAxisRotation + (Math.PI / 2.0)));

               synchronized (modelPoses)
               {
                  PoseReading lastPoseReading = modelPoses.get(modelName);
                  PoseReading poseReading = new PoseReading(modelName, timestamp, pose);

                  if (lastPoseReading == null)
                  {
                     modelPoses.put(modelName, poseReading);
                  }
                  else
                  {
                     if (!poseReading.equals(lastPoseReading))
                     {
                        modelPoses.put(modelName, poseReading);
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

//       ArrayList<String> modelNames = viconserver.getAvailableModels();
//       while (true)
//       {
//          for(String modelName: modelNames)
//          {
//             System.out.println(modelName + ": " + viconserver.getPose(modelName));
//          }
//          Thread.sleep(500);
//       }

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
