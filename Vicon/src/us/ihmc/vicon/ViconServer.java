package us.ihmc.vicon;

import com.yobotics.simulationconstructionset.util.math.filter.AlphaFilteredYoVariable;
import com.yobotics.simulationconstructionset.util.math.filter.FilteredVelocityYoVariable;
import us.ihmc.utilities.math.Differentiator;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.RotationFunctions;
import us.ihmc.utilities.remote.ReflectiveTCPServer;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;
import java.awt.image.ImageFilter;
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
         long lastVelocityUpdate = startTime;
         long runTime = System.currentTimeMillis();

         double alpha = 0.95;
         double velocityUpdateRate = 30;    // ms
         FilteredVelocityYoVariable xDifferentiator = new FilteredVelocityYoVariable("xVelocity", "xVelocity", alpha, velocityUpdateRate, null);
         FilteredVelocityYoVariable yDifferentiator = new FilteredVelocityYoVariable("yVelocity", "yVelocity", alpha, velocityUpdateRate, null);
         FilteredVelocityYoVariable zDifferentiator = new FilteredVelocityYoVariable("zVelocity", "zVelocity", alpha, velocityUpdateRate, null);

         AlphaFilteredYoVariable xVelocityFiltered = new AlphaFilteredYoVariable("xVelocityFilter", null, alpha);
         AlphaFilteredYoVariable yVelocityFiltered = new AlphaFilteredYoVariable("yVelocityFilter", null, alpha);
         AlphaFilteredYoVariable zVelocityFiltered = new AlphaFilteredYoVariable("zVelocityFilter", null, alpha);

         while (!DONE)
         {
            // test various alphas
//          if((System.currentTimeMillis()-runTime) > 20000)
//          {
//             alpha += 0.05;
//             velocityFiltered.setAlpha(alpha);
//             runTime = System.currentTimeMillis();
//          }

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
                                    -viconPose.xAxisRotation, (float)MathTools.trimAngleMinusPiToPi(viconPose.zAxisRotation + (Math.PI/2.0)));

               synchronized (modelPoses)
               {
                  PoseReading lastPoseReading = modelPoses.get(modelName);

                  long currentTime = System.currentTimeMillis();
                  if ((lastPoseReading != null) && (currentTime - lastVelocityUpdate) > velocityUpdateRate)
                  {
                     // compute velocity
                     double lastTimeStamp = lastPoseReading.getTimestamp();
                     velocityUpdateRate = (timestamp - lastTimeStamp) / 10000000.0;
                     xDifferentiator.update(pose.xPosition);
                     yDifferentiator.update(pose.yPosition);
                     zDifferentiator.update(pose.zPosition);
                     double xVelocity = xDifferentiator.getDoubleValue();
                     double yVelocity = yDifferentiator.getDoubleValue();
                     double zVelocity = zDifferentiator.getDoubleValue();
                     xVelocityFiltered.update(xVelocity);
                     yVelocityFiltered.update(yVelocity);
                     zVelocityFiltered.update(zVelocity);

//                   System.out.println(timestamp + ", " + pose + ", " + xVelocityFiltered.getDoubleValue() + ", " + yVelocityFiltered.getDoubleValue() + ", " + zVelocityFiltered.getDoubleValue() + ", " +alpha);
                     lastVelocityUpdate = currentTime;
                  }

                  Vector3d velocity = new Vector3d(xVelocityFiltered.getDoubleValue(), yVelocityFiltered.getDoubleValue(), zVelocityFiltered.getDoubleValue());
                  PoseReading poseReading = new PoseReading(modelName, timestamp, pose, velocity);

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
//         ArrayList<String> modelNames = viconserver.getAvailableModels();
//         while (true)
//         {
//            System.out.println(viconserver.getPose(modelNames.get(0)));
//            Thread.sleep(500);
//         }

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
