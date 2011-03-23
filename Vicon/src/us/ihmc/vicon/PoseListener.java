package us.ihmc.vicon;

import java.io.IOException;
import java.io.ObjectOutputStream;
import java.net.Socket;

/**
 * Last updated by: mjohnson
 * On: 3/17/11 4:48 PM
 */
public class PoseListener implements Runnable
{
   private String modelName;
   private long updateRateInMillis;
   private ViconServer viconServer;
   private String host;
   private int port;
   private boolean DONE = false;

   public PoseListener(String host, Integer port, String modelName, Long updateRateInMillis, ViconServer viconServer)
   {
      this.host = host;
      this.port = port;
      this.modelName = modelName;
      this.updateRateInMillis = updateRateInMillis;
      this.viconServer = viconServer;
   }

   public void run()
   {
      try
      {
         Socket socket = new Socket(host, port);
         ObjectOutputStream oos = new ObjectOutputStream(socket.getOutputStream());
         Pose lastPose = null;
         int updateCount = 0;
         long startTime = System.currentTimeMillis();

         while (!DONE)
         {
            Pose pose = viconServer.getPose(modelName);

            if ((lastPose != null) && !pose.equals(lastPose))
            {
               PoseReading poseReading = new PoseReading(modelName, System.nanoTime(), pose);
               oos.writeObject(poseReading);
               oos.flush();
               oos.reset();
               updateCount++;
            }

            long endTime = System.currentTimeMillis();
            if((endTime-startTime) > 1000)
            {
               System.out.println("updating listener for " + modelName + " at " +(int)(updateCount /(((double)(endTime-startTime))/1000.0)) + " Hz");
               updateCount = 0;
               startTime = endTime;
            }

            lastPose = pose;

            Thread.sleep(updateRateInMillis);
         }

         System.out.println("PoseListener: closing socket...");
         socket.close();
      }
      catch (Exception e)
      {
         System.out.println("PoseListener for " + modelName + " stopped (" + host + ":" + port + ")");
      }
   }

   public void stopListening()
   {
      DONE = true;
   }
}
