package us.ihmc.vicon;

import java.io.ObjectOutputStream;
import java.net.Socket;
import java.util.ArrayList;
import java.util.HashMap;

/**
 * Last updated by: mjohnson
 * On: 3/17/11 4:48 PM
 */
public class PoseListener implements Runnable
{
   private ViconServer viconServer;
   private String host;
   private int port;
   private boolean DONE = false;
   private HashMap<String, PoseReading> lastPoseReadings = new HashMap<String, PoseReading>();

   public PoseListener(String host, Integer port, ViconServer viconServer)
   {
      this.host = host;
      this.port = port;
      this.viconServer = viconServer;
   }

   public void run()
   {
      try
      {
         Socket socket = new Socket(host, port);
         ObjectOutputStream oos = new ObjectOutputStream(socket.getOutputStream());

         while (!DONE)
         {
            ArrayList<String> modelNames = viconServer.getAvailableModels();
            for (String modelName : modelNames)
            {
               PoseReading poseReading = viconServer.getReading(modelName);
               PoseReading lastPoseReading = lastPoseReadings.get(modelName);
               if (lastPoseReading == null)
               {
                  lastPoseReadings.put(modelName, poseReading);
               }
               else
               {
                  if (!poseReading.equals(lastPoseReading))
                  {
                     oos.writeObject(poseReading);
                     oos.flush();
                     oos.reset();
                     lastPoseReadings.put(modelName, poseReading);
                  }
               }
            }

            Thread.sleep(10);
         }

         System.out.println("PoseListener: closing socket...");
         socket.close();
      }
      catch (Exception e)
      {
         System.out.println("PoseListener stopped (" + host + ":" + port + ")");
      }
   }

   public void stopListening()
   {
      DONE = true;
   }
}
