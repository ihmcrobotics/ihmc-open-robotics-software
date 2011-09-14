package us.ihmc.vicon;

import java.io.ObjectOutputStream;
import java.net.Socket;
import java.util.ArrayList;
import java.util.HashMap;

/**
 * Last updated by: jsmith
 * On: 13/08/2011
 */
public class ViconServerListener implements Runnable
{
   private ViconServer viconServer;
   private String host;
   private int port;
   private boolean DONE = false;
   private HashMap<String, ViconModelReading> lastPoseReadings = new HashMap<String, ViconModelReading>();

   public ViconServerListener(String host, Integer port, ViconServer viconServer)
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
               ViconModelReading poseReading = viconServer.getReading(modelName);
               ViconModelReading lastPoseReading = lastPoseReadings.get(modelName);
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
