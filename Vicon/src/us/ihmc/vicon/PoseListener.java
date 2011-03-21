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
         while (!DONE)
         {
            Pose pose = viconServer.getPose(modelName);
            PoseReading poseReading = new PoseReading(modelName, System.currentTimeMillis(), pose);
            oos.writeObject(poseReading);
            oos.flush();
            oos.reset();
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
