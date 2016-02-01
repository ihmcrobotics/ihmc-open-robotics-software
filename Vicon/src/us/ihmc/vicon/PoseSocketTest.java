package us.ihmc.vicon;

import java.io.IOException;
import java.io.ObjectOutputStream;
import java.net.Socket;

/**
 * Last updated by: mjohnson
 * On: 3/18/11 10:13 AM
 */
public class PoseSocketTest
{
   public static void main(String[] args)
   {
      try
      {
         Socket socket = new Socket("10.2.36.1", 4444);

         QuaternionPose pose = new QuaternionPose();

         for (int i = 0; i < 10; i++)
         {
            ObjectOutputStream oos = new ObjectOutputStream(socket.getOutputStream());
            oos.writeObject(pose);
            pose.xPosition += 0.1;
         }

         socket.close();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }
}
