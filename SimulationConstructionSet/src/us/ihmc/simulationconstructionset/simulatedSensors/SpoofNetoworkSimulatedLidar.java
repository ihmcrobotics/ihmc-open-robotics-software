package us.ihmc.simulationconstructionset.simulatedSensors;

import java.io.IOException;
import java.io.Serializable;
import java.util.ArrayList;
import java.util.Random;

import javax.vecmath.Point3d;

import us.ihmc.communication.remote.DataObjectServer;
import us.ihmc.communication.remote.DataObjectTransponder;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.tools.thread.ThreadTools;

public class SpoofNetoworkSimulatedLidar
{
   static final int SPOOF_CLOUD_SIZE = 16200;
   int port;
   Random gen = new Random(2341238562937410L);
   DataObjectTransponder transponder;

   public SpoofNetoworkSimulatedLidar(int port)
   {
      this.port = port;
   }

   public void start()
   {
      transponder = new DataObjectServer(port);
      transponder.setName("SpoofLidarServer");
      ThreadTools.startAsDaemon(new Runnable()
      {
         @Override
         public void run()
         {
            waitACoupleSeconds();

            while (true)
            {
               LIDARScan spoofScan = generateNewSpoofScan();
               tryToSendSpoofScan(spoofScan);
            }


         }

         private void tryToSendSpoofScan(LIDARScan spoofScan)
         {
            try
            {
               transponder.sendData(LIDARScan.getSerialVersionUID(), spoofScan);
            }
            catch (IOException e)
            {
               e.printStackTrace();
            }
         }

         private void waitACoupleSeconds()
         {
            try
            {
               Thread.sleep(2000);
            }
            catch (InterruptedException e)
            {
               e.printStackTrace();
            }
         }

      }, "Spoofed Lidar Transponder Daemon");
   }

   protected LIDARScan generateNewSpoofScan()
   {
      RandomTools.generateRandomVector(gen, 10);
      ArrayList<Point3d> points = new ArrayList<Point3d>();
      for (int i = 0; i < SPOOF_CLOUD_SIZE; i++)
      {
         points.add(new Point3d(RandomTools.generateRandomVector(gen, 10)));
      }

      return new LIDARScan(points);
   }

   public static class LIDARScan implements Serializable
   {
      private static final long serialVersionUID = 6533143962275276098L;
      private final ArrayList<Point3d> points;
      private long initialTime;

      public LIDARScan(ArrayList<Point3d> points)
      {
         this.points = points;
         initialTime = System.currentTimeMillis();
      }

      public ArrayList<Point3d> getPoints()
      {
         return points;
      }

      public long getTimeDelay()
      {
         return System.currentTimeMillis() - initialTime;
      }
      
      public static long getSerialVersionUID()
      {
         return serialVersionUID;
      }
   }
}
