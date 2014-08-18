package us.ihmc.darpaRoboticsChallenge.networkProcessor;

import bubo.io.text.WriteCsvObject;
import com.thoughtworks.xstream.XStream;
import georegression.geometry.RotationMatrixGenerator;
import georegression.struct.point.Point3D_F64;
import georegression.struct.se.Se3_F64;
import georegression.struct.so.Quaternion_F64;
import org.ddogleg.struct.FastQueue;
import us.ihmc.darpaRoboticsChallenge.networking.DRCNetworkProcessorNetworkingManager;
import us.ihmc.darpaRoboticsChallenge.sensorProcessing.sensorData.TestbedServerPacket;
import us.ihmc.sensorProcessing.pointClouds.testbed.TestbedAutomaticAlignment;
import us.ihmc.utilities.lidar.polarLidar.LidarScan;

import javax.vecmath.Point3d;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.PrintStream;
import java.util.List;

/**
 * Collects data from network processing and computes location of testbed
 *
 * @author Peter Abeles
 */
public class NetworkProcessorTestbedAlignment implements Runnable
{

   final FastQueue<Point3D_F64> scanStorage = new FastQueue<>(Point3D_F64.class, true);

   volatile boolean active = false;
   volatile boolean processing = false;
   boolean loadedModel = false;

   TestbedAutomaticAlignment testbedFinder;

   DRCNetworkProcessorNetworkingManager networkManager;

   PrintStream out;

   boolean justCollectData;

   long integrationPeriod = 6000;
   long stopTime;

   int totalSaved = 0;

   public NetworkProcessorTestbedAlignment(DRCNetworkProcessorNetworkingManager networkManager)
   {

      this.networkManager = networkManager;

      try
      {
         Se3_F64 estimatedToModel = (Se3_F64) new XStream().fromXML(getClass().getResourceAsStream("/testbed/estimatedToModel.xml"));
         testbedFinder = new TestbedAutomaticAlignment(3,estimatedToModel);
         loadedModel = true;
      }
      catch (RuntimeException e)
      {
         System.out.println("Could not find estimatedToModel.xml");
      }
   }

   public void startCollection( boolean justCollectData )
   {
      if (loadedModel)
      {
         this.justCollectData = justCollectData;
         System.out.println("NetworkProcessorTestbedAlignment - startCollection()");
         synchronized (scanStorage)
         {
            // ignore commands to start collecting data if it's processing the previous cloud still
            if (processing || active)
            {
               System.out.println("  ignoring start request.  Busy");
               return;
            }
            out = null;
            scanStorage.reset();
            active = true;
            stopTime = System.currentTimeMillis() + integrationPeriod;
            networkManager.getControllerStateHandler().sendSerializableObject(new TestbedServerPacket(TestbedServerPacket.START_COLLECTING));
         }
      } else {
         System.out.println("NetworkProcessorTestbedAlignment - model not loaded");
         networkManager.getControllerStateHandler().sendSerializableObject(new TestbedServerPacket(TestbedServerPacket.FAILED_NORESOURCE));
      }
   }

   public void handlePacket(LidarScan polarLidarScan)
   {
      if (loadedModel)
      {
         synchronized (scanStorage)
         {
            if (!active)
               return;

            if (System.currentTimeMillis() < stopTime)
            {
               System.out.println("NetworkProcessorTestbedAlignment - collected packet");
               List<Point3d> points = polarLidarScan.getAllPoints();

               scanStorage.reset();
               for (int i = 0; i < points.size(); i++)
               {
                  Point3d p = points.get(i);
                  scanStorage.grow().set(p.x, p.y, p.z);
               }

               if( justCollectData ) {
                  if( out == null )
                     try {
                        out = new PrintStream(new FileOutputStream(String.format("savedTestbedCloud%02d_scans.csv",totalSaved++)));
                        out.println("# LIDAR scans.  (num) (x y z) .... ");
                     } catch (FileNotFoundException e) {
                        throw new RuntimeException(e);
                     }
                  out.print(scanStorage.size());
                  for (int i = 0; i < scanStorage.size(); i++) {
                     Point3D_F64 p = scanStorage.get(i);
                     out.printf(" %15f %15f %15f",p.x,p.y,p.z);
                  }
                  out.println();
                  out.flush();
               } else {
                  testbedFinder.addScan(scanStorage.toList());
               }
            }
            else
            {
               System.out.println("NetworkProcessorTestbedAlignment - handlePacket done collection");
               if( justCollectData ) {
                  out.close();
                  out = null;
                  processing = false;
                  active = false;
                  networkManager.getControllerStateHandler().sendSerializableObject(new TestbedServerPacket(TestbedServerPacket.DONE_COLLECTING_DATA));
               } else {
                  processing = true;
                  active = false;
               }
            }
         }
      }
   }

   @Override
   public void run()
   {
      if (loadedModel)
      {
         while (true)
         {
            if (processing)
            {
               networkManager.getControllerStateHandler().sendSerializableObject(new TestbedServerPacket(TestbedServerPacket.START_PROCESSING));


               TestbedServerPacket packet = new TestbedServerPacket();
               if ( testbedFinder.process() )
               {
                  packet.setResult(TestbedServerPacket.SUCCESS);

                  Se3_F64 modelToWorld = testbedFinder.getModelToWorld();
                  Quaternion_F64 quat64 = RotationMatrixGenerator.matrixToQuaternion(modelToWorld.R, null);

                  float[] quat = packet.getQuat();
                  quat[0] = (float) quat64.x;
                  quat[1] = (float) quat64.y;
                  quat[2] = (float) quat64.z;
                  quat[3] = (float) quat64.w;

                  float[] translation = packet.getTranslation();
                  translation[0] = (float) modelToWorld.T.x;
                  translation[1] = (float) modelToWorld.T.y;
                  translation[2] = (float) modelToWorld.T.z;
               }
               else
               {
                  packet.setResult(TestbedServerPacket.FAILED);
               }
               networkManager.getControllerStateHandler().sendSerializableObject(packet);
               processing = false;
            }
         }
      }
   }
}
