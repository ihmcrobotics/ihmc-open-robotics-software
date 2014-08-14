package us.ihmc.darpaRoboticsChallenge.networkProcessor;

import bubo.clouds.FactoryPointCloudShape;
import bubo.clouds.detect.CloudShapeTypes;
import bubo.clouds.detect.PointCloudShapeFinder;
import bubo.clouds.detect.wrapper.ConfigMultiShapeRansac;
import bubo.clouds.detect.wrapper.ConfigSurfaceNormals;
import bubo.io.text.WriteCsvObject;
import com.thoughtworks.xstream.XStream;
import georegression.geometry.RotationMatrixGenerator;
import georegression.struct.point.Point3D_F64;
import georegression.struct.se.Se3_F64;
import georegression.struct.so.Quaternion_F64;
import org.ddogleg.struct.FastQueue;
import us.ihmc.darpaRoboticsChallenge.networking.DRCNetworkProcessorNetworkingManager;
import us.ihmc.darpaRoboticsChallenge.sensorProcessing.sensorData.TestbedServerPacket;
import us.ihmc.sensorProcessing.pointClouds.testbed.DetectTestbedSaveTrasform;
import us.ihmc.utilities.lidar.PointCloudPacket;
import us.ihmc.utilities.lidar.polarLidar.LidarScan;

import javax.vecmath.Point3d;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.util.List;

/**
 * Collects data from network processing and computes location of testbed
 *
 * @author Peter Abeles
 */
public class NetworkProcessorTestbedAlignment implements Runnable
{

   final FastQueue<Point3D_F64> cloud = new FastQueue<>(Point3D_F64.class, true);

   volatile boolean active = false;
   volatile boolean processing = false;
   boolean loadedModel = false;

   PointCloudShapeFinder finder;
   Se3_F64 estimatedToModel;

   DRCNetworkProcessorNetworkingManager networkManager;

   long integrationPeriod = 6000;
   long stopTime;

   public NetworkProcessorTestbedAlignment(DRCNetworkProcessorNetworkingManager networkManager)
   {

      this.networkManager = networkManager;

      ConfigMultiShapeRansac configRansac = ConfigMultiShapeRansac.createDefault(500, 1.2, 0.025, CloudShapeTypes.PLANE);
      configRansac.minimumPoints = 5000;
      finder = FactoryPointCloudShape.ransacSingleAll(new ConfigSurfaceNormals(100, 0.15), configRansac);

      try
      {
         estimatedToModel = (Se3_F64) new XStream().fromXML(getClass().getResourceAsStream("/testbed/estimatedToModel.xml"));
         loadedModel = true;
      }
      catch (RuntimeException e)
      {
         System.out.println("Could not find estimatedToModel.xml");
      }
   }

   public void startCollection()
   {
      if (loadedModel)
      {
         synchronized (cloud)
         {
            // ignore commands to start collecting data if it's processing the previous cloud still
            if (processing || active)
            {
               System.out.println("  ignoring start request.  Busy");
               return;
            }
            cloud.reset();
            active = true;
            stopTime = System.currentTimeMillis() + integrationPeriod;
            networkManager.getControllerStateHandler().sendSerializableObject(new TestbedServerPacket(TestbedServerPacket.START_COLLECTING));
         }
      }
   }

   public void handlePacket(LidarScan polarLidarScan)
   {
      if (loadedModel)
      {
         synchronized (cloud)
         {
            if (!active)
               return;

            if (System.currentTimeMillis() < stopTime)
            {
               List<Point3d> points = polarLidarScan.getAllPoints();

               for (int i = 0; i < points.size(); i++)
               {
                  Point3d p = points.get(i);
                  cloud.grow().set(p.x, p.y, p.z);
               }
            }
            else
            {
               processing = true;
               active = false;
            }
         }
      }
   }

   public void handlePacket(PointCloudPacket packet)
   {
      if (loadedModel)
      {
         synchronized (cloud)
         {
            if (!active)
               return;

            if (System.currentTimeMillis() < stopTime)
            {
               int N = packet.getNumberOfPoints();

               float flatPoints[] = packet.getFlattenedPoints();

               for (int i = 0; i < N; i++)
               {
                  int index = i * 3;
                  cloud.grow().set(flatPoints[index], flatPoints[index + 1], flatPoints[index + 2]);
               }
            }
            else
            {
               processing = true;
               active = false;
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

               try
               {
                  WriteCsvObject write = new WriteCsvObject(new FileOutputStream("mycloud.csv"), Point3D_F64.class, "x", "y", "z");
                  for (int i = 0; i < cloud.size(); i++)
                  {
                     write.writeObject(cloud.get(i));
                  }
                  write.close();
               }
               catch (FileNotFoundException e)
               {

               }
               catch (IOException e)
               {

               }

               // find the planes
               finder.process(cloud.toList(), null);

               // find the testbed
               Se3_F64 estimatedToWorld = DetectTestbedSaveTrasform.findTestbed(finder.getFound());

               TestbedServerPacket packet = new TestbedServerPacket();
               if (estimatedToWorld != null)
               {
                  packet.setResult(TestbedServerPacket.SUCCESS);
                  Se3_F64 modelToWorld = estimatedToModel.invert(null).concat(estimatedToWorld, null);

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
