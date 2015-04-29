package us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.locks.ReentrantReadWriteLock;

import javax.vecmath.Point3d;
import javax.vecmath.Point3f;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.SdfLoader.SDFFullRobotModelFactory;
import us.ihmc.communication.net.NetStateListener;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.sensing.DepthDataClearCommand;
import us.ihmc.communication.packets.sensing.DepthDataClearCommand.DepthDataTree;
import us.ihmc.communication.packets.sensing.DepthDataFilterParameters;
import us.ihmc.communication.packets.sensing.DepthDataStateCommand;
import us.ihmc.communication.packets.sensing.DepthDataStateCommand.LidarState;
import us.ihmc.communication.producers.RobotConfigurationDataBuffer;
import us.ihmc.ihmcPerception.depthData.CollisionBoxProvider;
import us.ihmc.ihmcPerception.depthData.CollisionShapeTester;
import us.ihmc.ihmcPerception.depthData.DepthDataFilter;
import us.ihmc.ihmcPerception.depthData.PointCloudWorldPacketGenerator;
import us.ihmc.ihmcPerception.depthData.RobotDepthDataFilter;
import us.ihmc.utilities.Pair;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.ros.PPSTimestampOffsetProvider;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.wholeBodyController.DRCHandType;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

public class PointCloudDataReceiver extends Thread implements NetStateListener
{
   private final SDFFullRobotModel fullRobotModel;
   private final PPSTimestampOffsetProvider ppsTimestampOffsetProvider;
   private final RobotConfigurationDataBuffer robotConfigurationDataBuffer;
   private final DepthDataFilter depthDataFilter;
   private final CollisionShapeTester collisionBoxNode;

   private final PointCloudWorldPacketGenerator pointCloudWorldPacketGenerator;

   private final ReentrantReadWriteLock readWriteLock = new ReentrantReadWriteLock();
   private final LinkedBlockingQueue<Pair<PointCloudSource[], PointCloudData>> dataQueue = new LinkedBlockingQueue<Pair<PointCloudSource[], PointCloudData>>();
   private final AtomicBoolean sendData = new AtomicBoolean(false);
   private volatile boolean running = true;

   private volatile boolean clearQuadTree = false;
   private volatile boolean clearDecayingPointCloud = false;

   public PointCloudDataReceiver(SDFFullRobotModelFactory modelFactory, CollisionBoxProvider collisionBoxProvider, DRCHandType handType,
         PPSTimestampOffsetProvider ppsTimestampOffsetProvider, DRCRobotJointMap jointMap, RobotConfigurationDataBuffer robotConfigurationDataBuffer,
         PacketCommunicator sensorSuitePacketCommunicator)
   {
      this.fullRobotModel = modelFactory.createFullRobotModel();
      this.ppsTimestampOffsetProvider = ppsTimestampOffsetProvider;
      this.robotConfigurationDataBuffer = robotConfigurationDataBuffer;
      this.depthDataFilter = new RobotDepthDataFilter(handType, fullRobotModel, jointMap.getContactPointParameters().getFootContactPoints());
      this.pointCloudWorldPacketGenerator = new PointCloudWorldPacketGenerator(sensorSuitePacketCommunicator, readWriteLock.readLock(), depthDataFilter);

      if (collisionBoxProvider != null)
      {
         collisionBoxNode = new CollisionShapeTester(this.fullRobotModel, collisionBoxProvider);
      }
      else
      {
         collisionBoxNode = null;
      }
      setupListeners(sensorSuitePacketCommunicator);
      sensorSuitePacketCommunicator.attachStateListener(this);

   }

   public RigidBodyTransform getLidarToSensorTransform(String lidarName)
   {
      return fullRobotModel.getLidarBaseToSensorTransform(lidarName);
   }

   public ReferenceFrame getLidarFrame(String lidarName)
   {
      return fullRobotModel.getLidarBaseFrame(lidarName);
   }

   public InverseDynamicsJoint getLidarJoint(String lidarName)
   {
      return fullRobotModel.getLidarJoint(lidarName);
   }

   @Override
   public void run()
   {
      while (running)
      {
         try
         {
            Pair<PointCloudSource[], PointCloudData> pair = dataQueue.take(); // Do this outside lock to avoid dead-locks
            PointCloudSource[] cloudSources = pair.first();
            PointCloudData data = pair.second();

            readWriteLock.writeLock().lock();
            if (clearDecayingPointCloud)
            {
               if (robotConfigurationDataBuffer.updateFullRobotModelWithNewestData(fullRobotModel, null))
               {
                  depthDataFilter.clearLidarData(DepthDataTree.DECAY_POINT_CLOUD);
                  clearDecayingPointCloud = false;
               }
            }
            if (clearQuadTree)
            {
               if (robotConfigurationDataBuffer.updateFullRobotModelWithNewestData(fullRobotModel, null))
               {
                  depthDataFilter.clearLidarData(DepthDataTree.QUADTREE);
                  clearQuadTree = false;
               }
            }

            if (data != null && sendData.get())
            {
               long prevTimestamp = -1;

               RigidBodyTransform scanFrameToWorld = new RigidBodyTransform();
               for (int i = 0; i < data.points.size(); i++)
               {
                  long nextTimestamp = ppsTimestampOffsetProvider.adjustTimeStampToRobotClock(data.timestamps[i]);
                  if (nextTimestamp != prevTimestamp)
                  {
                     if (robotConfigurationDataBuffer.updateFullRobotModel(true, nextTimestamp, fullRobotModel, null) == -1)
                     {
                        continue;
                     }
                     if(collisionBoxNode != null)
                     {
                    	 collisionBoxNode.update();
                     }
                     prevTimestamp = nextTimestamp;
                  }

                  Point3d pointInWorld;
                  if (!data.scanFrame.isWorldFrame())
                  {
                     data.scanFrame.getTransformToDesiredFrame(scanFrameToWorld, ReferenceFrame.getWorldFrame());

                     pointInWorld = data.points.get(i);
                     scanFrameToWorld.transform(pointInWorld);
                  }
                  else
                  {
                     pointInWorld = data.points.get(i);
                  }

                  Point3d origin = new Point3d();
                  data.lidarFrame.getTransformToWorldFrame().transform(origin);
                  if (collisionBoxNode == null || !collisionBoxNode.contains(pointInWorld))
                  {
                     for (PointCloudSource cloudSource : cloudSources)
                     {
                        switch (cloudSource)
                        {
                        case NEARSCAN:
                           depthDataFilter.addNearScanPoint(pointInWorld, origin);
                           break;
                        case QUADTREE:
                           depthDataFilter.addQuatreePoint(pointInWorld, origin);
                           break;
                        default:
                           System.out.println(getClass().getSimpleName() + " unrecognized cloud source " + cloudSource.name());
                        }
                     }
                  }
               }
            }
            readWriteLock.writeLock().unlock();
         }
         catch (InterruptedException e)
         {
            continue;
         }

      }
   }

   /**
    * Receive new data. Data is stored in a queue, so do not reuse!
    * @param scanFrame
    * @param lidarFrame
    * @param timestamps
    * @param points
    * @param sources
    */
   public void receivedPointCloudData(ReferenceFrame scanFrame, ReferenceFrame lidarFrame, long[] timestamps, ArrayList<Point3d> points,
         PointCloudSource... sources)
   {
      if (timestamps.length != points.size())
      {
         throw new RuntimeException("Number of timestamps does not match number of points");
      }

      try
      {
            dataQueue.put(new Pair<PointCloudSource[], PointCloudData>(sources, new PointCloudData(scanFrame, lidarFrame, timestamps, points)));
      }
      catch (InterruptedException e)
      {
         e.printStackTrace();
      }

   }

   @Override
   public void connected()
   {
      readWriteLock.writeLock().lock();
      {
         setLidarState(LidarState.DISABLE);
      }
      readWriteLock.writeLock().unlock();
   }

   @Override
   public void disconnected()
   {
      sendData.set(false);
   }

   public void setLidarState(LidarState lidarState)
   {
      sendData.set(lidarState != LidarState.DISABLE);
   }

   private void setupListeners(PacketCommunicator sensorSuitePacketCommunicator)
   {

      sensorSuitePacketCommunicator.attachListener(DepthDataStateCommand.class, new PacketConsumer<DepthDataStateCommand>()
      {
         public void receivedPacket(DepthDataStateCommand object)
         {
            setLidarState(object.getLidarState());
         }
      });

      sensorSuitePacketCommunicator.attachListener(DepthDataClearCommand.class, new PacketConsumer<DepthDataClearCommand>()
      {
         public void receivedPacket(DepthDataClearCommand object)
         {
            clearLidar(object);
         }
      });

      sensorSuitePacketCommunicator.attachListener(DepthDataFilterParameters.class, new PacketConsumer<DepthDataFilterParameters>()
      {
         public void receivedPacket(DepthDataFilterParameters object)
         {
            setFilterParameters(object);
         }
      });
   }

   public void clearLidar(DepthDataClearCommand object)
   {
      switch (object.getDepthDataTree())
      {
      case DECAY_POINT_CLOUD:
         clearDecayingPointCloud = true;
         break;
      case QUADTREE:
         clearQuadTree = true;
         break;
      }
   }

   public void setFilterParameters(DepthDataFilterParameters parameters)
   {
      readWriteLock.writeLock().lock();
      {
         depthDataFilter.setParameters(parameters);
      }
      readWriteLock.writeLock().unlock();

   }

   public List<Point3d> getQuadTreePoints()
   {
      readWriteLock.readLock().lock();
      ArrayList<Point3d> groundPoints = new ArrayList<>();
      depthDataFilter.getQuadTree().getStoredPoints(groundPoints);
      readWriteLock.readLock().unlock();
      return groundPoints;
   }

   public Point3f[] getDecayingPointCloudPoints()
   {
      readWriteLock.readLock().lock();
      Point3f[] points = depthDataFilter.getNearScan().getPoints3f();
      readWriteLock.readLock().unlock();
      return points;
   }

   public void start()
   {
      super.start();
      this.pointCloudWorldPacketGenerator.start();
   }

   public void close()
   {
      running = false;
      interrupt();
      this.pointCloudWorldPacketGenerator.stop();
   }

   private static class PointCloudData
   {
      private ReferenceFrame scanFrame;
      private ReferenceFrame lidarFrame;
      private long[] timestamps;
      private ArrayList<Point3d> points;

      public PointCloudData(ReferenceFrame scanFrame, ReferenceFrame lidarFrame, long[] timestamps, ArrayList<Point3d> points)
      {
         this.scanFrame = scanFrame;
         this.lidarFrame = lidarFrame;
         this.timestamps = timestamps;
         this.points = points;
      }

   }
}
