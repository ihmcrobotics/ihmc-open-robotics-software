package us.ihmc.ihmcPerception.depthData;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.locks.ReentrantReadWriteLock;

import us.ihmc.communication.net.NetStateListener;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.humanoidRobotics.communication.packets.sensing.DepthDataClearCommand;
import us.ihmc.humanoidRobotics.communication.packets.sensing.DepthDataClearCommand.DepthDataTree;
import us.ihmc.humanoidRobotics.communication.packets.sensing.DepthDataFilterParameters;
import us.ihmc.humanoidRobotics.communication.packets.sensing.DepthDataStateCommand;
import us.ihmc.humanoidRobotics.communication.packets.sensing.DepthDataStateCommand.LidarState;
import us.ihmc.humanoidRobotics.communication.packets.sensing.MultisenseMocapExperimentPacket;
import us.ihmc.humanoidRobotics.communication.packets.sensing.MultisenseTest;
import us.ihmc.humanoidRobotics.kryo.PPSTimestampOffsetProvider;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataBuffer;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

public class PointCloudDataReceiver extends Thread implements NetStateListener, PointCloudDataReceiverInterface
{
   private final FullHumanoidRobotModel fullRobotModel;
   private final PPSTimestampOffsetProvider ppsTimestampOffsetProvider;
   private final RobotConfigurationDataBuffer robotConfigurationDataBuffer;
   private final DepthDataFilter depthDataFilter;
   private final CollisionShapeTester collisionBoxNode;

   private final PointCloudWorldPacketGenerator pointCloudWorldPacketGenerator;
   private final SideDependentList<ArrayList<Point2D>> contactPoints;

   private final ReentrantReadWriteLock readWriteLock = new ReentrantReadWriteLock();
   private final LinkedBlockingQueue<PointCloudData> dataQueue = new LinkedBlockingQueue<PointCloudData>();
   private final AtomicBoolean sendData = new AtomicBoolean(false);
   private volatile boolean running = true;

   private volatile boolean clearQuadTree = false;
   private volatile boolean clearDecayingPointCloud = false;
   private final PacketCommunicator sensorSuitePacketCommunicator;
   private boolean DEBUG_WITH_MOCAP = false;

   public PointCloudDataReceiver(FullHumanoidRobotModelFactory modelFactory, CollisionBoxProvider collisionBoxProvider,
         PPSTimestampOffsetProvider ppsTimestampOffsetProvider, DRCRobotJointMap jointMap, RobotConfigurationDataBuffer robotConfigurationDataBuffer,
         PacketCommunicator sensorSuitePacketCommunicator)
   {
      this.fullRobotModel = modelFactory.createFullRobotModel();
      this.ppsTimestampOffsetProvider = ppsTimestampOffsetProvider;
      this.robotConfigurationDataBuffer = robotConfigurationDataBuffer;
      this.depthDataFilter = new RobotDepthDataFilter(fullRobotModel);
      this.contactPoints = jointMap.getContactPointParameters().getFootContactPoints();
      this.pointCloudWorldPacketGenerator = new PointCloudWorldPacketGenerator(sensorSuitePacketCommunicator, readWriteLock.readLock(), depthDataFilter);
      this.sensorSuitePacketCommunicator = sensorSuitePacketCommunicator;
      
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

   private void addPointsUnderFeet()
   {
//      final double QuadTreePointUnderFeetScaling = 1.1;
      for (RobotSide side : RobotSide.values)
      {
         ReferenceFrame soleFrame = fullRobotModel.getSoleFrame(side);
         for (Point2D point : contactPoints.get(side))
         {
            FramePoint footContactPoint = new FramePoint(soleFrame, point.getX(), point.getY(), 0.0);
//            footContactPoint.scale(QuadTreePointUnderFeetScaling);
            footContactPoint.changeFrame(ReferenceFrame.getWorldFrame());
            depthDataFilter.getQuadTree().addPoint(footContactPoint.getX(), footContactPoint.getY(), footContactPoint.getZ());
         }

         FramePoint footCenter = new FramePoint(soleFrame, 0.0, 0.0, 0.0);
         footCenter.changeFrame(ReferenceFrame.getWorldFrame());
         depthDataFilter.getQuadTree().addPoint(footCenter.getX(), footCenter.getY(), footCenter.getZ());

      }

   }

   @Override
   public void run()
   {
      while (running)
      {
         try
         {
            PointCloudData data = dataQueue.take(); // Do this outside lock to avoid dead-locks
            readWriteLock.writeLock().lock();
            if(DEBUG_WITH_MOCAP)
            {
               for (PointCloudSource cloudSource : data.sources)
               {
                  if(cloudSource == PointCloudSource.NEARSCAN)
                  {
                     Point3D[] points = new Point3D[data.points.size()];
                     points = data.points.toArray(points);
                     MultisenseMocapExperimentPacket packet = new MultisenseMocapExperimentPacket();
                     packet.setPointCloud(points, MultisenseTest.NEAR_SCAN_IN_POINT_CLOUD_DATA_RECEIVER);
                     sensorSuitePacketCommunicator.send(packet);
                  }
               }
            }
            
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

            if (!depthDataFilter.getQuadTree().hasPoints())
            {
               if (robotConfigurationDataBuffer.updateFullRobotModelWithNewestData(fullRobotModel, null))
               {
                  addPointsUnderFeet();
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
                     if (collisionBoxNode != null)
                     {
                        collisionBoxNode.update();
                     }
                     prevTimestamp = nextTimestamp;
                  }

                  Point3D pointInWorld;
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

                  Point3D origin = new Point3D();
                  data.lidarFrame.getTransformToWorldFrame().transform(origin);
                  if (collisionBoxNode == null || !collisionBoxNode.contains(pointInWorld) || depthDataFilter.getParameters().boundingBoxScale<=0)                  
                  {
                     for (PointCloudSource cloudSource : data.sources)
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

               if (sendData.get())
                  pointCloudWorldPacketGenerator.setLidarPose(scanFrameToWorld);
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
   @Override
   public void receivedPointCloudData(ReferenceFrame scanFrame, ReferenceFrame lidarFrame, long[] timestamps, ArrayList<Point3D> points,
         PointCloudSource... sources)
   {
      if (timestamps.length != points.size())
      {
         throw new RuntimeException("Number of timestamps does not match number of points");
      }

      dataQueue.offer(new PointCloudData(scanFrame, lidarFrame, timestamps, points, sources));
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
         @Override
         public void receivedPacket(DepthDataStateCommand object)
         {
            setLidarState(object.getLidarState());
         }
      });

      sensorSuitePacketCommunicator.attachListener(DepthDataClearCommand.class, new PacketConsumer<DepthDataClearCommand>()
      {
         @Override
         public void receivedPacket(DepthDataClearCommand object)
         {
            clearLidar(object);
         }
      });

      sensorSuitePacketCommunicator.attachListener(DepthDataFilterParameters.class, new PacketConsumer<DepthDataFilterParameters>()
      {
         @Override
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

   public List<Point3D> getQuadTreePoints()
   {
      readWriteLock.readLock().lock();
      ArrayList<Point3D> groundPoints = new ArrayList<>();
      depthDataFilter.getQuadTree().getStoredPoints(groundPoints);
      readWriteLock.readLock().unlock();
      return groundPoints;
   }

   public Point3D32[] getDecayingPointCloudPoints()
   {
      readWriteLock.readLock().lock();
      Point3D32[] points = depthDataFilter.getNearScan().getPoints3f();
      readWriteLock.readLock().unlock();
      return points;
   }

   @Override
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
      public PointCloudSource[] sources;
      private ReferenceFrame scanFrame;
      private ReferenceFrame lidarFrame;
      private long[] timestamps;
      private ArrayList<Point3D> points;

      public PointCloudData(ReferenceFrame scanFrame, ReferenceFrame lidarFrame, long[] timestamps, ArrayList<Point3D> points, PointCloudSource[] sources)
      {
         this.scanFrame = scanFrame;
         this.lidarFrame = lidarFrame;
         this.timestamps = timestamps;
         this.points = points;
         this.sources = sources;
      }

   }
}
