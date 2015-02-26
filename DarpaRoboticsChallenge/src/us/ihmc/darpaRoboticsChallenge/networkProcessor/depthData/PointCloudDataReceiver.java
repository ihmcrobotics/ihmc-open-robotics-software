package us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.locks.ReentrantReadWriteLock;

import javax.vecmath.Point3d;
import javax.vecmath.Point3f;

import us.ihmc.communication.net.NetStateListener;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.interfaces.PacketCommunicator;
import us.ihmc.communication.packets.sensing.DepthDataClearCommand;
import us.ihmc.communication.packets.sensing.DepthDataClearCommand.DepthDataTree;
import us.ihmc.communication.packets.sensing.DepthDataFilterParameters;
import us.ihmc.communication.packets.sensing.DepthDataStateCommand;
import us.ihmc.communication.packets.sensing.DepthDataStateCommand.LidarState;
import us.ihmc.communication.producers.RobotPoseBuffer;
import us.ihmc.ihmcPerception.depthData.DepthDataFilter;
import us.ihmc.ihmcPerception.depthData.PointCloudWorldPacketGenerator;
import us.ihmc.ihmcPerception.depthData.RobotBoundingBoxes;
import us.ihmc.ihmcPerception.depthData.RobotDepthDataFilter;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.wholeBodyController.DRCRobotJointMap;

public class PointCloudDataReceiver extends Thread implements NetStateListener
{
   private final DepthDataFilter depthDataFilter;
   private final PointCloudWorldPacketGenerator pointCloudWorldPacketGenerator;
   private final RobotPoseBuffer robotPoseBuffer;

   private final ReentrantReadWriteLock readWriteLock = new ReentrantReadWriteLock();
   private final LinkedBlockingQueue<PointCloudData> dataQueue = new LinkedBlockingQueue<>();

   private final ArrayList<RobotBoundingBoxes> pointCloudFilters = new ArrayList<>();
   
   private final AtomicBoolean sendData = new AtomicBoolean(false);

   private volatile boolean running = true;

   public PointCloudDataReceiver(FullRobotModel fullRobotModel, DRCRobotJointMap jointMap,
         RobotPoseBuffer robotPoseBuffer, PacketCommunicator packetCommunicator)
   {
      this.depthDataFilter = new RobotDepthDataFilter(fullRobotModel, jointMap.getContactPointParameters().getFootContactPoints());
      this.robotPoseBuffer = robotPoseBuffer;
      this.pointCloudWorldPacketGenerator = new PointCloudWorldPacketGenerator(packetCommunicator, readWriteLock.readLock(), depthDataFilter);

      setupListeners(packetCommunicator);
      packetCommunicator.attachStateListener(this);

   }

   @Override
   public void run()
   {
      while (running)
      {
         try
         {
            PointCloudData data = dataQueue.take();
            if (data != null && sendData.get())
            {
               readWriteLock.writeLock().lock();
               for (int i = 0; i < data.points.size(); i++)
               {
                  Point3d pointInWorld = robotPoseBuffer.transformToWorld(data.scanFrame, data.timestamps[i], data.points.get(i));
                  
                  boolean include = true;
                  Point3d origin = data.originInWorld;
                  if(origin == null)
                  {
                     origin = new Point3d();
                     robotPoseBuffer.floorEntry(data.timestamps[0]).getLidarPose(0).transform(origin);
                  }
                  for(int f = 0; f < pointCloudFilters.size(); f++)
                  {
                     include &= pointCloudFilters.get(f).isValidPoint(origin, pointInWorld);
                  }
                  if(include)
                  {
                     depthDataFilter.addPoint(pointInWorld, origin);
                  }
               }
               readWriteLock.writeLock().unlock();
            }
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
    * @param originInWorld
    * @param timestamps
    * @param points
    */
   public void receivedPointCloudData(ReferenceFrame scanFrame, Point3d originInWorld, long[] timestamps, ArrayList<Point3d> points)
   {
      if (timestamps.length != points.size())
      {
         throw new RuntimeException("Number of timestamps does not match number of points");
      }

      try
      {
         dataQueue.put(new PointCloudData(scanFrame, originInWorld, timestamps, points));
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
         depthDataFilter.clearLidarData(DepthDataTree.QUADTREE);
         depthDataFilter.clearLidarData(DepthDataTree.DECAY_POINT_CLOUD);
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

   private void setupListeners(PacketCommunicator packetCommunicator)
   {

      packetCommunicator.attachListener(DepthDataStateCommand.class, new PacketConsumer<DepthDataStateCommand>()
      {
         public void receivedPacket(DepthDataStateCommand object)
         {
            setLidarState(object.getLidarState());
         }
      });

      packetCommunicator.attachListener(DepthDataClearCommand.class, new PacketConsumer<DepthDataClearCommand>()
      {
         public void receivedPacket(DepthDataClearCommand object)
         {
            clearLidar(object);
         }
      });

      packetCommunicator.attachListener(DepthDataFilterParameters.class, new PacketConsumer<DepthDataFilterParameters>()
      {
         public void receivedPacket(DepthDataFilterParameters object)
         {
            setFilterParameters(object);
         }
      });
   }

   public void clearLidar(DepthDataClearCommand object)
   {
      readWriteLock.writeLock().lock();
      {
         depthDataFilter.clearLidarData(object.getDepthDataTree());
      }
      readWriteLock.writeLock().unlock();

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
      private Point3d originInWorld;
      private long[] timestamps;
      private ArrayList<Point3d> points;

      public PointCloudData(ReferenceFrame scanFrame, Point3d originInWorld, long[] timestamps, ArrayList<Point3d> points)
      {
         this.scanFrame = scanFrame;
         this.originInWorld = originInWorld;
         this.timestamps = timestamps;
         this.points = points;
      }

   }

   public void addPointFilter(RobotBoundingBoxes robotBoundingBoxes)
   {
      pointCloudFilters.add(robotBoundingBoxes);
   }
}
