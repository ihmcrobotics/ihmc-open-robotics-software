package us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData;

import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.atomic.AtomicBoolean;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.communication.AbstractNetworkProcessorNetworkingManager;
import us.ihmc.communication.NetworkProcessorControllerCommandHandler;
import us.ihmc.communication.NetworkProcessorControllerStateHandler;
import us.ihmc.communication.net.NetStateListener;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.sensing.DepthDataClearCommand;
import us.ihmc.communication.packets.sensing.DepthDataClearCommand.DepthDataTree;
import us.ihmc.communication.packets.sensing.DepthDataFilterParameters;
import us.ihmc.communication.packets.sensing.DepthDataStateCommand;
import us.ihmc.communication.packets.sensing.DepthDataStateCommand.LidarState;
import us.ihmc.communication.packets.sensing.FilteredPointCloudPacket;
import us.ihmc.communication.packets.sensing.PointCloudPacket;
import us.ihmc.communication.packets.sensing.RobotPoseData;
import us.ihmc.communication.producers.RobotPoseBuffer;
import us.ihmc.communication.producers.RobotPoseBufferListener;
import us.ihmc.ihmcPerception.depthData.DepthDataFilter;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;

public class PointCloudDataReceiver implements RobotPoseBufferListener, NetStateListener
{

   protected ConcurrentLinkedQueue<PointCloudPacket> pendingScans = new ConcurrentLinkedQueue<PointCloudPacket>();
   protected volatile AtomicBoolean sendData = new AtomicBoolean(false);
   private final Object commandLock = new Object();
   private final DepthDataFilter lidarDataFilter;
   private final NetworkProcessorControllerStateHandler controllerStateHandler;
   protected LidarStateCommandListener lidarStateCommandListener;
   private final RobotPoseBuffer robotPoseBuffer;

   public PointCloudDataReceiver(RobotPoseBuffer robotPoseBuffer, AbstractNetworkProcessorNetworkingManager networkingManager, SDFFullRobotModel fullRobotModel,
         DepthDataFilter lidarDataFilter)
   {
      robotPoseBuffer.addListener(this);

      this.robotPoseBuffer = robotPoseBuffer;
      this.controllerStateHandler = networkingManager.getControllerStateHandler();
      this.lidarDataFilter = lidarDataFilter;
      
      lidarStateCommandListener = new LidarStateCommandListener(networkingManager.getControllerCommandHandler());

      networkingManager.attachStateListener(this);
   }

   @Override
   public void connected()
   {
      synchronized (commandLock)
      {
         DepthDataClearCommand clearQuadTreeCommand = new DepthDataClearCommand(DepthDataTree.QUADTREE);
         DepthDataClearCommand clearOctTreeCommand = new DepthDataClearCommand(DepthDataTree.OCTREE);

         setLidarState(LidarState.DISABLE);
         lidarDataFilter.clearLidarData(DepthDataTree.QUADTREE);
         lidarDataFilter.clearLidarData(DepthDataTree.OCTREE);

         controllerStateHandler.sendPacket(clearQuadTreeCommand);
         controllerStateHandler.sendPacket(clearOctTreeCommand);
      }
   }

   @Override
   public void disconnected()
   {
      synchronized (commandLock)
      {
         sendData.set(false);
      }
   }

   @Override
   public void newRobotPoseReceived(RobotPoseData robotPosedata)
   {
   }

   public void setLidarState(LidarState lidarState)
   {
      synchronized (commandLock)
      {
         lidarDataFilter.setLidarState(lidarState);

         if (lidarState != LidarState.DISABLE)
         {
            sendData.set(true);
         }
         else
         {
            sendData.set(false);
         }

         controllerStateHandler.sendPacket(new DepthDataStateCommand(lidarState));
      }
   }

   protected void updatePointCloud(PointCloudPacket pointCloud)
   {
      if (pendingScans.size() > 40)
      {
         pendingScans.poll();
      }
      pendingScans.add(pointCloud);
      synchronizeScanWithPoseAndPublish();
   }

   private boolean isNewScanAvailable()
   {
      PointCloudPacket cloud;
      if ((cloud = pendingScans.peek()) != null)
      {
         long adjustedTimeStamp = cloud.getTimeStamp();
         return !robotPoseBuffer.isPending(adjustedTimeStamp);
      }

      return false;
   }

   private void synchronizeScanWithPoseAndPublish()
   {
      while (isNewScanAvailable())
      {
         PointCloudPacket pointCloud = pendingScans.poll();
         RigidBodyTransform transformToWorld = getPointCloudTransform(pointCloud.getTimeStamp());
         if (transformToWorld == null)
         {
            System.out.println("PointCloudDataReceiver: Transform to world start was null");
            return;
         }
         if (sendData.get())
         {
            synchronized (commandLock)
            {
               FilteredPointCloudPacket rosPointCloud = lidarDataFilter.filterAndTransformPointCloud(pointCloud,transformToWorld);

               if (rosPointCloud.getNumberOfPoints() > 0)
               {
                  controllerStateHandler.sendPacket(rosPointCloud);
               }
            }
         }
      }
   }

   private RigidBodyTransform getPointCloudTransform(long timeStamp)
   {
      RobotPoseData robotPoseData = robotPoseBuffer.interpolate(timeStamp);
      if (robotPoseData == null)
      {
         System.out.println("LidarDataReceiver: Robot Pose Data Null");
         return null;
      }

      RigidBodyTransform transform = new RigidBodyTransform(robotPoseData.getLidarPoses()[0]);

      return transform;
   }

   public class LidarStateCommandListener implements DepthDataStateCommandListenerInterface
   {
      public LidarStateCommandListener(NetworkProcessorControllerCommandHandler commandHandler)
      {
         commandHandler.attachListener(DepthDataStateCommand.class, new PacketConsumer<DepthDataStateCommand>()
         {
            public void receivedPacket(DepthDataStateCommand object)
            {
               setLidarState(object);
            }
         });
         
         commandHandler.attachListener(DepthDataClearCommand.class, new PacketConsumer<DepthDataClearCommand>()
         {
            public void receivedPacket(DepthDataClearCommand object)
            {
               clearLidar(object);
            }
         });
         
         commandHandler.attachListener(DepthDataFilterParameters.class, new PacketConsumer<DepthDataFilterParameters>()
         {
            public void receivedPacket(DepthDataFilterParameters object)
            {
               setFilterParameters(object);
            }
         });
      }
      
      public void setLidarState(DepthDataStateCommand lidarStateCommand)
      {
         synchronized (commandLock)
         {
            PointCloudDataReceiver.this.setLidarState(lidarStateCommand.getLidarState());
         }
      }

      public void clearLidar(DepthDataClearCommand object)
      {
         synchronized (commandLock)
         {
            lidarDataFilter.clearLidarData(object.getDepthDataTree());
            controllerStateHandler.sendPacket(object);
         }
      }

      public void setFilterParameters(DepthDataFilterParameters parameters)
      {
         synchronized (commandLock)
         {
            lidarDataFilter.setParameters(parameters);
            controllerStateHandler.sendPacket(parameters);
         }
      }
   }

}
