package us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData;

import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.atomic.AtomicBoolean;

import javax.media.j3d.Transform3D;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.communication.packets.sensing.DepthDataClearCommand;
import us.ihmc.communication.packets.sensing.DepthDataClearCommand.DepthDataTree;
import us.ihmc.communication.packets.sensing.DepthDataFilterParameters;
import us.ihmc.communication.packets.sensing.DepthDataStateCommand;
import us.ihmc.communication.packets.sensing.PointCloudPacket;
import us.ihmc.communication.packets.sensing.DepthDataStateCommand.LidarState;
import us.ihmc.communication.packets.sensing.FilteredPointCloudPacket;
import us.ihmc.communication.packets.sensing.RobotPoseData;
import us.ihmc.communication.producers.RobotPoseBuffer;
import us.ihmc.communication.producers.RobotPoseBufferListener;
import us.ihmc.darpaRoboticsChallenge.networking.DRCNetworkProcessorControllerStateHandler;
import us.ihmc.darpaRoboticsChallenge.networking.DRCNetworkProcessorNetworkingManager;
import us.ihmc.utilities.net.NetStateListener;

public class PointCloudDataReceiver implements RobotPoseBufferListener, NetStateListener
{

   protected ConcurrentLinkedQueue<PointCloudPacket> pendingScans = new ConcurrentLinkedQueue<PointCloudPacket>();
   protected volatile AtomicBoolean sendData = new AtomicBoolean(false);
   private final Object commandLock = new Object();
   private final DepthDataFilter lidarDataFilter;
   private final DRCNetworkProcessorControllerStateHandler controllerStateHandler;
   protected LidarStateCommandListener lidarStateCommandListener = new LidarStateCommandListener();
   private final RobotPoseBuffer robotPoseBuffer;

   public PointCloudDataReceiver(RobotPoseBuffer robotPoseBuffer, DRCNetworkProcessorNetworkingManager networkingManager, SDFFullRobotModel fullRobotModel,
         DepthDataFilter lidarDataFilter)
   {
      robotPoseBuffer.addListener(this);

      this.robotPoseBuffer = robotPoseBuffer;
      this.controllerStateHandler = networkingManager.getControllerStateHandler();
      this.lidarDataFilter = lidarDataFilter;
      networkingManager.getControllerCommandHandler().setDepthDataCommandListener(lidarStateCommandListener);

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

         controllerStateHandler.sendSerializableObject(clearQuadTreeCommand);
         controllerStateHandler.sendSerializableObject(clearOctTreeCommand);
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

         controllerStateHandler.sendSerializableObject(new DepthDataStateCommand(lidarState));
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
         Transform3D transformToWorld = getPointCloudTransform(pointCloud.getTimeStamp());
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
                  controllerStateHandler.sendSerializableObject(rosPointCloud);
               }
            }
         }
      }
   }

   private Transform3D getPointCloudTransform(long timeStamp)
   {
      RobotPoseData robotPoseData = robotPoseBuffer.interpolate(timeStamp);
      if (robotPoseData == null)
      {
         System.out.println("LidarDataReceiver: Robot Pose Data Null");
         return null;
      }

      Transform3D transform = new Transform3D(robotPoseData.getLidarPoses()[0]);

      return transform;
   }

   public class LidarStateCommandListener implements DepthDataStateCommandListenerInterface
   {
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
            controllerStateHandler.sendSerializableObject(object);
         }
      }

      public void setFilterParameters(DepthDataFilterParameters parameters)
      {
         synchronized (commandLock)
         {
            lidarDataFilter.setParameters(parameters);
            controllerStateHandler.sendSerializableObject(parameters);
         }
      }
   }

}
