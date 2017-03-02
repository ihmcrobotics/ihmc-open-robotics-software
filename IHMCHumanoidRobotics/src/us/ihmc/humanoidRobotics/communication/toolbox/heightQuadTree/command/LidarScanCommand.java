package us.ihmc.humanoidRobotics.communication.toolbox.heightQuadTree.command;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.packets.LidarScanMessage;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class LidarScanCommand implements Command<LidarScanCommand, LidarScanMessage>
{
   private long timestamp = -1L;
   private final RecyclingArrayList<Point3D32> scan = new RecyclingArrayList<>(Point3D32.class);
   private final ReferenceFrame pointCloudFrame = ReferenceFrame.getWorldFrame();
   private final FramePose lidarPose = new FramePose();

   @Override
   public void clear()
   {
      timestamp = -1L;
      scan.clear();
   }

   @Override
   public void set(LidarScanCommand other)
   {
      timestamp = other.timestamp;
      lidarPose.setIncludingFrame(other.lidarPose);
      scan.clear();

      for (int i = 0; i < other.scan.size(); i++)
         scan.add().set(other.scan.get(i));
   }

   @Override
   public void set(LidarScanMessage message)
   {
      timestamp = message.robotTimestamp;
      message.getLidarPose(lidarPose);

      int index = 0;
      float[] newPointCloud = message.scan;
      scan.clear();

      while (index < newPointCloud.length)
      {
         Point3D32 point = scan.add();
         point.setX(newPointCloud[index++]);
         point.setY(newPointCloud[index++]);
         point.setZ(newPointCloud[index++]);
      }
   }

   public int getNumberOfPoints()
   {
      return scan.size();
   }

   public void getPoint(int index, Point3D32 pointToPack)
   {
      pointToPack.set(scan.get(index));
   }

   public void getPoint(int index, Point3D pointToPack)
   {
      pointToPack.set(scan.get(index));
   }

   public void getFramePoint(int index, FramePoint framePointToPack)
   {
      framePointToPack.setIncludingFrame(pointCloudFrame, scan.get(index));
   }

   public void getLidarPosition(Point3D positionToPack)
   {
      lidarPose.getPosition(positionToPack);
   }
   
   public void getLidarOrientation(Quaternion orientationToPack)
   {
      lidarPose.getOrientation(orientationToPack);
   }

   @Override
   public Class<LidarScanMessage> getMessageClass()
   {
      return LidarScanMessage.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return timestamp != -1L && !scan.isEmpty();
   }
}
