package us.ihmc.humanoidRobotics.communication.toolbox.heightQuadTree.command;

import javax.vecmath.Point3d;
import javax.vecmath.Point3f;
import javax.vecmath.Quat4d;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.packets.LidarScanMessage;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class LidarScanCommand implements Command<LidarScanCommand, LidarScanMessage>
{
   private long timestamp = -1L;
   private final RecyclingArrayList<Point3f> scan = new RecyclingArrayList<>(Point3f.class);
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
         Point3f point = scan.add();
         point.setX(newPointCloud[index++]);
         point.setY(newPointCloud[index++]);
         point.setZ(newPointCloud[index++]);
      }
   }

   public int getNumberOfPoints()
   {
      return scan.size();
   }

   public void getPoint(int index, Point3f pointToPack)
   {
      pointToPack.set(scan.get(index));
   }

   public void getPoint(int index, Point3d pointToPack)
   {
      pointToPack.set(scan.get(index));
   }

   public void getFramePoint(int index, FramePoint framePointToPack)
   {
      framePointToPack.setIncludingFrame(pointCloudFrame, scan.get(index));
   }

   public void getLidarPosition(Point3d positionToPack)
   {
      lidarPose.getPosition(positionToPack);
   }
   
   public void getLidarOrientation(Quat4d orientationToPack)
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
