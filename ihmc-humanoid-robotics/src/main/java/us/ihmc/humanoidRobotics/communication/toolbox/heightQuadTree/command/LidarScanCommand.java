package us.ihmc.humanoidRobotics.communication.toolbox.heightQuadTree.command;

import controller_msgs.msg.dds.LidarScanMessage;
import gnu.trove.list.array.TFloatArrayList;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.lists.RecyclingArrayList;

public class LidarScanCommand implements Command<LidarScanCommand, LidarScanMessage>
{
   private long timestamp = -1L;
   private final RecyclingArrayList<Point3D32> scan = new RecyclingArrayList<>(Point3D32.class);
   private final ReferenceFrame pointCloudFrame = ReferenceFrame.getWorldFrame();
   private final FramePose3D lidarPose = new FramePose3D();

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
      timestamp = message.getRobotTimestamp();
      lidarPose.setIncludingFrame(ReferenceFrame.getWorldFrame(), message.getLidarPosition(), message.getLidarOrientation());

      int index = 0;
      TFloatArrayList newPointCloud = message.getScan();
      scan.clear();

      while (index < newPointCloud.size())
      {
         Point3D32 point = scan.add();
         point.setX(newPointCloud.get(index++));
         point.setY(newPointCloud.get(index++));
         point.setZ(newPointCloud.get(index++));
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

   public void getFramePoint(int index, FramePoint3D framePointToPack)
   {
      framePointToPack.setIncludingFrame(pointCloudFrame, scan.get(index));
   }

   public void getLidarPosition(Point3D positionToPack)
   {
      positionToPack.set(lidarPose.getPosition());
   }
   
   public void getLidarOrientation(Quaternion orientationToPack)
   {
      orientationToPack.set(lidarPose.getOrientation());
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
