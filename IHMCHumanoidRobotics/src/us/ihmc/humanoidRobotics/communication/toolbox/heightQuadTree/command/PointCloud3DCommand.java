package us.ihmc.humanoidRobotics.communication.toolbox.heightQuadTree.command;

import javax.vecmath.Point3d;
import javax.vecmath.Point3f;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.humanoidRobotics.communication.packets.sensing.PointCloudWorldPacket;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class PointCloud3DCommand implements Command<PointCloud3DCommand, PointCloudWorldPacket>
{
   private long timestamp = -1L;
   private final RecyclingArrayList<Point3f> pointCloud = new RecyclingArrayList<>(Point3f.class);
   private final ReferenceFrame pointCloudFrame = ReferenceFrame.getWorldFrame();

   @Override
   public void clear()
   {
      timestamp = -1L;
      pointCloud.clear();
   }

   @Override
   public void set(PointCloud3DCommand other)
   {
      timestamp = other.timestamp;
      pointCloud.clear();

      for (int i = 0; i < other.pointCloud.size(); i++)
         pointCloud.add().set(other.pointCloud.get(i));
   }

   @Override
   public void set(PointCloudWorldPacket message)
   {
      timestamp = message.timestamp;

      int index = 0;
      float[] newPointCloud = message.decayingWorldScan;
      pointCloud.clear();

      while (index < newPointCloud.length)
      {
         Point3f point = pointCloud.add();
         point.setX(newPointCloud[index++]);
         point.setY(newPointCloud[index++]);
         point.setZ(newPointCloud[index++]);
      }
   }

   public int getNumberOfPoints()
   {
      return pointCloud.size();
   }

   public void getPoint(int index, Point3f pointToPack)
   {
      pointToPack.set(pointCloud.get(index));
   }

   public void getPoint(int index, Point3d pointToPack)
   {
      pointToPack.set(pointCloud.get(index));
   }

   public void getFramePoint(int index, FramePoint framePointToPack)
   {
      framePointToPack.setIncludingFrame(pointCloudFrame, pointCloud.get(index));
   }

   @Override
   public Class<PointCloudWorldPacket> getMessageClass()
   {
      return PointCloudWorldPacket.class;
   }

   @Override
   public boolean isCommandValid()
   {
      return timestamp != -1L && !pointCloud.isEmpty();
   }
}
