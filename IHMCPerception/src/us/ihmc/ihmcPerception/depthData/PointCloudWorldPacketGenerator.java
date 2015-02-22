package us.ihmc.ihmcPerception.depthData;

import java.util.ArrayList;

import javax.vecmath.Point3d;

import com.google.gdata.client.youtube.YouTubeQuery.Time;

import us.ihmc.communication.packets.sensing.PointCloudWorldPacket;
import us.ihmc.userInterface.util.TimestampedPoint;

public class PointCloudWorldPacketGenerator 
{
   DepthDataFilter depthDataFilter;
   public PointCloudWorldPacketGenerator(DepthDataFilter depthDataFilter)
   {
      this.depthDataFilter = depthDataFilter;
   }
   
   public PointCloudWorldPacket getPointCloudWorldPacket()
   {
      PointCloudWorldPacket packet = new PointCloudWorldPacket();
      ArrayList<Point3d> groundPoints = new ArrayList<>();
      depthDataFilter.getQuadTree().getStoredPoints(groundPoints);
      packet.setGroundQuadTreeSupport(groundPoints.toArray(new Point3d[groundPoints.size()]));
      
      ArrayList<TimestampedPoint> nearScanTimestampedPoints = depthDataFilter.getNearScan().getPointsCopy();
      ArrayList<Point3d> nearScanPoints = new ArrayList<>();
      for(TimestampedPoint point:nearScanTimestampedPoints)
      {
         nearScanPoints.add(new Point3d(point.x, point.y,point.z));
      }
      packet.setDecayingWorldScan(nearScanPoints.toArray(new Point3d[nearScanPoints.size()]));
      
      return packet;
   }
}

