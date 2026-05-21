package us.ihmc.rdx.ui.graphics;

import perception_msgs.FramePlanarRegionsListMessage;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.jros2.ROS2Node;

import java.util.concurrent.atomic.AtomicReference;

public class RDXMultiContactRegionHelper
{
   private final AtomicReference<FramePlanarRegionsListMessage> latestPlanarRegionsMessage = new AtomicReference<>();
   private final FullHumanoidRobotModel ghostFullRobotModel;

   public RDXMultiContactRegionHelper(FullHumanoidRobotModel ghostFullRobotModel, ROS2Node ros2Node)
   {
      this.ghostFullRobotModel = ghostFullRobotModel;
      ros2Node.createSubscription(PerceptionAPI.PERSPECTIVE_RAPID_REGIONS, s -> latestPlanarRegionsMessage.set(s.read()));
   }

   public Vector3D getNormalOfClosestRegion(RobotSide robotSide)
   {
      FramePoint3D handPosition = new FramePoint3D(ghostFullRobotModel.getHandControlFrame(robotSide));
      handPosition.changeFrame(ReferenceFrame.getWorldFrame());

      // Check for new surface normal
      FramePlanarRegionsListMessage planarRegionsMessage = latestPlanarRegionsMessage.getAndSet(null);
      if (planarRegionsMessage == null)
      {
         return null;
      }

      PlanarRegionsList planarRegions = PlanarRegionMessageConverter.convertToPlanarRegionsListInWorld(planarRegionsMessage);

      double minimumDistance = Double.POSITIVE_INFINITY;
      Vector3D normal = new Vector3D();

      for (int i = 0; i < planarRegions.getNumberOfPlanarRegions(); i++)
      {
         PlanarRegion region = planarRegions.getPlanarRegion(i);
         region.updateConvexHull();

         double area = region.getArea();
         double areaThreshold = 0.2;

         if (area < areaThreshold)
            continue;

         double distance = region.distance(handPosition);
         if (distance < minimumDistance)
         {
            minimumDistance = distance;
            normal.set(region.getNormal());
         }
      }

      if (Double.isInfinite(minimumDistance) || minimumDistance > 0.1)
      {
         return null;
      }
      else
      {
         return normal;
      }
   }
}
