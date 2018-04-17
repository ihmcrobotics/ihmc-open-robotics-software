package us.ihmc.robotEnvironmentAwareness.communication.converters;

import java.util.List;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.jOctoMap.node.NormalOcTreeNode;
import us.ihmc.robotEnvironmentAwareness.communication.packets.LineSegment3DMessage;
import us.ihmc.robotEnvironmentAwareness.communication.packets.OcTreeKeyMessage;
import us.ihmc.robotEnvironmentAwareness.communication.packets.PlanarRegionSegmentationMessage;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationNodeData;
import us.ihmc.robotEnvironmentAwareness.updaters.RegionFeaturesProvider;

public class REAPlanarRegionsConverter
{
   public static PlanarRegionSegmentationMessage[] createPlanarRegionSegmentationMessages(RegionFeaturesProvider regionFeaturesProvider)
   {
      return createPlanarRegionSegmentationMessages(regionFeaturesProvider.getSegmentationNodeData());
   }

   public static PlanarRegionSegmentationMessage[] createPlanarRegionSegmentationMessages(List<PlanarRegionSegmentationNodeData> regionsNodeData)
   {
      PlanarRegionSegmentationMessage[] messages = new PlanarRegionSegmentationMessage[regionsNodeData.size()];

      for (int regionIndex = 0; regionIndex < regionsNodeData.size(); regionIndex++)
      {
         PlanarRegionSegmentationNodeData nodeData = regionsNodeData.get(regionIndex);
         messages[regionIndex] = createPlanarRegionSegmentationMessage(nodeData);
      }
      return messages;
   }

   public static PlanarRegionSegmentationMessage createPlanarRegionSegmentationMessage(PlanarRegionSegmentationNodeData nodeData)
   {
      int regionId = nodeData.getId();
      Point3D32 origin = new Point3D32(nodeData.getOrigin());
      Vector3D32 normal = new Vector3D32(nodeData.getNormal());
      OcTreeKeyMessage[] nodeKeys = new OcTreeKeyMessage[nodeData.getNumberOfNodes()];
      Point3D32[] nodeHitLocations = new Point3D32[nodeData.getNumberOfNodes()];

      for (int nodeIndex = 0; nodeIndex < nodeData.getNumberOfNodes(); nodeIndex++)
      {
         NormalOcTreeNode node = nodeData.getNode(nodeIndex);
         OcTreeKeyMessage nodeKey = OcTreeMessageConverter.createOcTreeKeyMessage(node.getKeyCopy());
         nodeKeys[nodeIndex] = nodeKey;
         nodeHitLocations[nodeIndex] = new Point3D32(node.getHitLocationCopy());
      }
      PlanarRegionSegmentationMessage planarRegionNodeKeysMessage = createPlanarRegionSegmentationMessage(regionId, origin, normal, nodeKeys, nodeHitLocations);
      return planarRegionNodeKeysMessage;
   }

   public static LineSegment3DMessage[] createLineSegment3dMessages(RegionFeaturesProvider regionFeaturesProvider)
   {
      LineSegment3DMessage[] messages = new LineSegment3DMessage[regionFeaturesProvider.getNumberOfPlaneIntersections()];

      for (int i = 0; i < regionFeaturesProvider.getNumberOfPlaneIntersections(); i++)
      {
         messages[i] = new LineSegment3DMessage();
         messages[i].start = new Point3D32(regionFeaturesProvider.getIntersection(i).getFirstEndpoint());
         messages[i].end = new Point3D32(regionFeaturesProvider.getIntersection(i).getSecondEndpoint());
      }
      return messages;
   }

   public static PlanarRegionSegmentationMessage createPlanarRegionSegmentationMessage(int id, Point3D origin, Vector3D normal,
                                                                                       OcTreeKeyMessage[] regionNodeKeys, List<Point3D> hitLocations)
   {
      PlanarRegionSegmentationMessage message = new PlanarRegionSegmentationMessage();
      message.id = id;
      message.origin = new Point3D32(origin);
      message.normal = new Vector3D32(normal);
      message.nodeKeys = regionNodeKeys;
      message.hitLocations = hitLocations.stream().map(Point3D32::new).toArray(Point3D32[]::new);
      return message;
   }

   public static PlanarRegionSegmentationMessage createPlanarRegionSegmentationMessage(int id, Point3D32 origin, Vector3D32 normal,
                                                                                       OcTreeKeyMessage[] regionNodeKeys, Point3D32[] hitLocations)
   {
      PlanarRegionSegmentationMessage message = new PlanarRegionSegmentationMessage();
      message.id = id;
      message.origin = origin;
      message.normal = normal;
      message.nodeKeys = regionNodeKeys;
      message.hitLocations = hitLocations;
      return message;
   }
}
