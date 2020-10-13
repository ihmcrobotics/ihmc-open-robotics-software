package us.ihmc.avatar.slamTools;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.jOctoMap.iterators.OcTreeIterable;
import us.ihmc.jOctoMap.iterators.OcTreeIteratorFactory;
import us.ihmc.jOctoMap.node.NormalOcTreeNode;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;

public class OctreeYoGraphicsManager
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoFramePoint3D[] yoFrameNodeHitLocations;
   private final YoGraphicPosition[] yoGraphicNodeHitLocations;

   private static final double NORMAL_VIZ_LENGTH = 0.03;

   private final YoFrameVector3D[] yoFrameNormals;
   private final YoGraphicVector[] yoGraphicNormals;

   public OctreeYoGraphicsManager(String prefix, NormalOcTree octree, AppearanceDefinition appearance, YoRegistry registry,
                                  YoGraphicsListRegistry graphicsRegistry, boolean visualizeNormal)
   {
      YoGraphicsList yoGraphicListRegistry = new YoGraphicsList(prefix + "_Octree_Viz");

      int numberOfNodes = octree.getNumberOfNodes();

      yoFrameNodeHitLocations = new YoFramePoint3D[numberOfNodes];
      yoGraphicNodeHitLocations = new YoGraphicPosition[numberOfNodes];
      for (int i = 0; i < numberOfNodes; i++)
      {
         yoFrameNodeHitLocations[i] = new YoFramePoint3D(prefix + "_OctreeNode_" + i, worldFrame, registry);
         yoGraphicNodeHitLocations[i] = new YoGraphicPosition(prefix + "_OctreeNodeViz_" + i, yoFrameNodeHitLocations[i], 0.003, appearance);
         yoGraphicListRegistry.add(yoGraphicNodeHitLocations[i]);
      }

      yoFrameNormals = new YoFrameVector3D[numberOfNodes];
      yoGraphicNormals = new YoGraphicVector[numberOfNodes];
      for (int i = 0; i < numberOfNodes; i++)
      {
         yoFrameNormals[i] = new YoFrameVector3D(prefix + "_SurfelNormal_" + i, worldFrame, registry);
         yoGraphicNormals[i] = new YoGraphicVector(prefix + "_SurfelNormalViz_"
               + i, yoFrameNodeHitLocations[i], yoFrameNormals[i], NORMAL_VIZ_LENGTH, appearance, false);

         if (visualizeNormal)
            yoGraphicListRegistry.add(yoGraphicNormals[i]);
      }

      OcTreeIterable<NormalOcTreeNode> iterable = OcTreeIteratorFactory.createIterable(octree.getRoot());
      int index = 0;
      for (NormalOcTreeNode node : iterable)
      {
         Vector3D normal = new Vector3D();
         Point3D hitLocation = new Point3D();
         node.getNormal(normal);
         node.getHitLocation(hitLocation);

         yoFrameNodeHitLocations[index].set(hitLocation);
         yoFrameNormals[index].set(normal);

         index++;
      }

      graphicsRegistry.registerYoGraphicsList(yoGraphicListRegistry);
   }
}
