package us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders;

import java.util.Map;

import us.ihmc.jOctoMap.key.OcTreeKey;
import us.ihmc.log.LogTools;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.REAUIMessager;
import us.ihmc.robotEnvironmentAwareness.communication.packets.NormalOcTreeMessage;
import us.ihmc.robotEnvironmentAwareness.ui.UIOcTree;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.OcTreeMeshBuilder.DisplayType;

public class SLAMOcTreeMeshBuilder extends OcTreeMeshBuilder
{
   public SLAMOcTreeMeshBuilder(REAUIMessager uiMessager, Topic<Boolean> octreeEnableTopic, Topic<NormalOcTreeMessage> octreeStateTopic)
   {
      super(uiMessager, octreeEnableTopic, octreeStateTopic);
      coloringType.setValue(ColoringType.DEFAULT);
      displayType.setValue(DisplayType.HIT_LOCATION);
      treeDepthForDisplay.setValue(15);
      hidePlanarRegionNodes.setValue(false);
   }
   
//   @Override
//   public void run()
//   {
//      if (newSubOcTreeMeshViews.get() != null)
//      {
//         LogTools.warn("Rendering job is not done, waiting before creating new meshes.");
//         return;
//      }
//
//      if (displayType.getValue() == DisplayType.HIDE)
//         return;
//
//      if (enable.get())
//      {
//         NormalOcTreeMessage newMessage = ocTreeState.get();
//         Map<OcTreeKey, Integer> nodeKeyToRegionIdMap = createNodeKeyToRegionIdMap(planarRegionSegmentationState.getAndSet(null));
//
//         if (newMessage == null || nodeKeyToRegionIdMap == null)
//            return;
//
//         uiOcTree.set(new UIOcTree(ocTreeState.getAndSet(null), nodeKeyToRegionIdMap));
//         buildUIOcTreeMesh(uiOcTree.get());
//      }
//      else if (processChange.getAndSet(false) && uiOcTree.get() != null)
//      {
//         buildUIOcTreeMesh(uiOcTree.get());
//      }
//   }
}