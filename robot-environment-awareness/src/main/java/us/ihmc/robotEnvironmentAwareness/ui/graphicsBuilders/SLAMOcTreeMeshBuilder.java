package us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders;

import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.robotEnvironmentAwareness.communication.REAUIMessager;
import us.ihmc.robotEnvironmentAwareness.communication.packets.NormalOcTreeMessage;

public class SLAMOcTreeMeshBuilder extends OcTreeMeshBuilder
{
   public SLAMOcTreeMeshBuilder(REAUIMessager uiMessager, Topic<Boolean> octreeEnableTopic, Topic<Boolean> clearTopic,
                                Topic<NormalOcTreeMessage> octreeStateTopic, Topic<DisplayType> displayTypeTopic)
   {
      super(uiMessager, octreeEnableTopic, clearTopic, octreeStateTopic, displayTypeTopic);
      coloringType.setValue(ColoringType.HAS_CENTER);
      displayType.setValue(DisplayType.HIT_LOCATION);
      treeDepthForDisplay.setValue(15);
      hidePlanarRegionNodes.setValue(false);
      setOctreeHitLocationVizSize(0.02);
   }
}