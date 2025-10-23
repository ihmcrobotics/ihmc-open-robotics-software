package us.ihmc.behaviors.behaviorTree;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;

public class BehaviorTreeNonRootNodeState<D extends BehaviorTreeNodeDefinition> extends BehaviorTreeNodeState<D>
{
   protected final CRDTInfo crdtInfo; // convenient to have readily available
   protected final BehaviorTreeRootNodeState rootNode;
   protected final ReferenceFrameLibrary referenceFrameLibrary;
   protected final DRCRobotModel robotModel;

   public BehaviorTreeNonRootNodeState(long id, D definition, BehaviorTreeRootNodeState rootNode)
   {
      super(id, definition);

      this.crdtInfo = definition.getCRDTInfo();
      this.rootNode = rootNode;
      this.referenceFrameLibrary = rootNode.getReferenceFrameLibrary();
      this.robotModel = rootNode.getRobotModel();
   }
}
