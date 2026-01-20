package us.ihmc.behaviors.behaviorTree;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.behaviorTree.action.actions.AbilityHandActionComms;
import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneExecutor;
import us.ihmc.behaviors.tools.walkingController.ControllerStatusTracker;
import us.ihmc.perception.gpuMapping.TerrainMapData;
import us.ihmc.robotics.robotSide.SideDependentList;

import javax.annotation.Nullable;
import java.util.ArrayList;
import java.util.List;

/**
 * The base class for an executor node, which is the type that solely exists
 * on board the robot and controls the robot.
 *
 * @param <S> The type of this node's state instance.
 * @param <D> The type of this node's definition instance.
 */
public class BehaviorTreeNodeExecutor<S extends BehaviorTreeNodeState<D>,
                                      D extends BehaviorTreeNodeDefinition>
      implements BehaviorTreeNode<BehaviorTreeNodeExecutor<?, ?>, S, D>
{
   /** Convenient accessor to the state to keep the code clean, available to all inheriting classes. */
   protected final S state;
   /** Convenient accessor to the definition to keep the code clean, available to all inheriting classes. */
   protected final D definition;
   private final List<BehaviorTreeNodeExecutor<?, ?>> children = new ArrayList<>();
   protected final BehaviorTreeRootNodeExecutor rootNode;
   private transient BehaviorTreeNodeExecutor<?, ?> parent;

   protected final DRCRobotModel robotModel;
   protected final ROS2ControllerHelper ros2ControllerHelper;
   protected final ROS2SyncedRobotModel syncedRobot;
   protected final ControllerStatusTracker controllerStatusTracker;
   protected final SideDependentList<AbilityHandActionComms> abilityHandComms;
   protected final BehaviorTreeSceneExecutor scene;
   protected final TerrainMapData terrainMapData;

   /** For creating a basic node. */ // TODO: Should not exist???
   public BehaviorTreeNodeExecutor(long id, BehaviorTreeRootNodeExecutor rootNode)
   {
      this((S) new BehaviorTreeNodeState<D>(id, (D) new BehaviorTreeNodeDefinition(rootNode.getDefinition()), rootNode.getState()), rootNode);
   }

   /** For extending types. */
   public BehaviorTreeNodeExecutor(S state, BehaviorTreeRootNodeExecutor rootNode)
   {
      definition = state.getDefinition();
      this.state = state;
      this.rootNode = rootNode;
      this.robotModel = rootNode.getDefinition().getRobotModel();
      this.ros2ControllerHelper = rootNode.getRos2ControllerHelper();
      this.syncedRobot = rootNode.getSyncedRobot();
      this.controllerStatusTracker = rootNode.getControllerStatusTracker();
      this.abilityHandComms = rootNode.getAbilityHandComms();
      this.scene = rootNode.getScene();
      this.terrainMapData = rootNode.getTerrainMap();
   }

   /** Root node constructor. */
   public BehaviorTreeNodeExecutor(S state,
                                   ROS2ControllerHelper ros2ControllerHelper,
                                   ROS2SyncedRobotModel syncedRobot,
                                   ControllerStatusTracker controllerStatusTracker,
                                   SideDependentList<AbilityHandActionComms> abilityHandComms,
                                   BehaviorTreeSceneExecutor scene,
                                   TerrainMapData terrainMapData)
   {
      this.definition = state.getDefinition();
      this.state = state;
      this.rootNode = (BehaviorTreeRootNodeExecutor) this;
      this.robotModel = syncedRobot.getRobotModel();
      this.ros2ControllerHelper = ros2ControllerHelper;
      this.syncedRobot = syncedRobot;
      this.controllerStatusTracker = controllerStatusTracker;
      this.abilityHandComms = abilityHandComms;
      this.scene = scene;
      this.terrainMapData = terrainMapData;
   }

   /**
    * A method that should be called before each {@link #tick}
    * in order for nodes to know when they are no longer being selected.
    */
   public void clock()
   {
      state.setIsActive(false);

      for (BehaviorTreeNodeExecutor<?, ?> child : children)
      {
         child.clock();
      }
   }

   public void tick()
   {
      state.setIsActive(true);
   }

   public List<BehaviorTreeNodeExecutor<?, ?>> getChildren()
   {
      return children;
   }

   public BehaviorTreeRootNodeExecutor getRootNode()
   {
      return rootNode;
   }

   @Override
   public void setParent(@Nullable BehaviorTreeNodeExecutor<?, ?> parent)
   {
      this.parent = parent;
   }

   @Nullable
   @Override
   public BehaviorTreeNodeExecutor<?, ?> getParent()
   {
      return parent;
   }

   @Override
   public D getDefinition()
   {
      return definition;
   }

   @Override
   public S getState()
   {
      return state;
   }
}
