package us.ihmc.rdx.ui.behavior.registry;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeExecutor;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeStatus;
import us.ihmc.behaviors.behaviorTree.LocalOnlyBehaviorTreeNodeExecutor;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.sceneManager.RDXRenderableProvider;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.vr.RDXVRContext;

import java.util.ArrayList;
import java.util.Set;

/**
 * The UI has a tree structure, but not a decision or search one.
 * Currently calls propagate down to all the nodes so they can decide to take action.
 */
public abstract class RDXBehaviorUIInterface extends LocalOnlyBehaviorTreeNodeExecutor implements RDXRenderableProvider
{
   private final ArrayList<RDXBehaviorUIInterface> children = new ArrayList<>();

   protected RDXBehaviorUIInterface()
   {
   }

   public abstract void create(RDXBaseUI baseUI);

   public void handleVREvents(RDXVRContext vrContext)
   {

   }

   /**
    * Currently, nodes must remain at a fixed size when rendering them.
    * ImGui.dummy() can be used if node space should be reserved for later.
    */
   public abstract void renderTreeNodeImGuiWidgets();

   public abstract void update();

   public final void updateIncludingChildren()
   {
      update();

      for (RDXBehaviorUIInterface child : children)
      {
         child.updateIncludingChildren();
      }
   }

   public abstract void destroy();

   public void addChild(RDXBehaviorUIInterface child)
   {
      children.add(child);
   }

   public ArrayList<RDXBehaviorUIInterface> getUIChildren()
   {
      return children;
   }

   public void addChildPanels(RDXPanel parentPanel)
   {

   }

   public final void addChildPanelsIncludingChildren(RDXPanel parentPanel)
   {
      addChildPanels(parentPanel);

      for (RDXBehaviorUIInterface child : children)
      {
         child.addChildPanelsIncludingChildren(parentPanel);
      }
   }

   public void syncTree(BehaviorTreeNodeExecutor<?, ?> externalNode)
   {
//      getState().setStatus(externalNode.getState().getStatus());
      getDefinition().setDescription(externalNode.getDefinition().getDescription());

      for (BehaviorTreeNodeExecutor<?, ?> externalChild : externalNode.getChildren())
      {
         for (RDXBehaviorUIInterface child : children)
         {
            if (externalChild.getDefinition().getDescription().equals(child.getDefinition().getDescription()))
            {
               child.syncTree(externalChild);
            }
         }
      }
   }

   @Override
   public BehaviorTreeNodeStatus determineStatus()
   {
      return null;
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevels)
   {

   }

   public void clearChildren()
   {
      children.clear();
   }

   public int generateUID()
   {
      return toString().hashCode(); // Maybe change later? Works fine for now
   }

   @Override
   public String toString()
   {
      StringBuilder out = new StringBuilder();

      out.append(getClass().getSimpleName());
      out.append("(");
      for (RDXBehaviorUIInterface child : this.getUIChildren()) {
         out.append(child.toString()).append(",");
      }
      out.append(")");

      return out.toString();
   }
}
