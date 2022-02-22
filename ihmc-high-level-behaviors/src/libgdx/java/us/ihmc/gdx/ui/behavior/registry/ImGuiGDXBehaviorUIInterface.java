package us.ihmc.gdx.ui.behavior.registry;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.behaviors.tools.behaviorTree.*;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.vr.GDXVRContext;

import java.util.ArrayList;

/**
 * The UI has a tree structure, but not a decision or search one.
 * Currently calls propagate down to all the nodes so they can decide to take action.
 */
public abstract class ImGuiGDXBehaviorUIInterface extends BehaviorTreeNode implements RenderableProvider
{
   private final ArrayList<ImGuiGDXBehaviorUIInterface> children = new ArrayList<>();

   protected ImGuiGDXBehaviorUIInterface()
   {
   }

   public abstract void create(GDXImGuiBasedUI baseUI);

   public void handleVREvents(GDXVRContext vrContext)
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

      for (ImGuiGDXBehaviorUIInterface child : children)
      {
         child.updateIncludingChildren();
      }
   }

   public abstract void destroy();

   public void addChild(ImGuiGDXBehaviorUIInterface child)
   {
      children.add(child);
   }

   public ArrayList<ImGuiGDXBehaviorUIInterface> getUIChildren()
   {
      return children;
   }

   public void addChildPanels(ImGuiPanel parentPanel)
   {

   }

   public final void addChildPanelsIncludingChildren(ImGuiPanel parentPanel)
   {
      addChildPanels(parentPanel);

      for (ImGuiGDXBehaviorUIInterface child : children)
      {
         child.addChildPanelsIncludingChildren(parentPanel);
      }
   }

   public void syncTree(BehaviorTreeNodeBasics externalNode)
   {
      setPreviousStatus(externalNode.getPreviousStatus());
      setName(externalNode.getName());
      setLastTickMillis(externalNode.getLastTickMillis());
      setType(externalNode.getType());

      if (externalNode instanceof BehaviorTreeControlFlowNodeBasics)
      {
         BehaviorTreeControlFlowNodeBasics externalControlFlowNode = (BehaviorTreeControlFlowNodeBasics) externalNode;
         for (BehaviorTreeNodeBasics externalChild : externalControlFlowNode.getChildren())
         {
            for (ImGuiGDXBehaviorUIInterface child : children)
            {
               if (externalChild.getName().equals(child.getName()))
               {
                  child.syncTree(externalChild);
               }
            }
         }
      }
   }

   @Override
   public BehaviorTreeNodeStatus tickInternal()
   {
      return null;
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
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

      out.append(this.getType());
      out.append("(");
      for (ImGuiGDXBehaviorUIInterface child : this.getUIChildren()) {
         out.append(child.toString()).append(",");
      }
      out.append(")");

      return out.toString();
   }
}
