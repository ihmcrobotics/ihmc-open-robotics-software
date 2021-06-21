package us.ihmc.gdx.ui.behaviors.registry;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.behaviors.tools.behaviorTree.*;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.vr.GDXVRManager;

import java.util.ArrayList;

/**
 * The UI has a tree structure, but not a decision or search one.
 * Currently calls propagate down to all the nodes so they can decide to take action.
 */
public abstract class GDXBehaviorUIInterface extends BehaviorTreeNode implements RenderableProvider
{
   private final ArrayList<GDXBehaviorUIInterface> children = new ArrayList<>();

   protected GDXBehaviorUIInterface()
   {
   }

   public abstract void create(GDXImGuiBasedUI baseUI);

//   public abstract void setEnabled(boolean enabled);

   public void handleVREvents(GDXVRManager vrManager)
   {

   }

   public abstract void renderTreeNodeImGuiWidgets();

   public abstract void renderRegularPanelImGuiWidgets();

   public abstract void update();

   public final void updateIncludingChildren()
   {
      update();

      for (GDXBehaviorUIInterface child : children)
      {
         child.updateIncludingChildren();
      }
   }

   public final void renderRegularPanelImGuiWidgetsAndChildren()
   {
      renderRegularPanelImGuiWidgets();

      for (GDXBehaviorUIInterface child : children)
      {
         child.renderRegularPanelImGuiWidgetsAndChildren();
      }
   }

   public abstract void destroy();

   public abstract Point2D getTreeNodeInitialPosition();

   public void addChild(GDXBehaviorUIInterface child)
   {
      children.add(child);
   }

   public ArrayList<GDXBehaviorUIInterface> getUIChildren()
   {
      return children;
   }

   public void addChildPanels(ImGuiPanel parentPanel)
   {

   }

   public final void addChildPanelsIncludingChildren(ImGuiPanel parentPanel)
   {
      addChildPanels(parentPanel);

      for (GDXBehaviorUIInterface child : children)
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
            for (GDXBehaviorUIInterface child : children)
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

   public int generateUID() {
      return this.toString().hashCode(); //maybe change later? works fine for now
   }

   @Override
   public String toString() {
      StringBuilder out = new StringBuilder();

      out.append(this.getType());
      out.append("(");
      for (GDXBehaviorUIInterface child : this.getUIChildren()) {
         out.append(child.toString()).append(",");
      }
      out.append(")");

      return out.toString();
   }

   //   protected void enable3DVisualizations()
//   {
//   }
//
//   protected ROS2NodeInterface getRos2Node()
//   {
//      return ros2Node;
//   }
//
//   protected Messager getBehaviorMessager()
//   {
//      return behaviorMessager;
//   }
//
//   protected DRCRobotModel getRobotModel()
//   {
//      return robotModel;
//   }
}
