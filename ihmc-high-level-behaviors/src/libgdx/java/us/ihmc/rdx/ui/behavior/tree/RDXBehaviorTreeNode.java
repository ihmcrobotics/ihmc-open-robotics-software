package us.ihmc.rdx.ui.behavior.tree;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.behaviors.behaviorTree.*;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.log.LogTools;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.ImStringWrapper;
import us.ihmc.rdx.input.ImGui3DViewInput;
import us.ihmc.rdx.vr.RDXVRContext;

import java.util.ArrayList;
import java.util.List;

public abstract class RDXBehaviorTreeNode<S extends BehaviorTreeNodeState<D>,
                                          D extends BehaviorTreeNodeDefinition>
      implements BehaviorTreeNodeExtension<RDXBehaviorTreeNode<?, ?>, S, S, D>
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private ImStringWrapper descriptionWrapper;
   private final TypedNotification<Boolean> expandCollapseRequest = new TypedNotification<>();

   private final List<RDXBehaviorTreeNode<?, ?>> children = new ArrayList<>();

   public void update()
   {
      if (descriptionWrapper == null)
      {
         descriptionWrapper = new ImStringWrapper(getDefinition()::getDescription,
                                                  getDefinition()::setDescription,
                                                  imString -> ImGuiTools.inputText(labels.getHidden("description"), imString));
      }
   }

   public void calculateVRPick(RDXVRContext vrContext)
   {

   }

   public void processVRInput(RDXVRContext vrContext)
   {

   }

   public void calculate3DViewPick(ImGui3DViewInput input)
   {

   }

   public void process3DViewInput(ImGui3DViewInput input)
   {

   }

   public void renderImGuiWidgets()
   {

   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {

   }

   @Override
   public void destroy()
   {
      LogTools.info("Destroying node: {}:{}", getState().getDefinition().getDescription(), getState().getID());
      getState().destroy();
   }

   public ImStringWrapper getDescriptionWrapper()
   {
      return descriptionWrapper;
   }

   public TypedNotification<Boolean> getExpandCollapseRequest()
   {
      return expandCollapseRequest;
   }

   public List<RDXBehaviorTreeNode<?, ?>> getChildren()
   {
      return children;
   }

   @Override
   public S getExtendedNode()
   {
      return getState();
   }

   @Override
   public D getDefinition()
   {
      return getState().getDefinition();
   }
}
