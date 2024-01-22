package us.ihmc.rdx.ui.tools;

import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.perception.sceneGraph.SceneNode;
import us.ihmc.rdx.imgui.ImGuiReferenceFrameLibraryCombo;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;

import java.util.function.Consumer;
import java.util.function.Supplier;

public class ImGuiSceneGraphFramesCombo extends ImGuiReferenceFrameLibraryCombo
{
   private final SceneGraph sceneGraph;

   public ImGuiSceneGraphFramesCombo(String comboName,
                                     ReferenceFrameLibrary referenceFrameLibrary,
                                     SceneGraph sceneGraph,
                                     Supplier<String> currentFrameNameGetter,
                                     Consumer<String> currentFrameNameSetter)
   {
      super(comboName, referenceFrameLibrary, currentFrameNameGetter, currentFrameNameSetter);
      this.sceneGraph = sceneGraph;
   }

   @Override
   public void render()
   {
      renderBefore();

      addSceneGraphFrames(sceneGraph.getRootNode());

      renderAfter();
   }

   private void addSceneGraphFrames(SceneNode sceneNode)
   {
      getReferenceFrameLibraryNames().add(sceneNode.getNodeFrame().getName());

      for (SceneNode child : sceneNode.getChildren())
      {
         addSceneGraphFrames(child);
      }
   }
}
