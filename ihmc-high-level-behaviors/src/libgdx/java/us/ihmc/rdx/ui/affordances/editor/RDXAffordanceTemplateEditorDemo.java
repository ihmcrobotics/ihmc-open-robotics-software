package us.ihmc.rdx.ui.affordances.editor;

import us.ihmc.perception.sceneGraph.SceneGraph;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;

public class RDXAffordanceTemplateEditorDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private RDXAffordanceTemplateEditorUI editor;
   private final SceneGraph sceneGraph = new SceneGraph(null);

   public RDXAffordanceTemplateEditorDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();
            editor = new RDXAffordanceTemplateEditorUI(baseUI, sceneGraph);
         }

         @Override
         public void render()
         {
            editor.update();
            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXAffordanceTemplateEditorDemo();
   }
}