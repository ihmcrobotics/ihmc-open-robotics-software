package us.ihmc.rdx.ui.affordances.editor;

import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.ui.RDXBaseUI;

public class RDXAffordanceTemplateEditorDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private RDXAffordanceTemplateEditorUI editor;

   public RDXAffordanceTemplateEditorDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();
            editor = new RDXAffordanceTemplateEditorUI(baseUI);
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