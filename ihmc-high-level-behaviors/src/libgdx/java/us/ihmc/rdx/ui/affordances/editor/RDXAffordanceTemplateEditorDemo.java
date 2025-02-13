package us.ihmc.rdx.ui.affordances.editor;

import us.ihmc.rdx.gdx.RDXLwjgl3ApplicationAdapter;
import us.ihmc.rdx.RDXBaseUI;

public class RDXAffordanceTemplateEditorDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private RDXAffordanceTemplateEditorUI editor;

   public RDXAffordanceTemplateEditorDemo()
   {
      baseUI.launchRDXApplication(new RDXLwjgl3ApplicationAdapter()
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