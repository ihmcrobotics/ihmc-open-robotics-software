package us.ihmc.rdx.ui.affordances.editor;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;

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
            baseUI.getPrimaryScene().addModelInstance(new ModelInstance(RDXModelBuilder.createCoordinateFrame(0.3)));
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