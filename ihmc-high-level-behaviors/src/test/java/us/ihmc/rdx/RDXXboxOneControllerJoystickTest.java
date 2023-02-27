package us.ihmc.rdx;

import com.badlogic.gdx.controllers.Controller;
import com.badlogic.gdx.controllers.Controllers;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDXBaseUI;

public class RDXXboxOneControllerJoystickTest
{

   private final RDXBaseUI baseUI = new RDXBaseUI();
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private Controller currentController;
   private boolean currentControllerConnected;

   public RDXXboxOneControllerJoystickTest()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            baseUI.getPrimaryScene().addRenderableProvider(this::getRenderables);
            baseUI.getImGuiPanelManager().addPanel("Xbox Controller", this::renderImGuiWidgets);
         }

         @Override
         public void render()
         {
            currentController = Controllers.getCurrent();
            currentControllerConnected = currentController != null;

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         private void renderImGuiWidgets()
         {
            ImGui.text("Current controller connected: " + currentControllerConnected);

            if (currentControllerConnected)
            {
               for (int i = currentController.getMinButtonIndex(); i < currentController.getMaxButtonIndex(); i++)
               {
                  ImGui.text("Button " + i + ": " + currentController.getButton(i));
               }

               for (int i = 0; i < currentController.getAxisCount(); i++)
               {
                  ImGui.text("Axis " + i + ": " + currentController.getAxis(i));
               }
            }
         }

         private void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
         {
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
      new RDXXboxOneControllerJoystickTest();
   }
}
