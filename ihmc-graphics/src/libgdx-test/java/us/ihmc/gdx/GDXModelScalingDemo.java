package us.ihmc.gdx;

import imgui.type.ImFloat;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.tools.GDXModelInstanceScaler;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.gizmo.GDXPose3DGizmo;

public class GDXModelScalingDemo
{
   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(),
                                                              "ihmc-open-robotics-software",
                                                              "ihmc-graphics/src/libgdx-test/resources",
                                                              "Scaling Demo");
   private final ImFloat scale = new ImFloat(1.0f);
   private boolean changed = false;
   private GDXPose3DGizmo gizmo;
   private GDXModelInstanceScaler scaledCouchModel;

   public GDXModelScalingDemo()
   {
      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            scaledCouchModel = new GDXModelInstanceScaler("Couch/Couch.g3dj", 1.0);
            baseUI.getPrimaryScene().addRenderableProvider(scaledCouchModel::getRenderables);

            baseUI.getPrimary3DPanel().getCamera3D().changeCameraPosition(-5.0, 5.0, 3.0);

            baseUI.getImGuiPanelManager().addPanel("Scaling", this::renderImGuiWidgets);

            gizmo = new GDXPose3DGizmo();
            gizmo.createAndSetupDefault(baseUI.getPrimary3DPanel());
         }

         @Override
         public void render()
         {
            if (changed)
            {
               scaledCouchModel.scale(scale.get());
            }

            GDXTools.toGDX(gizmo.getTransformToParent(), scaledCouchModel.getPoseTransform());

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         private void renderImGuiWidgets()
         {
            changed = ImGuiTools.volatileInputFloat("scale", scale);
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
      new GDXModelScalingDemo();
   }
}
