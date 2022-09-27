package us.ihmc.gdx;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import imgui.type.ImFloat;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.tools.GDXModelBuilder;
import us.ihmc.gdx.tools.GDXModelInstanceScaler;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.gizmo.GDXPose3DGizmo;
import us.ihmc.log.LogTools;

public class GDXModelScalingDemo
{
   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(),
                                                              "ihmc-open-robotics-software",
                                                              "ihmc-graphics/src/libgdx-test/resources",
                                                              "Scaling Demo");
   private final ImFloat scale = new ImFloat(1.0f);
   private boolean changed = false;
   private GDXPose3DGizmo gizmo;
   private GDXModelInstanceScaler scaledModel;

   public GDXModelScalingDemo()
   {
      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            scaledModel = new GDXModelInstanceScaler("right_wrist_roll_gripper.g3dj", 1.0);
            baseUI.getPrimaryScene().addRenderableProvider(scaledModel::getRenderables);

            ModelInstance sphere = GDXModelBuilder.createSphere(0.015f, Color.RED);
            GDXTools.toGDX(scaledModel.getWholeModelCentroid(), sphere.transform);
            LogTools.info(scaledModel.getWholeModelCentroid());
            baseUI.getPrimaryScene().addModelInstance(sphere);

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
               scaledModel.scale(scale.get());
            }

            GDXTools.toGDX(gizmo.getTransformToParent(), scaledModel.getPoseTransform());

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
