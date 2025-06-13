package us.ihmc.rdx;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import imgui.ImGui;
import imgui.type.ImFloat;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.tools.RDXModelInstanceScaler;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;

public class RDXModelScalingDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI("Scaling Demo");
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImFloat scale = new ImFloat(1.0f);
   private RDXPose3DGizmo gizmo;
   private RDXModelInstanceScaler scaledModel;

   public RDXModelScalingDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            scaledModel = new RDXModelInstanceScaler("right_wrist_roll_gripper.g3dj");
            baseUI.getPrimaryScene().addRenderableProvider(scaledModel::getRenderables);

            baseUI.getPrimary3DPanel().getCamera3D().changeCameraPosition(-5.0, 5.0, 3.0);

            baseUI.getImGuiPanelManager().addPanel("Scaling", this::renderImGuiWidgets);

            gizmo = new RDXPose3DGizmo();
            gizmo.createAndSetupDefault(baseUI.getPrimary3DPanel());
         }

         @Override
         public void render()
         {
            LibGDXTools.toLibGDX(gizmo.getTransformToParent(), scaledModel.getPoseTransform());

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         private void renderImGuiWidgets()
         {
            if (ImGui.sliderFloat(labels.get("Scale"), scale.getData(), 0.1f, 10.0f))
            {
               scaledModel.scale(scale.get());
            }
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
      new RDXModelScalingDemo();
   }
}
