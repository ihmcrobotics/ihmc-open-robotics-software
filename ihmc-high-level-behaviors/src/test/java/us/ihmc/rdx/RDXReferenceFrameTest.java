package us.ihmc.rdx;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImDouble;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;

public class RDXReferenceFrameTest
{

   private final RDXBaseUI baseUI = new RDXBaseUI();
   private RigidBodyTransform userTransformToParent;
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean showWorldFrame = new ImBoolean(true);
   private final ImBoolean showUserFrame = new ImBoolean(true);
   private final ImDouble yaw = new ImDouble();
   private final ImDouble pitch = new ImDouble();
   private final ImDouble roll = new ImDouble();
   private RDXReferenceFrameGraphic userReferenceFrameGraphic;

   public RDXReferenceFrameTest()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {

         private ReferenceFrame userReferenceFrame;
         private ModelInstance worldFrameGraphic;

         @Override
         public void create()
         {
            baseUI.create();

            LibGDXTools.setOpacity(worldFrameGraphic, 0.6f);

            userTransformToParent = new RigidBodyTransform();
            userReferenceFrame = ReferenceFrameTools.constructFrameWithChangingTransformToParent("zForwardXRightYDownFrame",
                                                                                                 ReferenceFrame.getWorldFrame(), userTransformToParent);
            userReferenceFrameGraphic = new RDXReferenceFrameGraphic(0.2);
            userReferenceFrameGraphic.setToReferenceFrame(userReferenceFrame);

            baseUI.getPrimaryScene().addRenderableProvider(this::getRenderables);
            baseUI.getImGuiPanelManager().addPanel("Reference Frames", this::renderImGuiWidgets);
         }

         private void renderImGuiWidgets()
         {
            ImGui.checkbox(labels.get("Show World Frame"), showWorldFrame);
            ImGui.checkbox(labels.get("Show User Frame"), showUserFrame);
            boolean input = false;
            input |= ImGui.inputDouble(labels.get("Yaw"), yaw);
            input |= ImGui.inputDouble(labels.get("Pitch"), pitch);
            input |= ImGui.inputDouble(labels.get("Roll"), roll);

            if (input)
            {
               userTransformToParent.getRotation().setYawPitchRoll(Math.toRadians(yaw.get()), Math.toRadians(pitch.get()), Math.toRadians(roll.get()));
               userReferenceFrame.update();
               userReferenceFrameGraphic.setToReferenceFrame(userReferenceFrame);
            }
         }

         @Override
         public void render()
         {
            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         private void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
         {
            if (showWorldFrame.get())
               worldFrameGraphic.getRenderables(renderables, pool);
            if (showUserFrame.get())
               userReferenceFrameGraphic.getRenderables(renderables, pool);
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
      new RDXReferenceFrameTest();
   }
}
