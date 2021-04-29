package us.ihmc.gdx.simulation.environment;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.internal.ImGui;
import us.ihmc.gdx.imgui.ImGui3DViewInput;
import us.ihmc.gdx.imgui.ImGuiTools;
import us.ihmc.gdx.simulation.environment.object.objects.GDXMediumCinderBlockRoughed;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.graphics.GDXPose3DWidget;

public class GDXEnvironment implements RenderableProvider
{
   private final static String WINDOW_NAME = ImGuiTools.uniqueLabel(GDXEnvironment.class, "Environment");
   private GDXMediumCinderBlockRoughed cinderBlock;
   private final GDXPose3DWidget pose3DWidget = new GDXPose3DWidget();

   public void create(GDXImGuiBasedUI baseUI)
   {
      cinderBlock = new GDXMediumCinderBlockRoughed();
      baseUI.getSceneManager().addRenderableProvider(this);

      pose3DWidget.create(baseUI);
      baseUI.addImGui3DViewInputProcessor(pose3DWidget::process3DViewInput);
   }

   private void process3DViewInput(ImGui3DViewInput viewInput)
   {

   }

   public void render()
   {
      ImGui.begin(WINDOW_NAME);

      pose3DWidget.render();

      ImGui.end();

      GDXTools.toGDX(pose3DWidget.getTransform(), cinderBlock.getModelInstance().transform);
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      cinderBlock.getModelInstance().getRenderables(renderables, pool);
      pose3DWidget.getRenderables(renderables, pool);
   }

   public String getWindowName()
   {
      return WINDOW_NAME;
   }
}
