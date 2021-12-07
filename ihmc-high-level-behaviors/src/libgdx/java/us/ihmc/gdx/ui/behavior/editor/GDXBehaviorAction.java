package us.ihmc.gdx.ui.behavior.editor;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.type.ImBoolean;
import us.ihmc.gdx.input.ImGui3DViewInput;

public interface GDXBehaviorAction
{
   public void process3DViewInput(ImGui3DViewInput input);

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool);

   public void destroy();

   public ImBoolean getSelected();

   public String getNameForDisplay();
}
