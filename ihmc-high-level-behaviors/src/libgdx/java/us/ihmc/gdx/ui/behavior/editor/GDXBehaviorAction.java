package us.ihmc.gdx.ui.behavior.editor;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.node.ObjectNode;
import imgui.type.ImBoolean;
import us.ihmc.gdx.input.ImGui3DViewInput;

public interface GDXBehaviorAction
{
   public void update();

   public void calculate3DViewPick(ImGui3DViewInput input);

   public void process3DViewInput(ImGui3DViewInput input);

   public void renderImGuiWidgets();

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool);

   public void saveToFile(ObjectNode jsonNode);

   public void loadFromFile(JsonNode jsonNode);

   public void performAction();

   public void destroy();

   public ImBoolean getSelected();

   public String getNameForDisplay();
}
