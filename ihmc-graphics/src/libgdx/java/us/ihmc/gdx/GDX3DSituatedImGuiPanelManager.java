package us.ihmc.gdx;

import com.badlogic.gdx.graphics.g3d.*;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.*;
import imgui.flag.ImGuiCol;
import imgui.flag.ImGuiConfigFlags;
import imgui.gl3.ImGuiImplGl3;
import imgui.glfw.ImGuiImplGlfw;
import imgui.internal.ImGuiContext;
import us.ihmc.gdx.imgui.ImGuiTools;

import java.util.ArrayList;

public class GDX3DSituatedImGuiPanelManager implements RenderableProvider
{
   private final ArrayList<GDX3DSituatedImGuiPanel> panels = new ArrayList<>();
   private ImFont font = null;
   private ModelInstance modelInstance = null;
   private ImGuiImplGlfw imGuiGlfw;
   private ImGuiImplGl3 imGuiGl3;
   private ImGuiContext savedImGuiContext = null;
   private ImGuiContext virtualImGuiContext = null;
   private int width = 800;
   private int height = 600;

   public void create(ImGuiImplGlfw imGuiGlfw, ImGuiImplGl3 imGuiGl3)
   {
      this.imGuiGlfw = imGuiGlfw;
      this.imGuiGl3 = imGuiGl3;

      savedImGuiContext = ImGui.getCurrentContext();
      virtualImGuiContext = ImGui.createContext();
      ImGui.setCurrentContext(virtualImGuiContext);

      final ImGuiIO io = ImGui.getIO();
      io.setIniFilename(null); // We don't want to save .ini file
      io.addConfigFlags(ImGuiConfigFlags.DockingEnable);
      io.addConfigFlags(ImGuiConfigFlags.ViewportsEnable);
      io.setConfigViewportsNoTaskBarIcon(true);
      io.setConfigWindowsMoveFromTitleBarOnly(true);
      io.setConfigViewportsNoDecoration(false);
      io.setConfigDockingTransparentPayload(false);

      ImGui.styleColorsLight();

      font = ImGuiTools.setupFonts(io);

      if (io.hasConfigFlags(ImGuiConfigFlags.ViewportsEnable))
      {
         final ImGuiStyle style = imgui.ImGui.getStyle();
         style.setWindowRounding(0.0f);
         style.setColor(ImGuiCol.WindowBg, imgui.ImGui.getColorU32(ImGuiCol.WindowBg, 1));
      }

      io.setDisplaySize(width, height);

      ImGui.setCurrentContext(savedImGuiContext);
   }

   public void update()
   {
      savedImGuiContext = ImGui.getCurrentContext();
      ImGui.setCurrentContext(virtualImGuiContext);

      ImGui.newFrame();
      ImGui.pushFont(font);

      for (GDX3DSituatedImGuiPanel panel : panels)
      {
         if (ImGui.begin(panel.getPanelName()))
         {
            panel.renderImGuiWidgets();
            ImGui.end();
         }
      }

      ImGui.popFont();
      ImGui.render();
      imGuiGl3.renderDrawData(ImGui.getDrawData());

      ImGui.setCurrentContext(savedImGuiContext);
   }

   public void addPanel(GDX3DSituatedImGuiPanel panel)
   {
      panels.add(panel);
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (modelInstance != null)
         modelInstance.getRenderables(renderables, pool);
   }

   public void dispose()
   {
      imGuiGl3.dispose();
      imGuiGlfw.dispose();
   }
}
