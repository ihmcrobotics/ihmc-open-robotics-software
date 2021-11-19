package us.ihmc.gdx;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImFont;
import imgui.ImGui;
import imgui.ImGuiIO;
import imgui.ImGuiStyle;
import imgui.flag.ImGuiCol;
import imgui.flag.ImGuiConfigFlags;
import imgui.gl3.ImGuiImplGl3;
import imgui.glfw.ImGuiImplGlfw;
import imgui.internal.ImGuiContext;
import org.lwjgl.glfw.GLFW;
import org.lwjgl.system.MemoryUtil;
import us.ihmc.gdx.imgui.ImGuiTools;

import java.util.ArrayList;

public class GDX3DSituatedImGuiPanelManagerOld implements RenderableProvider
{
   private final ArrayList<GDX3DSituatedImGuiPanel> panels = new ArrayList<>();
   private ImFont font = null;

   private ModelInstance modelInstance = null;

   private final ImGuiImplGlfw imGuiGlfw = new ImGuiImplGlfw();
   private final ImGuiImplGl3 imGuiGl3 = new ImGuiImplGl3();

   private final long virtualWindowContext;
   private ImGuiContext virtualImGuiContext = null;
   private long savedGLFWContext = 0;
   private ImGuiContext savedImGuiContext = null;

   public GDX3DSituatedImGuiPanelManagerOld()
   {
      this.savedGLFWContext = GLFW.glfwGetCurrentContext();

      GLFW.glfwWindowHint(GLFW.GLFW_VISIBLE, GLFW.GLFW_FALSE);
      this.virtualWindowContext = GLFW.glfwCreateWindow(3000, 2000, "", MemoryUtil.NULL, MemoryUtil.NULL);
   }

   public void create()
   {
      backupAndSwitchContext(); //This is also where the ImGuiContext is created

      imGuiGlfw.init(virtualWindowContext, true);
      imGuiGl3.init();

      restoreSavedContext();
   }

   public void update()
   {
      backupAndSwitchContext();

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

      restoreSavedContext();
   }

   private void backupAndSwitchContext()
   {
      savedGLFWContext = GLFW.glfwGetCurrentContext();
      GLFW.glfwMakeContextCurrent(virtualWindowContext);

      savedImGuiContext = ImGui.getCurrentContext();
      if (virtualImGuiContext == null)
      {
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
            final ImGuiStyle style = ImGui.getStyle();
            style.setWindowRounding(0.0f);
            style.setColor(ImGuiCol.WindowBg, ImGui.getColorU32(ImGuiCol.WindowBg, 1));
         }

         int[] fbWidth = new int[1];
         int[] fbHeight = new int[1];
         GLFW.glfwGetFramebufferSize(virtualWindowContext, fbWidth, fbHeight);

         io.setDisplaySize(fbWidth[0], fbHeight[0]);
      }
      else
      {
         ImGui.setCurrentContext(virtualImGuiContext);
      }
   }

   private void restoreSavedContext()
   {
      GLFW.glfwMakeContextCurrent(savedGLFWContext);

      if (savedImGuiContext != null && savedImGuiContext.ptr != 0)
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
