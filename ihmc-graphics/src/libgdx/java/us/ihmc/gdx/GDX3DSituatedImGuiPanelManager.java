package us.ihmc.gdx;

import com.badlogic.gdx.graphics.g3d.*;
import com.badlogic.gdx.graphics.g3d.utils.ModelBuilder;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.*;
import imgui.flag.ImGuiCol;
import imgui.flag.ImGuiConfigFlags;
import imgui.gl3.ImGuiImplGl3;
import imgui.glfw.ImGuiImplGlfw;
import imgui.internal.ImGuiContext;
import org.lwjgl.system.MemoryUtil;
import us.ihmc.gdx.imgui.ImGuiTools;

import java.util.ArrayList;

import static org.lwjgl.glfw.GLFW.*;

public class GDX3DSituatedImGuiPanelManager implements RenderableProvider
{
   private static GDX3DSituatedImGuiPanelManager instance = null;

   public static GDX3DSituatedImGuiPanelManager getInstance()
   {
      if (instance == null)
         instance = new GDX3DSituatedImGuiPanelManager();

      return instance;
   }

   private final ArrayList<GDX3DSituatedImGuiPanel> panels = new ArrayList<>();
   private ImFont font = null;

   private final ModelBuilder BUILDER = new ModelBuilder();
   private ModelInstance modelInstance = null;

   private final ImGuiImplGlfw imGuiGlfw = new ImGuiImplGlfw();
   private final ImGuiImplGl3 imGuiGl3 = new ImGuiImplGl3();

   private final long virtualWindowContext;
   private ImGuiContext virtualImGuiContext = null;
   private long savedGLFWContext = 0;
   private ImGuiContext savedImGuiContext = null;

   private void backupAndSwitchContext()
   {
      savedGLFWContext = glfwGetCurrentContext();
      glfwMakeContextCurrent(virtualWindowContext);

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
            final ImGuiStyle style = imgui.ImGui.getStyle();
            style.setWindowRounding(0.0f);
            style.setColor(ImGuiCol.WindowBg, imgui.ImGui.getColorU32(ImGuiCol.WindowBg, 1));
         }

         int[] fbWidth = new int[1];
         int[] fbHeight = new int[1];
         glfwGetFramebufferSize(virtualWindowContext, fbWidth, fbHeight);

         io.setDisplaySize(fbWidth[0], fbHeight[0]);
      }
      else
      {
         ImGui.setCurrentContext(virtualImGuiContext);
      }
   }

   private void restoreSavedContext()
   {
      glfwMakeContextCurrent(savedGLFWContext);

      if (savedImGuiContext != null && savedImGuiContext.ptr != 0)
         ImGui.setCurrentContext(savedImGuiContext);
   }

   private GDX3DSituatedImGuiPanelManager()
   {
      this.savedGLFWContext = glfwGetCurrentContext();

      glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
      this.virtualWindowContext = glfwCreateWindow(3000, 2000, "", MemoryUtil.NULL, MemoryUtil.NULL);
   }

   public void create()
   {
      backupAndSwitchContext(); //This is also where the ImGuiContext is created

      if (!glfwInit()) //Probably already initialized by this point, which immediately returns true
      {
         throw new IllegalStateException("Unable to initialize GLFW");
      }

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
         if (ImGui.begin(panel.getName()))
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
      for (GDX3DSituatedImGuiPanel panel : panels)
         panel.dispose();

      imGuiGl3.dispose();
      imGuiGlfw.dispose();
   }

   @Override
   protected void finalize() throws Throwable
   {
      this.dispose();
      super.finalize();
   }
}
