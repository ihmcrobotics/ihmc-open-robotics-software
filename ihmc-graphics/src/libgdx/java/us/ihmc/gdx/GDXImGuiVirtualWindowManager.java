package us.ihmc.gdx;

import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.VertexAttributes;
import com.badlogic.gdx.graphics.g3d.*;
import com.badlogic.gdx.graphics.g3d.attributes.BlendingAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.TextureAttribute;
import com.badlogic.gdx.graphics.g3d.utils.ModelBuilder;
import com.badlogic.gdx.graphics.glutils.FrameBuffer;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.*;
import imgui.flag.ImGuiCol;
import imgui.flag.ImGuiConfigFlags;
import imgui.gl3.ImGuiImplGl3;
import imgui.glfw.ImGuiImplGlfw;
import imgui.internal.ImGuiContext;
import org.lwjgl.opengl.GL30;
import us.ihmc.gdx.imgui.ImGuiTools;

import java.util.ArrayList;

import static org.lwjgl.glfw.GLFW.*;

public class GDXImGuiVirtualWindowManager implements RenderableProvider
{
   private static GDXImGuiVirtualWindowManager instance = null;

   public static GDXImGuiVirtualWindowManager getInstance() {
      if (instance == null)
         instance = new GDXImGuiVirtualWindowManager();

      return instance;
   }

   private final ArrayList<GDXImGuiWindow> panels = new ArrayList<>();
   private ImFont font = null;

   private final ModelBuilder BUILDER = new ModelBuilder();
   private ModelInstance modelInstance = null;

   private final ImGuiImplGlfw imGuiGlfw = new ImGuiImplGlfw();
   private final ImGuiImplGl3 imGuiGl3 = new ImGuiImplGl3();

   private final long virtualWindowContext;
   private ImGuiContext virtualImGuiContext = null;
   private long savedGLFWContext = 0;
   private ImGuiContext savedImGuiContext = null;

   private void backupAndSwitchContext() {
      savedImGuiContext = ImGui.getCurrentContext();
      if (virtualImGuiContext == null) {
         virtualImGuiContext = ImGui.createContext();

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

         io.setDisplaySize(3000, 2000);
      }
      ImGui.setCurrentContext(virtualImGuiContext);

      savedGLFWContext = glfwGetCurrentContext();
      glfwMakeContextCurrent(virtualWindowContext);
   }

   private void restoreSavedContext() {
      glfwMakeContextCurrent(savedGLFWContext);

      if (savedImGuiContext != null && savedImGuiContext.ptr != 0)
         ImGui.setCurrentContext(savedImGuiContext);
   }

   private GDXImGuiVirtualWindowManager() {
      if (!glfwInit())
      {
         throw new IllegalStateException("Unable to initialize GLFW");
      }

      this.savedGLFWContext = glfwGetCurrentContext();

      glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
      this.virtualWindowContext = glfwCreateWindow(3000, 2000, "", 0, 0); //0 is NULL?

      restoreSavedContext();
   }

   public void create() {
      backupAndSwitchContext(); //This is also where the ImGuiContext is created

      imGuiGlfw.init(virtualWindowContext, true);
      imGuiGl3.init();

      restoreSavedContext();
   }

   public void update() {
      backupAndSwitchContext();

      ImGui.newFrame();

      for (GDXImGuiWindow panel : panels) {
         if (ImGui.begin(panel.getName())) {
            panel.renderImGuiWidgets();
            ImGui.end();
         }
      }

      ImGui.render();
      imGuiGl3.renderDrawData(ImGui.getDrawData());

      restoreSavedContext();
   }

   public void addPanel(GDXImGuiWindow panel) {
      panels.add(panel);
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      if (modelInstance != null)
         modelInstance.getRenderables(renderables, pool);
   }

   public void dispose() {
      for (GDXImGuiWindow panel : panels)
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
