package us.ihmc.gdx;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3Application;
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3ApplicationConfiguration;
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3Graphics;
import com.badlogic.gdx.graphics.GL20;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import imgui.*;
import imgui.internal.ImGui;
import imgui.flag.*;
import imgui.gl3.ImGuiImplGl3;
import imgui.glfw.ImGuiImplGlfw;
import imgui.internal.ImGuiDockNode;
import imgui.type.ImInt;
import org.lwjgl.glfw.GLFWErrorCallback;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.log.LogTools;
import us.ihmc.tools.string.StringTools;

import static org.lwjgl.glfw.GLFW.*;

public class GDX3DFullImGuiDemo
{
   private ModelInstance boxes;
   private ModelInstance coordinateFrame;

   private boolean isInitialized = false;

   public GDX3DFullImGuiDemo()
   {
//      GDXApplicationCreator.launchGDXApplication(new PrivateGDXApplication(), "GDX3DDemo", 1100, 800);

      Lwjgl3ApplicationConfiguration applicationConfiguration = new Lwjgl3ApplicationConfiguration();
      applicationConfiguration.setTitle(getClass().getSimpleName());
      applicationConfiguration.setWindowedMode(1100, 800);
      applicationConfiguration.useVsync(true);
      applicationConfiguration.setBackBufferConfig(8, 8, 8, 8, 16, 0, 4);

      ThreadTools.startAThread(() -> new Lwjgl3Application(new PrivateGDXApplication(), applicationConfiguration), getClass().getSimpleName());
   }

   class PrivateGDXApplication extends GDX3DApplication
   {
      private final ImGuiImplGlfw imGuiGlfw = new ImGuiImplGlfw();
      private final ImGuiImplGl3 imGuiGl3 = new ImGuiImplGl3();
      private String glslVersion;
      private long windowHandle;

      private final Stopwatch stopwatch = new Stopwatch().start();
      private ImFont imFont;

      @Override
      public void create()
      {
         super.create();

         coordinateFrame = new ModelInstance(GDXModelPrimitives.createCoordinateFrame(0.3));
         boxes = new BoxesDemoModel().newInstance();

         GLFWErrorCallback.createPrint(System.err).set();

         if (!glfwInit())
         {
            throw new IllegalStateException("Unable to initialize GLFW");
         }
//         glfwDefaultWindowHints();
//         if (SystemUtils.IS_OS_MAC) {
//            glslVersion = "#version 150";
//            glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
//            glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
//            glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
//            glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // Required on Mac
//         } else {
//            glslVersion = "#version 130";
//            glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
//            glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
//         }

//         GL.createCapabilities();

         ImGui.createContext();

         final ImGuiIO io = ImGui.getIO();
         io.setIniFilename(null); // We don't want to save .ini file
//         io.addConfigFlags(ImGuiConfigFlags.NavEnableKeyboard);
         io.addConfigFlags(ImGuiConfigFlags.DockingEnable);
//         io.addConfigFlags(ImGuiConfigFlags.ViewportsEnable);
         io.setConfigViewportsNoTaskBarIcon(true);
         io.setConfigWindowsMoveFromTitleBarOnly(true);

         ImGui.styleColorsLight();
         imFont = ImGuiTools.setupFonts(io);

         // When viewports are enabled we tweak WindowRounding/WindowBg so platform windows can look identical to regular ones.
         if (io.hasConfigFlags(ImGuiConfigFlags.ViewportsEnable))
         {
            final ImGuiStyle style = imgui.ImGui.getStyle();
            style.setWindowRounding(0.0f);
            style.setColor(ImGuiCol.WindowBg, imgui.ImGui.getColorU32(ImGuiCol.WindowBg, 1));
         }

         windowHandle = ((Lwjgl3Graphics) Gdx.graphics).getWindow().getWindowHandle();

         imGuiGlfw.init(windowHandle, true);
         imGuiGl3.init(glslVersion);
      }

      @Override
      public void render()
      {

         Gdx.gl.glClearColor(0, 0, 0, 1.0f);
//         Gdx.gl.glClearColor(0.5019608f, 0.5019608f, 0.5019608f, 1.0f);
         Gdx.gl.glClear(GL20.GL_COLOR_BUFFER_BIT | GL20.GL_DEPTH_BUFFER_BIT);

         imGuiGlfw.newFrame();
         ImGui.newFrame();

         ImGui.pushFont(imFont);

//         ImGui.setNextWindowBgAlpha(0.0f);
         int flags = ImGuiDockNodeFlags.None;
         flags += ImGuiDockNodeFlags.PassthruCentralNode;
         int dockspaceId = ImGui.dockSpaceOverViewport(ImGui.getMainViewport(), flags);


//         if (ImGui.begin(""))

         //         ImGui.setNextWindowDockID(dockspaceId, ImGuiCond.FirstUseEver);

         ImGui.pushStyleColor(ImGuiCol.WindowBg, 0f, 0f, 0f, 0f);
//         ImGui.setNextWindowBgAlpha(0.0f);
         flags = ImGuiWindowFlags.None;
//         flags += ImGuiWindowFlags.NoNavInputs;
//         flags += ImGuiWindowFlags.NoMouseInputs;
         ImGui.begin("3D View", flags);

         ImGui.popStyleColor();

         float posX = ImGui.getWindowPosX();
         float posY = ImGui.getWindowPosY();
         float sizeX = ImGui.getWindowSizeX();
         float sizeY = ImGui.getWindowSizeY();
         float itemRectMinX = ImGui.getItemRectMinX();
         float itemRectMinY = ImGui.getItemRectMinY();
         float itemRectMaxX = ImGui.getItemRectMaxX();
         float itemRectMaxY = ImGui.getItemRectMaxY();
         setViewportBounds((int) posX, getCurrentWindowHeight() - (int) posY - (int) sizeY, (int) sizeX, (int) sizeY);

//         ImGui.getWindowDrawList().addRectFilled(itemRectMinX, itemRectMinY, itemRectMaxX, itemRectMaxY, ImColor.floatToColor(0.5019608f, 0.5019608f, 0.5019608f, 1.0f));
         ImGui.getWindowDrawList().addRectFilled(posX, posY, posX + sizeX, posY + sizeY, ImColor.floatToColor(0.5019608f, 0.5019608f, 0.5019608f, 1.0f));

         ImGui.end();

         getCamera3D().clearInputExclusionBoxes();

//         ImGui.setNextWindowDockID(dockspaceId, ImGuiCond.FirstUseEver);
//         ImGui.setNextWindowBgAlpha(0.0f);

         ImGui.begin("Window");

         if (ImGui.beginTabBar("main"))
         {
            if (ImGui.beginTabItem("Window"))
            {
               ImGui.text("Tab bar detected!");
               ImGui.endTabItem();
            }
            ImGui.endTabBar();
         }

         ImGui.text(StringTools.format3D("Time: {} s", stopwatch.totalElapsed()).get());
         ImGui.button("I'm a Button!");


//         if (ImGui.beginTabItem("Window"))
//         {
//            ImGui.button("I'm a tab Button!");
//            ImGui.endTabItem();
//         }

         float[] values = new float[100];
         for (int i = 0; i < 100; i++)
         {
            values[i] = i;
         }

         ImGui.plotLines("Histogram", values, 100);

         getCamera3D().addInputExclusionBox(ImGuiTools.windowBoundingBox());

         ImGui.end();

         if (!isInitialized)
         {
            ImGui.dockBuilderRemoveNode(dockspaceId);
            flags = imgui.internal.flag.ImGuiDockNodeFlags.None;
            flags += imgui.internal.flag.ImGuiDockNodeFlags.PassthruCentralNode;
            flags += imgui.internal.flag.ImGuiDockNodeFlags.DockSpace;
            ImGui.dockBuilderAddNode(dockspaceId, flags);
            ImGui.dockBuilderSetNodeSize(dockspaceId, getCurrentWindowWidth(), getCurrentWindowHeight());
            ImInt outIdAtOppositeDir = new ImInt();
            int dockRightId = ImGui.dockBuilderSplitNode(dockspaceId, ImGuiDir.Right, 0.20f, null, outIdAtOppositeDir);
            ImGui.dockBuilderDockWindow("3D View", dockspaceId);
            ImGui.dockBuilderDockWindow("Window", dockRightId);
            ImGui.dockBuilderFinish(dockspaceId);
         }




         ImGui.popFont();

         ImGui.render();
         imGuiGl3.renderDrawData(ImGui.getDrawData());

//         ImGui.updatePlatformWindows();

         renderBefore();

         getModelBatch().render(coordinateFrame, getEnvironment());
         getModelBatch().render(boxes, getEnvironment());

         renderAfter();

         glfwSwapBuffers(windowHandle);
         glfwPollEvents();

         isInitialized = true;
      }

      @Override
      public void dispose()
      {
         super.dispose();
         imGuiGl3.dispose();
         imGuiGlfw.dispose();

         ImGui.destroyContext();
      }
   }

   public static void main(String[] args)
   {
      new GDX3DFullImGuiDemo();
   }
}
