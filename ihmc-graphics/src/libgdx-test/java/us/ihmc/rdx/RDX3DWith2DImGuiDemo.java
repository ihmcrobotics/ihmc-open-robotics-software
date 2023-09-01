package us.ihmc.rdx;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3Graphics;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import imgui.ImGui;
import imgui.ImGuiIO;
import imgui.gl3.ImGuiImplGl3;
import imgui.glfw.ImGuiImplGlfw;
import org.lwjgl.glfw.GLFWErrorCallback;
import org.lwjgl.opengl.GL41;
import us.ihmc.rdx.imgui.ImGuiTools;
import us.ihmc.rdx.sceneManager.RDX3DBareBonesScene;
import us.ihmc.rdx.tools.BoxesDemoModel;
import us.ihmc.rdx.tools.LibGDXApplicationCreator;
import us.ihmc.rdx.tools.RDXModelBuilder;

import static org.lwjgl.glfw.GLFW.*;

public class RDX3DWith2DImGuiDemo
{
   private final ImGuiImplGlfw imGuiGlfw = new ImGuiImplGlfw();
   private final ImGuiImplGl3 imGuiGl3 = new ImGuiImplGl3();
   private String glslVersion;
   private long windowHandle;

   public RDX3DWith2DImGuiDemo()
   {
      RDX3DBareBonesScene sceneManager = new RDX3DBareBonesScene();
      LibGDXApplicationCreator.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            sceneManager.create();

            sceneManager.addModelInstance(new ModelInstance(RDXModelBuilder.createCoordinateFrame(0.3)));
            sceneManager.addModelInstance(new BoxesDemoModel().newInstance());

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
            //         io.addConfigFlags(ImGuiConfigFlags.NavEnableKeyboard);  // Enable Keyboard Controls
            //         io.addConfigFlags(ImGuiConfigFlags.DockingEnable);      // Enable Docking
            //         io.addConfigFlags(ImGuiConfigFlags.ViewportsEnable);    // Enable Multi-Viewport / Platform Windows
            //         io.setConfigViewportsNoTaskBarIcon(true);

            ImGuiTools.setupFonts(io);

            windowHandle = ((Lwjgl3Graphics) Gdx.graphics).getWindow().getWindowHandle();

            imGuiGlfw.init(windowHandle, true);
            imGuiGl3.init(glslVersion);
         }

         @Override
         public void render()
         {
            GL41.glClearColor(0.5019608f, 0.5019608f, 0.5019608f, 1.0f);
            GL41.glClear(GL41.GL_COLOR_BUFFER_BIT | GL41.GL_DEPTH_BUFFER_BIT);

            sceneManager.render();

            imGuiGlfw.newFrame();
            ImGui.newFrame();

            ImGui.begin("Window");
            ImGui.button("I'm a Button!");

            float[] values = new float[100];
            for (int i = 0; i < 100; i++)
            {
               values[i] = i;
            }

            ImGui.plotLines("Histogram", values, 100);

            ImGui.end();

            ImGui.render();
            imGuiGl3.renderDrawData(ImGui.getDrawData());

            glfwSwapBuffers(windowHandle);
            glfwPollEvents();
         }

         @Override
         public void dispose()
         {
            sceneManager.dispose();
            imGuiGl3.dispose();
            imGuiGlfw.dispose();

            ImGui.destroyContext();
         }
      }, getClass().getSimpleName(), 1100, 800);
   }

   public static void main(String[] args)
   {
      new RDX3DWith2DImGuiDemo();
   }
}
