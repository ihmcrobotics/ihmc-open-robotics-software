package us.ihmc.gdx;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3Graphics;
import com.badlogic.gdx.graphics.GL20;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import imgui.ImGui;
import imgui.ImGuiIO;
import imgui.ImGuiViewport;
import imgui.gl3.ImGuiImplGl3;
import imgui.glfw.ImGuiImplGlfw;
import org.lwjgl.glfw.GLFWErrorCallback;
import us.ihmc.log.LogTools;

import static org.lwjgl.glfw.GLFW.*;

public class GDX3DWith2DImGuiDemo
{
   private ModelInstance boxes;
   private ModelInstance coordinateFrame;


   public GDX3DWith2DImGuiDemo()
   {
      GDXApplicationCreator.launchGDXApplication(new PrivateGDXApplication(), "GDX3DDemo", 1100, 800);
   }

   class PrivateGDXApplication extends GDX3DApplication
   {
      private final ImGuiImplGlfw imGuiGlfw = new ImGuiImplGlfw();
      private final ImGuiImplGl3 imGuiGl3 = new ImGuiImplGl3();
      private String glslVersion;
      private long windowHandle;

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

         Gdx.gl.glClearColor(0.5019608f, 0.5019608f, 0.5019608f, 1.0f);
         Gdx.gl.glClear(GL20.GL_COLOR_BUFFER_BIT | GL20.GL_DEPTH_BUFFER_BIT);

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

         ImGui.begin("3D View");

         ImGui.setWindowSize(800, 400);

//         ImGui.checkbox("Check it", false);

         float posX = ImGui.getWindowPosX();
         float posY = ImGui.getWindowPosY();
         float sizeX = ImGui.getWindowSizeX();
         float sizeY = ImGui.getWindowSizeY();

         setViewportBounds((int) posX, getCurrentWindowHeight() - (int) posY - (int) sizeY, (int) sizeX, (int) sizeY);

         //         float posX = ImGui.getMainViewport().getPosX();
         //         float posY = ImGui.getMainViewport().getPosY();
         //         float sizeX = ImGui.getMainViewport().getSizeX();
         //         float sizeY = ImGui.getMainViewport().getSizeY();
         LogTools.info(posX + posY +sizeX + sizeY);

         ImGui.end();


//         glClearColor(exampleUi.backgroundColor[0], exampleUi.backgroundColor[1], exampleUi.backgroundColor[2], 0.0f);
//         glClear(GL_COLOR_BUFFER_BIT);


         ImGui.render();
         imGuiGl3.renderDrawData(ImGui.getDrawData());

         renderBefore();

         getModelBatch().render(coordinateFrame, getEnvironment());
         getModelBatch().render(boxes, getEnvironment());

         renderAfter();
//         float posX = ImGui.getMainViewport().getDrawData().getDisplayPosX();
//         float posY = ImGui.getMainViewport().getDrawData().getDisplayPosY();
//         float sizeX = ImGui.getMainViewport().getDrawData().getDisplaySizeX();
//         float sizeY = ImGui.getMainViewport().getDrawData().getDisplaySizeY();
//
////         float posX = ImGui.getMainViewport().getPosX();
////         float posY = ImGui.getMainViewport().getPosY();
////         float sizeX = ImGui.getMainViewport().getSizeX();
////         float sizeY = ImGui.getMainViewport().getSizeY();
//         LogTools.info(posX + posY +sizeX + sizeY);

         glfwSwapBuffers(windowHandle);
         glfwPollEvents();
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
      new GDX3DWith2DImGuiDemo();
   }
}
