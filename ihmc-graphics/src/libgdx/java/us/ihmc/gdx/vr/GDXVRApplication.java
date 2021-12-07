package us.ihmc.gdx.vr;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3Files;
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3NativesLoader;
import org.lwjgl.glfw.GLFW;
import org.lwjgl.glfw.GLFWErrorCallback;
import org.lwjgl.opengl.GL;
import org.lwjgl.system.MemoryUtil;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.sceneManager.GDX3DSceneBasics;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.vr.minimalGDX.MinimalGDXApplication;
import us.ihmc.gdx.vr.minimalGDX.MinimalGDXGraphics;
import us.ihmc.gdx.vr.minimalGDX.MinimalLwjgl3GL30;

/**
 * This class provides a VR application with an offscreen GLFW window that is not rendered to.
 * This allows VR to run as fast as possible without extra stuff going on.
 */
public class GDXVRApplication
{
   private static GLFWErrorCallback errorCallback;
   private long windowHandle;
   private volatile boolean running = true;
   private GDX3DSceneBasics sceneBasics;
   private GDXVRContext vrContext;

   public void launch(Lwjgl3ApplicationAdapter applicationAdapter)
   {
      Lwjgl3NativesLoader.load();

      errorCallback = GLFWErrorCallback.createPrint(System.err);
      GLFW.glfwSetErrorCallback(errorCallback);
      if (!GLFW.glfwInit())
      {
         throw new RuntimeException("Unable to initialize GLFW");
      }

      GLFW.glfwWindowHint(GLFW.GLFW_VISIBLE, GLFW.GLFW_FALSE);
      windowHandle = GLFW.glfwCreateWindow(640, 480, "", MemoryUtil.NULL, MemoryUtil.NULL);
      GLFW.glfwMakeContextCurrent(windowHandle);

      GL.createCapabilities();
      GDXTools.printGLVersion();

      Gdx.gl30 = new MinimalLwjgl3GL30();
      Gdx.gl = Gdx.gl30;
      Gdx.gl20 = Gdx.gl30;
      Gdx.files = new Lwjgl3Files();
      Gdx.graphics = new MinimalGDXGraphics();
      MinimalGDXApplication app = new MinimalGDXApplication();
      Gdx.app = app;

      sceneBasics = new GDX3DSceneBasics();
      sceneBasics.create();

      vrContext = new GDXVRContext();
      vrContext.initSystem();
      vrContext.setupEyes();

      applicationAdapter.create();

      while (running)
      {
         vrContext.waitGetPoses();
         vrContext.pollEvents();
         while (!app.getPostRunnables().isEmpty())
         {
            app.getPostRunnables().pollFirst().run();
         }
         applicationAdapter.render();
         vrContext.renderEyes(sceneBasics);
      }

      applicationAdapter.dispose();

      GLFW.glfwDestroyWindow(windowHandle);
      errorCallback.free();
      errorCallback = null;
      GLFW.glfwTerminate();
   }

   public GDX3DSceneBasics getSceneBasics()
   {
      return sceneBasics;
   }

   public GDXVRContext getVRContext()
   {
      return vrContext;
   }

   public void exit()
   {
      running = false;
   }
}
