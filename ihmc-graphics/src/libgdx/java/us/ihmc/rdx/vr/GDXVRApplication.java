package us.ihmc.rdx.vr;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3Files;
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3NativesLoader;
import org.lwjgl.glfw.GLFW;
import org.lwjgl.glfw.GLFWErrorCallback;
import org.lwjgl.opengl.GL;
import org.lwjgl.system.MemoryUtil;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.sceneManager.GDX3DScene;
import us.ihmc.rdx.tools.GDXTools;
import us.ihmc.rdx.vr.minimalGDX.MinimalGDXApplication;
import us.ihmc.rdx.vr.minimalGDX.MinimalGDXGraphics;
import us.ihmc.rdx.vr.minimalGDX.MinimalLwjgl3GL30;

/**
 * This class provides a VR application with an offscreen GLFW window that is not rendered to.
 * This allows VR to run as fast as possible without extra stuff going on.
 */
public class GDXVRApplication
{
   private static GLFWErrorCallback errorCallback;
   private long windowHandle;
   private volatile boolean running = true;
   private GDX3DScene scene;
   private GDXVRContext vrContext;
   private int renderNumber = 0;
   private Runnable thingToRunAfter10Frames;

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

      scene = new GDX3DScene();
      scene.create();

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
         if (renderNumber < 10)
         {
            renderNumber++;
            if (renderNumber == 10)
            {
               if (thingToRunAfter10Frames != null)
               {
                  thingToRunAfter10Frames.run();
               }
            }
         }
         applicationAdapter.render();
         vrContext.renderEyes(scene);
      }

      applicationAdapter.dispose();

      GLFW.glfwDestroyWindow(windowHandle);
      errorCallback.free();
      errorCallback = null;
      GLFW.glfwTerminate();
   }

   public void runAfter10Frames(Runnable thingToRunAfter10Frames)
   {
      this.thingToRunAfter10Frames = thingToRunAfter10Frames;
   }

   public GDX3DScene getScene()
   {
      return scene;
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
