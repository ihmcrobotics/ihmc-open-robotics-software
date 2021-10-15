package us.ihmc.gdx;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3NativesLoader;
import org.lwjgl.glfw.GLFW;
import org.lwjgl.glfw.GLFWErrorCallback;
import org.lwjgl.opengl.GL;
import org.lwjgl.system.MemoryUtil;
import us.ihmc.gdx.sceneManager.GDX3DSceneBasics;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.vr.GDXVRContext;
import us.ihmc.gdx.vr.GDXVRControllerButtons;
import us.ihmc.robotics.robotSide.RobotSide;

public class GDXVROnlyDemo
{
   private static GLFWErrorCallback errorCallback;
   private final GDXVRContext vrContext;
   private final GDX3DSceneBasics sceneBasics;
   private final long windowHandle;
   private volatile boolean running = true;

   public GDXVROnlyDemo()
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

      Gdx.gl30 = new Lwjgl3GL30ForMinimalVRDemo();
      Gdx.gl = Gdx.gl30;
      Gdx.gl20 = Gdx.gl30;

      sceneBasics = new GDX3DSceneBasics();
      sceneBasics.addCoordinateFrame(1.0);

      vrContext = new GDXVRContext();
      vrContext.initSystem();
      vrContext.setupEyes();

      vrContext.addVRInputProcessor(vrContext ->
      {
         vrContext.getController(RobotSide.RIGHT, controller ->
         {
            if (controller.isButtonNewlyPressed(GDXVRControllerButtons.A))
            {
               running = false;
            }
         });
      });

      while (running)
      {
         vrContext.waitGetPoses();
         vrContext.pollEvents();
         vrContext.renderEyes(sceneBasics);
      }

      GLFW.glfwDestroyWindow(windowHandle);
      errorCallback.free();
      errorCallback = null;
      GLFW.glfwTerminate();
   }

   public static void main(String[] args)
   {
      new GDXVROnlyDemo();
   }
}
