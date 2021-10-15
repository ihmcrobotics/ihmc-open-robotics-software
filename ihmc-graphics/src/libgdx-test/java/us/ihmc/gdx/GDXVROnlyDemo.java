package us.ihmc.gdx;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3Files;
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3NativesLoader;
import org.lwjgl.glfw.GLFW;
import org.lwjgl.glfw.GLFWErrorCallback;
import org.lwjgl.opengl.GL;
import org.lwjgl.system.MemoryUtil;
import us.ihmc.gdx.sceneManager.GDX3DSceneBasics;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
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
      Gdx.files = new Lwjgl3Files();
      Gdx.graphics = new MinimalGDXGraphics();
      Gdx.app = new MinimalGDXApplication();

      sceneBasics = new GDX3DSceneBasics();
      sceneBasics.create();
      sceneBasics.addDefaultLighting();
      sceneBasics.addCoordinateFrame(1.0);

      vrContext = new GDXVRContext();
      vrContext.initSystem();
      vrContext.setupEyes();
      sceneBasics.addRenderableProvider(((renderables, pool) ->
      {
         for (RobotSide side : RobotSide.values)
         {
            vrContext.getController(side, controller -> controller.getModelInstance().getRenderables(renderables, pool));
            vrContext.getEyes().get(side).getCoordinateFrameInstance().getRenderables(renderables, pool);
         }
         vrContext.getBaseStations(baseStation -> baseStation.getModelInstance().getRenderables(renderables, pool));
         vrContext.getGenericDevices(genericDevice -> genericDevice.getModelInstance().getRenderables(renderables, pool));
      }), GDXSceneLevel.VIRTUAL);

      vrContext.addVRInputProcessor(vrContext ->
      {
         vrContext.getController(RobotSide.RIGHT, controller ->
         {
            if (controller.isButtonNewlyPressed(GDXVRControllerButtons.INDEX_A))
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
