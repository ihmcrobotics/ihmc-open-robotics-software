package us.ihmc.gdx;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3Graphics;
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3NativesLoader;
import org.lwjgl.opengl.GL;
import us.ihmc.gdx.sceneManager.GDX3DSceneBasics;
import us.ihmc.gdx.vr.GDXVRContext;
import us.ihmc.gdx.vr.GDXVRControllerButtons;
import us.ihmc.robotics.robotSide.RobotSide;

public class GDXVROnlyDemo
{
   private final GDXVRContext vrContext;
   private final GDX3DSceneBasics sceneBasics;
   private volatile boolean running = true;

   public GDXVROnlyDemo()
   {
      Lwjgl3NativesLoader.load();
//      GL.createCapabilities();

//      Gdx.graphics = new Lwjgl3Graphics(this);
//      Gdx.gl30 = Gdx.graphics.getGL30();
//      Gdx.gl20 = Gdx.gl30 != null ? Gdx.gl30 : Gdx.graphics.getGL20();
//      Gdx.gl = Gdx.gl30 != null ? Gdx.gl30 : Gdx.gl20;


      // TODO: Create an OpenGL context without GLFW?

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
   }

   public static void main(String[] args)
   {
      new GDXVROnlyDemo();
   }
}
