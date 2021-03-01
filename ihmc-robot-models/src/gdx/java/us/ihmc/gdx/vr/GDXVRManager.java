package us.ihmc.gdx.vr;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.gdx.sceneManager.GDX3DSceneManager;
import us.ihmc.gdx.sceneManager.GDX3DSceneTools;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.HashMap;
import java.util.HashSet;

public class GDXVRManager implements RenderableProvider
{
   private GDXVRContext context;
   private HashSet<ModelInstance> modelInstances = new HashSet<>();
   private ModelInstance headsetModelInstance;
   private SideDependentList<GDXVRContext.VRDevice> controllers = new SideDependentList<>();
   private boolean skipHeadset = false;

   public void create()
   {
      context = new GDXVRContext();

      // transform space to Z up, X forward
      YawPitchRoll yawPitchRoll = new YawPitchRoll();
      yawPitchRoll.setRoll(Math.toRadians(90.0));
      yawPitchRoll.setYaw(Math.toRadians(-90.0));
      RotationMatrix rotationMatrix = new RotationMatrix(yawPitchRoll);
      GDXTools.toGDX(rotationMatrix, context.getTrackerSpaceToWorldspaceRotationOffset());

      context.getEyeData(GDXVRContext.Eye.Left).camera.far = 100f;
      context.getEyeData(GDXVRContext.Eye.Right).camera.far = 100f;

      context.addListener(new GDXVRContext.VRDeviceListener()
      {
         @Override
         public void connected(GDXVRContext.VRDevice device)
         {
            LogTools.info("{} connected", device);
            if (device.getModelInstance() != null)
            {
               modelInstances.add(device.getModelInstance());

               if (device.getType() == GDXVRContext.VRDeviceType.HeadMountedDisplay)
               {
                  headsetModelInstance = device.getModelInstance();
               }
               else if(device.getType() == GDXVRContext.VRDeviceType.Controller)
               {
                  if (device.getControllerRole() == GDXVRContext.VRControllerRole.LeftHand)
                  {
                     controllers.set(RobotSide.LEFT, device);
                  }
                  else if (device.getControllerRole() == GDXVRContext.VRControllerRole.RightHand)
                  {
                     controllers.set(RobotSide.RIGHT, device);
                  }
                  else
                  {
                     if (!controllers.containsKey(RobotSide.LEFT))
                     {
                        controllers.set(RobotSide.LEFT, device);
                     }
                     else if (!controllers.containsKey(RobotSide.RIGHT))
                     {
                        controllers.set(RobotSide.RIGHT, device);
                     }
                  }
               }

               LogTools.info("{} controllers registered.", controllers.sides());
            }
         }

         @Override
         public void disconnected(GDXVRContext.VRDevice device)
         {
            LogTools.info("{} disconnected", device);
         }

         @Override
         public void buttonPressed(GDXVRContext.VRDevice device, int button)
         {
            LogTools.info("{} button pressed: {}", device, button);
         }

         @Override
         public void buttonReleased(GDXVRContext.VRDevice device, int button)
         {
            LogTools.info("{} button released: {}", device, button);
         }
      });

      context.addListener(new VRDeviceAdapter()
      {
         @Override
         public void buttonPressed(GDXVRContext.VRDevice device, int button)
         {
//            if (controllers.get(Robot))
         }
      });
   }

   public void pollEvents()
   {
      context.pollEvents();
   }

   public void render(GDX3DSceneManager sceneManager)
   {
      context.begin();
      renderScene(GDXVRContext.Eye.Left, sceneManager);
      renderScene(GDXVRContext.Eye.Right, sceneManager);
      context.end();
   }

   private void renderScene(GDXVRContext.Eye eye, GDX3DSceneManager sceneManager)
   {
      GDXVRCamera camera = context.getEyeData(eye).camera;

      context.beginEye(eye);

      int width = context.getEyeData(eye).getFrameBuffer().getWidth();
      int height = context.getEyeData(eye).getFrameBuffer().getHeight();
      Gdx.gl.glViewport(0, 0, width, height);

      GDX3DSceneTools.glClearGray();

      skipHeadset = true;
      sceneManager.renderToCamera(camera);
      skipHeadset = false;

      context.endEye();
   }

   public SideDependentList<GDXVRContext.VRDevice> getControllers()
   {
      return controllers;
   }

   public void dispose()
   {
      if (context != null)
         context.dispose();

      for (ModelInstance modelInstance : modelInstances)
      {
         ExceptionTools.handle(modelInstance.model::dispose, DefaultExceptionHandler.PRINT_MESSAGE);
      }
   }

   @Override
   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool)
   {
      for (ModelInstance modelInstance : modelInstances)
      {
         if (!skipHeadset || !(modelInstance == headsetModelInstance))
         {
            modelInstance.getRenderables(renderables, pool);
         }
      }
   }

   public GDXVRContext getContext()
   {
      return context;
   }
}
