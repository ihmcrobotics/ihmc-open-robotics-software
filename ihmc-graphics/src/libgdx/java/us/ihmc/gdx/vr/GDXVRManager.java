package us.ihmc.gdx.vr;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.gdx.imgui.ImGui3DViewInput;
import us.ihmc.gdx.sceneManager.GDX3DSceneManager;
import us.ihmc.gdx.sceneManager.GDX3DSceneTools;
import us.ihmc.gdx.ui.GDXPose3DWidget;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.ArrayList;
import java.util.HashSet;

import static us.ihmc.gdx.vr.GDXVRControllerButtons.SteamVR_Touchpad;

public class GDXVRManager implements RenderableProvider
{
   private static boolean ENABLE_VR = Boolean.parseBoolean(System.getProperty("enable.vr"));

   private GDXVRContext context;
   private final HashSet<ModelInstance> modelInstances = new HashSet<>();
   private ModelInstance headsetModelInstance;
   private final SideDependentList<GDXVRDevice> controllers = new SideDependentList<>();
   private boolean skipHeadset = false;
   private boolean holdingTouchpadToMove = false;
   private final Point3D initialVRSpacePosition = new Point3D();
   private final Point3D initialVRControllerPosition = new Point3D();
   private final Point3D currentVRControllerPosition = new Point3D();
   private final Vector3D deltaVRControllerPosition = new Vector3D();
   private final Point3D resultVRSpacePosition = new Point3D();
   private final Point3D lastVRSpacePosition = new Point3D();
   private final ArrayList<Runnable> thingsToCreateOnEnable = new ArrayList<>();
   private final GDXPose3DWidget scenePose = new GDXPose3DWidget("VR Space");

   public void create()
   {
      ENABLE_VR = true;
      System.setProperty("enable.vr", "true");

      context = new GDXVRContext();

      scenePose.create();

      context.getPerEyeData().get(RobotSide.LEFT).getCamera().far = 100f;
      context.getPerEyeData().get(RobotSide.RIGHT).getCamera().far = 100f;

      context.addListener(new GDXVRDeviceListener()
      {
         @Override
         public void connected(GDXVRDevice device)
         {
            LogTools.info("{} connected", device);
            if (device.getModelInstance() != null)
            {
               modelInstances.add(device.getModelInstance());

               if (device.getType() == GDXVRDeviceType.HeadMountedDisplay)
               {
                  headsetModelInstance = device.getModelInstance();
               }
               else if(device.getType() == GDXVRDeviceType.Controller)
               {
                  if (device.getControllerRole() == GDXVRControllerRole.LeftHand)
                  {
                     controllers.set(RobotSide.LEFT, device);
                  }
                  else if (device.getControllerRole() == GDXVRControllerRole.RightHand)
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
         public void disconnected(GDXVRDevice device)
         {
            LogTools.info("{} disconnected", device);
         }

         @Override
         public void buttonPressed(GDXVRDevice device, int button)
         {
            LogTools.info("{} button pressed: {}", device, button);
         }

         @Override
         public void buttonReleased(GDXVRDevice device, int button)
         {
            LogTools.info("{} button released: {}", device, button);
         }
      });

      context.addListener(new GDXVRDeviceAdapter()
      {
         @Override
         public void buttonPressed(GDXVRDevice device, int button)
         {
            if (device == controllers.get(RobotSide.RIGHT) && button == SteamVR_Touchpad)
            {
//               GDXTools.toEuclid(controllers.get(RobotSide.RIGHT).getWorldTransformGDX(), initialVRControllerPosition);
//               GDXTools.toEuclid(context.getTrackerSpaceOriginToWorldSpaceTranslationOffset(), initialVRSpacePosition);
//               context.getToZUpXForward().transform(initialVRSpacePosition);
//               lastVRSpacePosition.set(initialVRSpacePosition);
               holdingTouchpadToMove = true;
            }
         }

         @Override
         public void buttonReleased(GDXVRDevice device, int button)
         {
            if (device == controllers.get(RobotSide.RIGHT) && button == SteamVR_Touchpad)
            {
               holdingTouchpadToMove = false;
            }
         }
      });

      for (Runnable runnable : thingsToCreateOnEnable)
      {
         runnable.run();
      }
   }

   public void pollEvents()
   {
      context.pollEvents();
   }

   public void render(GDX3DSceneManager sceneManager)
   {
      // Wait for VR setup to be ready. This is the primary indicator.
      if (context.getDeviceByType(GDXVRDeviceType.HeadMountedDisplay) == null)
         return;

      context.teleport(scenePose.getTransform());

      if (holdingTouchpadToMove)
      {
//         GDXTools.toEuclid(controllers.get(RobotSide.RIGHT).getWorldTransformGDX(), currentVRControllerPosition);
//         resultVRSpacePosition.set(initialVRSpacePosition);
//         deltaVRControllerPosition.sub(currentVRControllerPosition, initialVRControllerPosition);
//         resultVRSpacePosition.sub(deltaVRControllerPosition);
//         resultVRSpacePosition.sub(lastVRSpacePosition);
//         GDXTools.toGDX(resultVRSpacePosition, context.getTrackerSpaceOriginToWorldSpaceTranslationOffset());
//         lastVRSpacePosition.set(resultVRSpacePosition);
//         context.getToZUpXForward().inverseTransform(currentVRControllerPosition);
//         GDXTools.toGDX(currentVRControllerPosition, context.getTrackerSpaceOriginToWorldSpaceTranslationOffset());
//         context.getTrackerSpaceOriginToWorldSpaceTranslationOffset().set(0.1f, 0.0f, 0.0f);
      }
      else
      {
//         context.getTrackerSpaceOriginToWorldSpaceTranslationOffset().set(0.0f, 0.0f, 0.0f);
      }

      context.begin();
      renderScene(RobotSide.LEFT, sceneManager);
      renderScene(RobotSide.RIGHT, sceneManager);
      context.end();
   }

   private void renderScene(RobotSide eye, GDX3DSceneManager sceneManager)
   {
      GDXVRPerEyeData eyeData = context.getPerEyeData().get(eye);
      GDXVRCamera camera = eyeData.getCamera();

      context.beginEye(eye);

      int width = eyeData.getFrameBuffer().getWidth();
      int height = eyeData.getFrameBuffer().getHeight();
      Gdx.gl.glViewport(0, 0, width, height);

      GDX3DSceneTools.glClearGray();

      skipHeadset = true;
      sceneManager.renderToCamera(camera);
      skipHeadset = false;

      context.endEye();
   }

   public SideDependentList<GDXVRDevice> getControllers()
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

   public void create(Runnable runnable)
   {
      thingsToCreateOnEnable.add(runnable);
   }

   public void process3DViewInput(ImGui3DViewInput input)
   {
      scenePose.process3DViewInput(input);
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

      scenePose.getRenderables(renderables, pool);
   }

   public GDXVRContext getContext()
   {
      return context;
   }

   public static boolean isVREnabled()
   {
      return ENABLE_VR;
   }
}
