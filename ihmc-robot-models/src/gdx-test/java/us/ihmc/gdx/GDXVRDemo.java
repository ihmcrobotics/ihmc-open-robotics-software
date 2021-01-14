package us.ihmc.gdx;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.math.Vector3;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.gdx.sceneManager.GDX3DSceneManager;
import us.ihmc.gdx.tools.GDXApplicationCreator;
import us.ihmc.gdx.tools.GDXModelPrimitives;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.vr.GDXVRContext;
import us.ihmc.gdx.vr.GDXVRManager;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class GDXVRDemo
{
   private GDX3DSceneManager sceneManager = new GDX3DSceneManager();
   private GDXVRManager vrManager = new GDXVRManager();
   private SideDependentList<ModelInstance> controllerCoordinateFrames = new SideDependentList<>();

   public GDXVRDemo()
   {
      GDXApplicationCreator.launchGDXApplication(new PrivateGDXApplication(), getClass());
   }

   class PrivateGDXApplication extends Lwjgl3ApplicationAdapter
   {
      @Override
      public void create()
      {
         sceneManager.create();
         vrManager.create();

         sceneManager.addCoordinateFrame(0.3);
         sceneManager.addModelInstance(new BoxesDemoModel().newInstance());
         sceneManager.addRenderableProvider(vrManager);

         for (RobotSide side : RobotSide.values)
         {
            ModelInstance coordinateFrameInstance = GDXModelPrimitives.createCoordinateFrameInstance(0.1);
            controllerCoordinateFrames.put(side, coordinateFrameInstance);
            sceneManager.addModelInstance(coordinateFrameInstance);
         }
      }

      @Override
      public void render()
      {
         vrManager.pollEvents();

         for (RobotSide side : vrManager.getControllers().sides())
         {
//            RigidBodyTransform transformToParent = vrManager.getControllers().get(side).getReferenceFrame().getTransformToParent();
//            GDXTools.toGDX(transformToParent, controllerCoordinateFrames.get(side).transform);

            controllerCoordinateFrames.get(side).transform.set(vrManager.getControllers().get(side).getWorldTransformGDX());

//
//            Vector3 position = vrManager.getControllers().get(side).getPosition(GDXVRContext.Space.World);
//            Vector3 direction = vrManager.getControllers().get(side).getDirection(GDXVRContext.Space.World);
//            Vector3 up = vrManager.getControllers().get(side).getUp(GDXVRContext.Space.World);
//
//            controllerCoordinateFrames.get(side).transform.setToLookAt(direction, up);
//            controllerCoordinateFrames.get(side).transform.setTranslation(position);
         }

         vrManager.render(sceneManager);

         sceneManager.glClearGray();
         sceneManager.render();
      }

      @Override
      public void dispose()
      {
         vrManager.dispose();
         sceneManager.dispose();
      }
   }

   public static void main(String[] args)
   {
      new GDXVRDemo();
   }
}
