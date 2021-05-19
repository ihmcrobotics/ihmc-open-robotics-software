package us.ihmc.gdx;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.gdx.sceneManager.GDX3DSceneManager;
import us.ihmc.gdx.tools.BoxesDemoModel;
import us.ihmc.gdx.tools.GDXApplicationCreator;
import us.ihmc.gdx.tools.GDXModelPrimitives;
import us.ihmc.gdx.vr.GDXVRManager;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class GDXVRDemo
{
   private final GDX3DSceneManager sceneManager = new GDX3DSceneManager();
   private final GDXVRManager vrManager = new GDXVRManager();
   private final SideDependentList<ModelInstance> controllerCoordinateFrames = new SideDependentList<>();

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
            vrManager.getControllers().get(side).getPose(ReferenceFrame.getWorldFrame(), controllerCoordinateFrames.get(side).transform);
         }

         vrManager.render(sceneManager);

         sceneManager.setViewportBoundsToWindow();
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
