package us.ihmc.gdx;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.gdx.sceneManager.GDX3DSceneManager;
import us.ihmc.gdx.sceneManager.GDX3DSceneTools;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.simulation.GDXLowLevelDepthSensorSimulator;
import us.ihmc.gdx.tools.GDXApplicationCreator;
import us.ihmc.gdx.tools.GDXModelPrimitives;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.vr.GDXVRContext;
import us.ihmc.gdx.vr.GDXVRManager;
import us.ihmc.gdx.vr.VRDeviceAdapter;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import static us.ihmc.gdx.vr.GDXVRContext.VRControllerButtons.SteamVR_Trigger;

public class GDXVRDepthSensorDemo
{
   private ModelInstance cylinder;
   private boolean moveWithController = true;

   public GDXVRDepthSensorDemo()
   {
      GDX3DSceneManager sceneManager = new GDX3DSceneManager();
      GDXLowLevelDepthSensorSimulator depthSensorSimulator = new GDXLowLevelDepthSensorSimulator(90.0, 640, 480, 0.05, 10.0);
      GDXPointCloudRenderer pointCloudRenderer = new GDXPointCloudRenderer();
      GDXVRManager vrManager = new GDXVRManager();
      SideDependentList<ModelInstance> controllerCoordinateFrames = new SideDependentList<>();
      GDXApplicationCreator.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            sceneManager.create();
            vrManager.create();

            sceneManager.addCoordinateFrame(1.0);
            DepthSensorDemoObjectsModel depthSensorDemoObjectsModel = new DepthSensorDemoObjectsModel();
            cylinder = depthSensorDemoObjectsModel.buildCylinder();
            sceneManager.addModelInstance(cylinder);
            sceneManager.addModelInstance(depthSensorDemoObjectsModel.newInstance());
            sceneManager.addRenderableProvider(vrManager, GDXSceneLevel.VIRTUAL);

            depthSensorSimulator.create();
            depthSensorSimulator.getCamera().position.set(0.0f, -1.0f, 1.0f);
            depthSensorSimulator.getCamera().direction.set(0.0f, 0.0f, -1.0f);

            pointCloudRenderer.create((int) depthSensorSimulator.getCamera().viewportHeight * (int) depthSensorSimulator.getCamera().viewportWidth);
            sceneManager.addRenderableProvider(pointCloudRenderer, GDXSceneLevel.VIRTUAL);

            for (RobotSide side : RobotSide.values)
            {
               ModelInstance coordinateFrameInstance = GDXModelPrimitives.createCoordinateFrameInstance(0.1);
               controllerCoordinateFrames.put(side, coordinateFrameInstance);
               sceneManager.addModelInstance(coordinateFrameInstance, GDXSceneLevel.VIRTUAL);
            }

            vrManager.getContext().addListener(new VRDeviceAdapter()
            {
               @Override
               public void buttonPressed(GDXVRContext.VRDevice device, int button)
               {
                  if (button == SteamVR_Trigger)
                  {
                     moveWithController = !moveWithController;

                  }
               }
            });
         }

         @Override
         public void resize(int width, int height)
         {
            sceneManager.setViewportBounds(0, 0, width, height);
         }

         @Override
         public void render()
         {
            vrManager.pollEvents();

            for (RobotSide side : vrManager.getControllers().sides())
            {
               RigidBodyTransform transformToParent = vrManager.getControllers().get(side).getReferenceFrame().getTransformToParent();
               if (side.equals(RobotSide.LEFT) && moveWithController)
               {
                  depthSensorSimulator.setCameraWorldTransform(vrManager.getControllers().get(side).getWorldTransformGDX());
                  GDXTools.toGDX(transformToParent, controllerCoordinateFrames.get(side).transform);
               }
               else
               {
                  GDXTools.toGDX(transformToParent, cylinder.nodes.get(0).globalTransform);
               }
            }

            depthSensorSimulator.render(sceneManager);
            pointCloudRenderer.setPointsToRender(depthSensorSimulator.getPoints());

            GDX3DSceneTools.glClearGray();
            pointCloudRenderer.updateMesh();
            sceneManager.setViewportBoundsToWindow();
            sceneManager.render();
         }

         @Override
         public void dispose()
         {
            depthSensorSimulator.dispose();
            pointCloudRenderer.dispose();
            vrManager.dispose();
            sceneManager.dispose();
         }
      }, "GDX3DDemo", 1100, 800);
   }

   public static void main(String[] args)
   {
      new GDXVRDepthSensorDemo();
   }
}
