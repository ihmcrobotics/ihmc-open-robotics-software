package us.ihmc.gdx;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.math.Matrix4;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.gdx.sceneManager.GDX3DSceneManager;
import us.ihmc.gdx.sceneManager.GDX3DSceneTools;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.simulation.GDXLowLevelDepthSensorSimulator;
import us.ihmc.gdx.tools.GDXApplicationCreator;
import us.ihmc.gdx.tools.GDXModelPrimitives;
import us.ihmc.gdx.vr.GDXVRDevice;
import us.ihmc.gdx.vr.GDXVRManager;
import us.ihmc.gdx.vr.GDXVRDeviceAdapter;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import static us.ihmc.gdx.vr.GDXVRControllerButtons.SteamVR_Trigger;

public class GDXVRDepthSensorDemo
{
   private ModelInstance cylinder;
   private boolean moveWithController = true;
   private final Matrix4 tempTransform = new Matrix4();

   public GDXVRDepthSensorDemo()
   {
      GDX3DSceneManager sceneManager = new GDX3DSceneManager();
      GDXLowLevelDepthSensorSimulator depthSensorSimulator = new GDXLowLevelDepthSensorSimulator("Sensor", 90.0, 640, 480, 0.05, 10.0);
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

            vrManager.getContext().addListener(new GDXVRDeviceAdapter()
            {
               @Override
               public void buttonPressed(GDXVRDevice device, int button)
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
               if (side.equals(RobotSide.LEFT) && moveWithController)
               {
                  vrManager.getControllers().get(side).getPose(ReferenceFrame.getWorldFrame(), tempTransform);
                  depthSensorSimulator.setCameraWorldTransform(tempTransform);
                  controllerCoordinateFrames.get(side).transform.set(tempTransform);
               }
               else
               {
                  vrManager.getControllers().get(side).getPose(ReferenceFrame.getWorldFrame(), cylinder.nodes.get(0).globalTransform);
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
