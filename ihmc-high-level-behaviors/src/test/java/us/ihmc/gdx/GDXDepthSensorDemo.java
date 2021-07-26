package us.ihmc.gdx;

import us.ihmc.gdx.sceneManager.GDX3DSceneManager;
import us.ihmc.gdx.sceneManager.GDX3DSceneTools;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.simulation.GDXLowLevelDepthSensorSimulator;
import us.ihmc.gdx.tools.GDXApplicationCreator;

public class GDXDepthSensorDemo
{
   public GDXDepthSensorDemo()
   {
      GDX3DSceneManager sceneManager = new GDX3DSceneManager();
      GDXLowLevelDepthSensorSimulator depthSensorSimulator = new GDXLowLevelDepthSensorSimulator("Sensor", 80.0, 800, 600, 0.05, 5.0);
      GDXPointCloudRenderer pointCloudRenderer = new GDXPointCloudRenderer();
      GDXApplicationCreator.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            sceneManager.create();

            sceneManager.addCoordinateFrame(0.3);
            sceneManager.addModelInstance(new DepthSensorDemoObjectsModel().newInstance());

            depthSensorSimulator.create();
            depthSensorSimulator.getCamera().position.set(0.0f, -1.0f, 1.0f);
            depthSensorSimulator.getCamera().direction.set(0.0f, 0.0f, -1.0f);

            pointCloudRenderer.create((int) depthSensorSimulator.getCamera().viewportHeight * (int) depthSensorSimulator.getCamera().viewportWidth);
            sceneManager.addRenderableProvider(pointCloudRenderer, GDXSceneLevel.VIRTUAL);
         }

         @Override
         public void render()
         {
            depthSensorSimulator.getCamera().position.set(0.0f, -0.5f, 2.0f);
//            depthSensorSimulator.getCamera().position.set(0.0f, 0.0f, 0.0f);
            depthSensorSimulator.getCamera().direction.set(0.0f, 0.0f, (float) -Math.toRadians(90.0));

            depthSensorSimulator.render(sceneManager);
            pointCloudRenderer.setPointsToRender(depthSensorSimulator.getPoints());

            GDX3DSceneTools.glClearGray();
            pointCloudRenderer.updateMesh();
            sceneManager.setViewportBoundsToWindow();
            sceneManager.render();
         }
      }, "GDX3DDemo", 1100, 800);
   }

   public static void main(String[] args)
   {
      new GDXDepthSensorDemo();
   }
}
