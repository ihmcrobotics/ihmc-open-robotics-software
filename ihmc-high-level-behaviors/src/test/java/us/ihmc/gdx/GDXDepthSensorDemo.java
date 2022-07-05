package us.ihmc.gdx;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.math.Matrix4;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.gdx.sceneManager.GDX3DBareBonesScene;
import us.ihmc.gdx.sceneManager.GDX3DSceneTools;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.simulation.sensors.GDXLowLevelDepthSensorSimulator;
import us.ihmc.gdx.tools.GDXApplicationCreator;
import us.ihmc.gdx.tools.GDXTools;

public class GDXDepthSensorDemo
{
   public GDXDepthSensorDemo()
   {
      GDX3DBareBonesScene sceneManager = new GDX3DBareBonesScene();
      GDXLowLevelDepthSensorSimulator depthSensorSimulator = new GDXLowLevelDepthSensorSimulator("Sensor", 80.0, 800, 600, 0.05, 5.0, 0.03, 0.07, false);
      GDXPointCloudRenderer pointCloudRenderer = new GDXPointCloudRenderer();
      GDXApplicationCreator.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            sceneManager.create();

            sceneManager.addCoordinateFrame(0.3);
            sceneManager.addModelInstance(new DepthSensorDemoObjectsModel().newInstance());

            pointCloudRenderer.create(depthSensorSimulator.getNumberOfPoints());
            sceneManager.addRenderableProvider(pointCloudRenderer, GDXSceneLevel.VIRTUAL);

            depthSensorSimulator.create(pointCloudRenderer.getVertexBuffer());
            RigidBodyTransform transformToWorld = new RigidBodyTransform();
            transformToWorld.getRotation().appendYawRotation(Math.toRadians(180.0));
            transformToWorld.getRotation().appendPitchRotation(Math.toRadians(90.0));
            transformToWorld.getTranslation().add(0.0f, -0.5f, 2.0f);
            Matrix4 gdxTransform = new Matrix4();
            GDXTools.toGDX(transformToWorld, gdxTransform);
            depthSensorSimulator.setCameraWorldTransform(gdxTransform);
         }

         @Override
         public void render()
         {
            depthSensorSimulator.render(sceneManager.getScene(), Color.WHITE, 0.01f);
            pointCloudRenderer.updateMeshFastest(depthSensorSimulator.getNumberOfPoints());

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
