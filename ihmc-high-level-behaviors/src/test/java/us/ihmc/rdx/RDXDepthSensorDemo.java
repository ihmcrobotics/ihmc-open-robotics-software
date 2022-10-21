package us.ihmc.rdx;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.math.Matrix4;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.rdx.sceneManager.RDX3DBareBonesScene;
import us.ihmc.rdx.sceneManager.RDX3DSceneTools;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.simulation.sensors.RDXLowLevelDepthSensorSimulator;
import us.ihmc.rdx.tools.LibGDXApplicationCreator;
import us.ihmc.rdx.tools.LibGDXTools;

public class RDXDepthSensorDemo
{
   public RDXDepthSensorDemo()
   {
      RDX3DBareBonesScene sceneManager = new RDX3DBareBonesScene();
      RDXLowLevelDepthSensorSimulator depthSensorSimulator = new RDXLowLevelDepthSensorSimulator("Sensor", 80.0, 800, 600, 0.05, 5.0, 0.03, 0.07, false);
      RDXPointCloudRenderer pointCloudRenderer = new RDXPointCloudRenderer();
      LibGDXApplicationCreator.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            sceneManager.create();

            sceneManager.addCoordinateFrame(0.3);
            sceneManager.addModelInstance(new DepthSensorDemoObjectsModel().newInstance());

            pointCloudRenderer.create(depthSensorSimulator.getNumberOfPoints());
            sceneManager.addRenderableProvider(pointCloudRenderer, RDXSceneLevel.VIRTUAL);

            depthSensorSimulator.create(pointCloudRenderer.getVertexBuffer());
            RigidBodyTransform transformToWorld = new RigidBodyTransform();
            transformToWorld.getRotation().appendYawRotation(Math.toRadians(180.0));
            transformToWorld.getRotation().appendPitchRotation(Math.toRadians(90.0));
            transformToWorld.getTranslation().add(0.0f, -0.5f, 2.0f);
            Matrix4 gdxTransform = new Matrix4();
            LibGDXTools.toLibGDX(transformToWorld, gdxTransform);
            depthSensorSimulator.setCameraWorldTransform(gdxTransform);
         }

         @Override
         public void render()
         {
            depthSensorSimulator.render(sceneManager.getScene(), false, Color.WHITE, 0.01f);
            pointCloudRenderer.updateMeshFastest(depthSensorSimulator.getNumberOfPoints());

            RDX3DSceneTools.glClearGray();
            pointCloudRenderer.updateMesh();
            sceneManager.setViewportBoundsToWindow();
            sceneManager.render();
         }
      }, "RDX3DDemo", 1100, 800);
   }

   public static void main(String[] args)
   {
      new RDXDepthSensorDemo();
   }
}
