package us.ihmc.rdx;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.math.Matrix4;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.simulation.sensors.RDXLowLevelDepthSensorSimulator;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.ui.RDXBaseUI;

/**
 * TODO: Fix me
 */
public class RDXDepthSensorDemo
{
   private RDXLowLevelDepthSensorSimulator depthSensorSimulator;
   private RDXPointCloudRenderer pointCloudRenderer;

   public RDXDepthSensorDemo()
   {
      RDXBaseUI baseUI = new RDXBaseUI();
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            depthSensorSimulator = new RDXLowLevelDepthSensorSimulator("Sensor", 80.0, 800, 600, 0.05, 5.0, 0.03, 0.07, false);
            pointCloudRenderer = new RDXPointCloudRenderer();

            baseUI.getPrimaryScene().addCoordinateFrame(0.3);
            baseUI.getPrimaryScene().addModelInstance(new DepthSensorDemoObjectsModel().newInstance());

            pointCloudRenderer.create(depthSensorSimulator.getNumberOfPoints());
            baseUI.getPrimaryScene().addRenderableProvider(pointCloudRenderer, RDXSceneLevel.VIRTUAL);

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
            depthSensorSimulator.render(baseUI.getPrimaryScene(), false, Color.WHITE, 1.0f);
            pointCloudRenderer.updateMeshFastest(depthSensorSimulator.getNumberOfPoints());

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXDepthSensorDemo();
   }
}
