package us.ihmc.gdx;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.PerspectiveCamera;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.utils.CameraInputController;
import org.lwjgl.opengl.GL41;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.gdx.lighting.*;
import us.ihmc.gdx.sceneManager.GDX3DSceneTools;
import us.ihmc.gdx.tools.GDXApplicationCreator;
import us.ihmc.gdx.tools.GDXModelPrimitives;

import java.util.ArrayList;

public class GDX3DWithShadowsDemo
{
   private PerspectiveCamera camera;
   private CameraInputController cameraController;
   private final ArrayList<ModelInstance> instances = new ArrayList<>();
   private ModelInstance box;
   private GDXPointLight light;
   private GDXShadowManager manager;

   public GDX3DWithShadowsDemo()
   {
      GDXApplicationCreator.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            //GDX stuff
            GL41.glEnable(GL41.GL_BLEND);

            //Camera initialization
            camera = new PerspectiveCamera(67, Gdx.graphics.getWidth(), Gdx.graphics.getHeight());
            camera.position.set(0f, 7f, 10f);
            camera.lookAt(0, 0, 0);
            camera.near = 1f;
            camera.far = 50f;
            camera.update();

            manager = new GDXShadowManager(1.0f, 0.4f);
            light = new GDXPointLight();
            light.getPosition().set(6.0, 3.0, -6.0);
            manager.getPointLights().add(light);
            GDXDirectionalLight directionalLight = new GDXDirectionalLight();
            directionalLight.getPosition().set(10.0, 10.0, 10.0);
            directionalLight.getDirection().set(-1.0, -1.0, -1.0);
            manager.getDirectionalLights().add(directionalLight);

            //Add model instances
            instances.add(box = GDXModelPrimitives.buildModelInstance(meshBuilder ->
            {
               meshBuilder.addBox(2, 2, 2, new Point3D(0, 1.5, 0), Color.RED);
            }, "box"));
            instances.add(GDXModelPrimitives.buildModelInstance(meshBuilder ->
            {
               meshBuilder.addBox(10, 1, 10, new Point3D(), Color.YELLOW);
            }, "box"));

            Gdx.input.setInputProcessor(cameraController = new CameraInputController(camera));
         }

         @Override
         public void render()
         {
            box.transform.setToTranslation((float) Math.sin(System.currentTimeMillis() / 500d) * 2, 0, (float) Math.cos(System.currentTimeMillis() / 500d) * 2);
            light.getPosition().set((float) Math.sin(System.currentTimeMillis() / 1000d) * 10, 10, (float) Math.cos(System.currentTimeMillis() / 1000d) * 10);
            light.update();

            cameraController.update();

            GL41.glViewport(0, 0, Gdx.graphics.getBackBufferWidth(), Gdx.graphics.getBackBufferHeight());

            GDX3DSceneTools.glClearGray();

            manager.renderShadows(camera, instances);
            manager.preRender(camera);
            manager.render(instances);
            manager.postRender();
         }

         @Override
         public void dispose()
         {
            manager.dispose();
         }
      }, GDX3DWithShadowsDemo.class);
   }

   public static void main(String[] args)
   {
      new GDX3DWithShadowsDemo();
   }
}
