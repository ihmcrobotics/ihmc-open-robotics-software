/*******************************************************************************
 * Copyright 2011 See AUTHORS file.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ******************************************************************************/

package us.ihmc.gdx;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.Camera;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.PerspectiveCamera;
import com.badlogic.gdx.graphics.g3d.Environment;
import com.badlogic.gdx.graphics.g3d.ModelBatch;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.environment.PointLight;
import com.badlogic.gdx.graphics.g3d.utils.CameraInputController;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.tests.g3d.shadows.system.ShadowSystem;
import com.badlogic.gdx.tests.g3d.shadows.system.classical.ClassicalShadowSystem;
import com.badlogic.gdx.tests.g3d.shadows.utils.AABBNearFarAnalyzer;
import com.badlogic.gdx.tests.g3d.shadows.utils.BoundingSphereDirectionalAnalyzer;
import com.badlogic.gdx.tests.g3d.shadows.utils.FixedShadowMapAllocator;
import com.badlogic.gdx.tests.g3d.shadows.utils.FrustumLightFilter;
import com.badlogic.gdx.utils.Array;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.gdx.sceneManager.GDX3DSceneTools;
import us.ihmc.gdx.tools.GDXApplicationCreator;
import us.ihmc.gdx.tools.GDXModelPrimitives;

public class GDX3DWithShadowsDemo {
   private PerspectiveCamera cam;
   private CameraInputController camController;

   private ModelBatch shadowBatch;
   private ModelBatch normalBatch;
   private Array<ModelInstance> shadowInstances = new Array<>();
   private Array<ModelInstance> normalInstances = new Array<>();
   private ModelInstance shadowBox;
   private ModelInstance normalBox;
   private Environment shadowEnvironment;
   private Environment normalEnvironment;

   private ShadowSystem system;
   private Array<ModelBatch> passBatches = new Array<ModelBatch>();

   public void create () {
      //Camera initialization
      cam = new PerspectiveCamera(67, Gdx.graphics.getWidth(), Gdx.graphics.getHeight());
      cam.position.set(0f, 7f, 10f);
      cam.lookAt(0, 0, 0);
      cam.near = 1f;
      cam.far = 50f;
      cam.update();

      //Environment initialization
      shadowEnvironment = new Environment();
      shadowEnvironment.set(new ColorAttribute(ColorAttribute.AmbientLight, 0.4f, 0.4f, 0.4f, 1.0f));

      normalEnvironment = new Environment();
      normalEnvironment.set(new ColorAttribute(ColorAttribute.AmbientLight, 0.4f, 0.4f, 0.4f, 1.0f));
      normalEnvironment.add(new PointLight().set(Color.WHITE, new Vector3(-2, 1, 1), 1.5f));
      normalEnvironment.add(new PointLight().set(Color.WHITE, new Vector3(-4, 1, -1), 1.5f));

      system = new ClassicalShadowSystem(new AABBNearFarAnalyzer(), new FixedShadowMapAllocator(2048, 4), new BoundingSphereDirectionalAnalyzer(), new FrustumLightFilter());

      PointLight light1 = new PointLight();
      light1.set(Color.WHITE, new Vector3(1, 1, 1), 1.5f);
      shadowEnvironment.add(light1);
      system.addLight(light1);

      PointLight light2 = new PointLight();
      light2.set(Color.WHITE, new Vector3(-1, 1, -1), 1.5f);
      shadowEnvironment.add(light2);
      system.addLight(light2);

      //Add model instances
      shadowInstances.add(shadowBox = GDXModelPrimitives.buildModelInstance(meshBuilder ->
                                                          {
                                                             meshBuilder.addBox(0.2, 0.2, 0.2, new Point3D(0, 0.15, 0), Color.BLUE);
                                                          }, "box"));
      shadowInstances.add(GDXModelPrimitives.buildModelInstance(meshBuilder ->
                                                          {
                                                             meshBuilder.addBox(1, 0.1, 1, new Point3D(), Color.GREEN);
                                                          }, "box"));
      shadowInstances.add(GDXModelPrimitives.buildModelInstance(meshBuilder ->
                                                                {
                                                                   meshBuilder.addBox(1, 0.1, 1, new Point3D(0, -0.1, 0), Color.DARK_GRAY);
                                                                }, "box"));
      normalInstances.add(GDXModelPrimitives.buildModelInstance(meshBuilder ->
                                                                {
                                                                   meshBuilder.addSphere(0.04f, new Point3D(1, 1, 1), Color.WHITE);
                                                                }, "light"));
      normalInstances.add(GDXModelPrimitives.buildModelInstance(meshBuilder ->
                                                                {
                                                                   meshBuilder.addSphere(0.04f, new Point3D(-1, 1, -1), Color.WHITE);
                                                                }, "light"));

      normalInstances.add(normalBox = GDXModelPrimitives.buildModelInstance(meshBuilder ->
                                                                      {
                                                                         meshBuilder.addBox(0.2, 0.2, 0.2, new Point3D(-3, 0.15, 0), Color.BLUE);
                                                                      }, "box"));
      normalInstances.add(GDXModelPrimitives.buildModelInstance(meshBuilder ->
                                                                {
                                                                   meshBuilder.addBox(1, 0.1, 1, new Point3D(-3, 0, 0), Color.GREEN);
                                                                }, "box"));
      normalInstances.add(GDXModelPrimitives.buildModelInstance(meshBuilder ->
                                                                {
                                                                   meshBuilder.addBox(1, 0.1, 1, new Point3D(-3, -0.1, 0), Color.RED);
                                                                }, "box"));
      normalInstances.add(GDXModelPrimitives.buildModelInstance(meshBuilder ->
                                                                {
                                                                   meshBuilder.addSphere(0.04f, new Point3D(-2, 1, 1), Color.WHITE);
                                                                }, "light"));
      normalInstances.add(GDXModelPrimitives.buildModelInstance(meshBuilder ->
                                                                {
                                                                   meshBuilder.addSphere(0.04f, new Point3D(-4, 1, -1), Color.WHITE);
                                                                }, "light"));

      system.init();

      for (int i = 0; i < system.getPassQuantity(); i++) {
         passBatches.add(new ModelBatch(system.getPassShaderProvider(i)));
      }

      shadowBatch = new ModelBatch(system.getShaderProvider());
      normalBatch = new ModelBatch();

      Gdx.input.setInputProcessor(camController = new CameraInputController(cam));

      time = System.currentTimeMillis();
   }

   private long time;
   private long prev;
   private double polar = 0;

   public void render () {
      prev = time;
      time = System.currentTimeMillis();
      double delta = (time - prev) / 1000f;
      double pos = time / 1000f;
      polar += delta * 3;

      camController.update();

      shadowBox.transform.setToTranslation((float) Math.sin(polar) / 4, 0, (float) Math.cos(polar) / 4);
      normalBox.transform.setToTranslation((float) Math.sin(polar) / 4, 0, (float) Math.cos(polar) / 4);

      Gdx.gl.glViewport(0, 0, Gdx.graphics.getBackBufferWidth(), Gdx.graphics.getBackBufferHeight());

      GDX3DSceneTools.glClearGray();

      system.begin(cam, shadowInstances);
      system.update();
      for (int i = 0; i < system.getPassQuantity(); i++) {
         system.begin(i);
         Camera camera;
         while ((camera = system.next()) != null) {
            passBatches.get(i).begin(camera);
            passBatches.get(i).render(shadowInstances, shadowEnvironment);
            passBatches.get(i).end();
         }
         system.end(i);
      }
      system.end();

      shadowBatch.begin(cam);
      shadowBatch.render(shadowInstances, shadowEnvironment);
      shadowBatch.end();

      normalBatch.begin(cam);
      normalBatch.render(normalInstances, normalEnvironment);
      normalBatch.end();
   }

   public void dispose () {
      shadowBatch.dispose();
   }

   public static void main(String[] args)
   {
      GDX3DWithShadowsDemo demo = new GDX3DWithShadowsDemo();
      GDXApplicationCreator.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            demo.create();
         }

         @Override
         public void render()
         {
            demo.render();
         }

         @Override
         public void dispose()
         {
            demo.dispose();
         }
      }, GDX3DWithShadowsDemo.class);
   }
}
