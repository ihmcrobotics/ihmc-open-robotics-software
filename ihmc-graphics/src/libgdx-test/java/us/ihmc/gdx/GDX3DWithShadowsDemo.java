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
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.GL20;
import com.badlogic.gdx.graphics.PerspectiveCamera;
import com.badlogic.gdx.graphics.VertexAttributes.Usage;
import com.badlogic.gdx.graphics.g3d.Environment;
import com.badlogic.gdx.graphics.g3d.Material;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelBatch;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.environment.DirectionalShadowLight;
import com.badlogic.gdx.graphics.g3d.utils.CameraInputController;
import com.badlogic.gdx.graphics.g3d.utils.DepthShaderProvider;
import com.badlogic.gdx.graphics.g3d.utils.MeshPartBuilder;
import com.badlogic.gdx.graphics.g3d.utils.ModelBuilder;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.tests.utils.GdxTest;
import com.badlogic.gdx.utils.Array;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.sceneManager.GDX3DSceneTools;
import us.ihmc.gdx.tools.GDXApplicationCreator;
import us.ihmc.gdx.tools.GDXModelPrimitives;

public class GDX3DWithShadowsDemo {
   private PerspectiveCamera cam;
   private CameraInputController camController;

   private ModelBatch modelBatch;
   private Array<ModelInstance> instances = new Array<>();
   private Environment environment;

   private DirectionalShadowLight shadowLight;
   private ModelBatch shadowBatch;

   public void create () {
      //Camera initialization
      cam = new PerspectiveCamera(67, Gdx.graphics.getWidth(), Gdx.graphics.getHeight());
      cam.position.set(0f, 7f, 10f);
      cam.lookAt(0, 0, 0);
      cam.near = 1f;
      cam.far = 50f;
      cam.update();

      //Model batch stuff
      modelBatch = new ModelBatch();

      //Environment initialization
      environment = new Environment();
      environment.set(new ColorAttribute(ColorAttribute.AmbientLight, .4f, .4f, .4f, 1f));
      environment.add((shadowLight = new DirectionalShadowLight(1024, 1024, 5f, 5f, 1f, 100f))
                            .set(0.8f, 0.8f, 0.8f, -1f, -.8f, -.2f));
      environment.shadowMap = shadowLight;

      //Add model instances
      instances.add(GDXModelPrimitives.buildModelInstance(meshBuilder ->
                                                          {
                                                             meshBuilder.addBox(0.2, 0.2, 0.2, new Point3D(0, 0.15, 0), Color.RED);
                                                          }, "box"));
      instances.add(GDXModelPrimitives.buildModelInstance(meshBuilder ->
                                                          {
                                                             meshBuilder.addBox(1, 0.1, 1, new Point3D(), Color.YELLOW);
                                                          }, "box"));

      shadowBatch = new ModelBatch(new DepthShaderProvider());

      Gdx.input.setInputProcessor(camController = new CameraInputController(cam));
   }

   public void render () {
      camController.update();

      Gdx.gl.glViewport(0, 0, Gdx.graphics.getBackBufferWidth(), Gdx.graphics.getBackBufferHeight());

      GDX3DSceneTools.glClearGray();

      shadowLight.begin(Vector3.Zero, cam.direction);
      shadowBatch.begin(shadowLight.getCamera());
      shadowBatch.render(instances);
      shadowBatch.end();
      shadowLight.end();

      modelBatch.begin(cam);
      modelBatch.render(instances, environment);
      modelBatch.end();
   }

   public void dispose () {
      modelBatch.dispose();
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
