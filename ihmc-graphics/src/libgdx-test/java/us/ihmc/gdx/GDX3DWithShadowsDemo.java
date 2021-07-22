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
import com.badlogic.gdx.graphics.g3d.*;
import com.badlogic.gdx.graphics.g3d.attributes.BlendingAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.environment.DirectionalShadowLight;
import com.badlogic.gdx.graphics.g3d.utils.*;
import com.badlogic.gdx.graphics.glutils.ShaderProgram;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.tests.utils.GdxTest;
import com.badlogic.gdx.utils.Array;
import org.lwjgl.opengl.GL30;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.lighting.GDXDirectionalLight;
import us.ihmc.gdx.lighting.GDXLight;
import us.ihmc.gdx.lighting.GDXSceneShader;
import us.ihmc.gdx.lighting.GDXShadowManager;
import us.ihmc.gdx.sceneManager.GDX3DSceneTools;
import us.ihmc.gdx.tools.GDXApplicationCreator;
import us.ihmc.gdx.tools.GDXModelPrimitives;

public class GDX3DWithShadowsDemo {
   private PerspectiveCamera cam;
   private CameraInputController camController;

   private ModelBatch modelBatch;
   private Array<ModelInstance> instances = new Array<>();
   private ModelInstance box;

   private GDXShadowManager manager;

   public void create () {
      //GDX stuff
      Gdx.gl.glEnable(GL30.GL_BLEND);

      //Camera initialization
      cam = new PerspectiveCamera(67, Gdx.graphics.getWidth(), Gdx.graphics.getHeight());
      cam.position.set(0f, 7f, 10f);
      cam.lookAt(0, 0, 0);
      cam.near = 1f;
      cam.far = 50f;
      cam.update();

      //Model batch and shadow stuff
      ShaderProgram program = new ShaderProgram(GDXShadowManager.getVertexShader(), GDXShadowManager.getFragmentShader());
      modelBatch = new ModelBatch(new DefaultShaderProvider() {
         @Override
         protected Shader createShader(Renderable renderable)
         {
            return new GDXSceneShader(renderable, program);
         }
      });

      manager = new GDXShadowManager(program);
      GDXLight light = new GDXDirectionalLight(new Vector3(1, 1, 1), new Vector3(-1, -1, -1));
      light.init(); //TODO change me?
      manager.addLight(light);

      //Add model instances
      instances.add(box = GDXModelPrimitives.buildModelInstance(meshBuilder ->
                                                          {
                                                             meshBuilder.addBox(0.2, 0.2, 0.2, new Point3D(0, 0.15, 0), Color.RED);
                                                          }, "box"));
      instances.add(GDXModelPrimitives.buildModelInstance(meshBuilder ->
                                                          {
                                                             meshBuilder.addBox(1, 0.1, 1, new Point3D(), Color.YELLOW);
                                                          }, "box"));

      Gdx.input.setInputProcessor(camController = new CameraInputController(cam));
   }

   public void render () {
      box.transform.setToTranslation((float) Math.sin(System.currentTimeMillis() / 500d) / 4, 0, (float) Math.cos(System.currentTimeMillis() / 500d) / 4);

      camController.update();

      Gdx.gl.glViewport(0, 0, Gdx.graphics.getBackBufferWidth(), Gdx.graphics.getBackBufferHeight());

      GDX3DSceneTools.glClearGray();

      manager.render(instances);

      modelBatch.begin(cam);
      modelBatch.render(instances);
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
