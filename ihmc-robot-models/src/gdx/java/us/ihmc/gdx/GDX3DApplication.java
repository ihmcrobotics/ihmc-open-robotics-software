package us.ihmc.gdx;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.InputMultiplexer;
import com.badlogic.gdx.InputProcessor;
import com.badlogic.gdx.graphics.GL20;
import com.badlogic.gdx.graphics.g3d.Environment;
import com.badlogic.gdx.graphics.g3d.ModelBatch;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.DirectionalLightsAttribute;
import com.badlogic.gdx.graphics.g3d.environment.DirectionalLight;
import com.badlogic.gdx.graphics.profiling.GLProfiler;
import com.badlogic.gdx.utils.viewport.ScreenViewport;
import com.badlogic.gdx.utils.viewport.Viewport;

import java.util.ArrayList;

/**
 * TODO: Pause and resume?
 */
public class GDX3DApplication extends Lwjgl3ApplicationAdapter
{
   private InputMultiplexer inputMultiplexer;
   private FocusBasedGDXCamera camera3D;
   private Environment environment;
   private Viewport viewport;
   private ModelBatch modelBatch;

   private int currentWindowWidth;
   private int currentWindowHeight;

   private ArrayList<ModelInstance> modelInstances = new ArrayList<>();

   public void addModelInstance(ModelInstance modelInstance)
   {
      modelInstances.add(modelInstance);
   }

   public void addInputProcessor(InputProcessor inputProcessor)
   {
      inputMultiplexer.addProcessor(inputProcessor);
   }

   @Override
   public void create()
   {
      new GLProfiler(Gdx.graphics).enable();
      GDXTools.syncLogLevelWithLogTools();

      environment = new Environment();
      environment.set(new ColorAttribute(ColorAttribute.AmbientLight, 0.4f, 0.4f, 0.4f, 1f));
      DirectionalLightsAttribute directionalLights = new DirectionalLightsAttribute();
      directionalLights.lights.add(new DirectionalLight().set(0.8f, 0.8f, 0.8f, -1f, -0.8f, -0.2f));
      environment.set(directionalLights);

      modelBatch = new ModelBatch();

      inputMultiplexer = new InputMultiplexer();
      Gdx.input.setInputProcessor(inputMultiplexer);

      camera3D = new FocusBasedGDXCamera();
      inputMultiplexer.addProcessor(camera3D.getInputProcessor());
      viewport = new ScreenViewport(camera3D);
   }

   @Override
   public void resize(int width, int height)
   {
      this.currentWindowWidth = width;
      this.currentWindowHeight = height;
   }

   @Override
   public void render()
   {
      Gdx.gl.glClearColor(0.5019608f, 0.5019608f, 0.5019608f, 1.0f);
      Gdx.gl.glClear(GL20.GL_COLOR_BUFFER_BIT | GL20.GL_DEPTH_BUFFER_BIT);
      Gdx.gl.glEnable(GL20.GL_TEXTURE_2D);

      viewport.update(currentWindowWidth, currentWindowHeight * 3 / 4);
      Gdx.gl.glViewport(0, Gdx.graphics.getHeight() * 1 / 4, Gdx.graphics.getWidth(), Gdx.graphics.getHeight() * 3 / 4);

      camera3D.render();

      modelBatch.begin(camera3D);
      modelBatch.render(camera3D.getFocusPointSphere(), environment);
      for (ModelInstance modelInstance : modelInstances)
      {
         modelBatch.render(modelInstance, environment);
      }
      modelBatch.end();
   }

   @Override
   public void dispose()
   {
      for (ModelInstance modelInstance : modelInstances)
      {
         modelInstance.model.dispose();
      }

      camera3D.dispose();
      modelBatch.dispose();
   }

   @Override
   public boolean closeRequested()
   {
      return true;
   }

   public int getCurrentWindowWidth()
   {
      return currentWindowWidth;
   }

   public int getCurrentWindowHeight()
   {
      return currentWindowHeight;
   }
}
