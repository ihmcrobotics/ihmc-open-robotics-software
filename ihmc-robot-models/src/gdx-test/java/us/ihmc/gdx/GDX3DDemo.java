package us.ihmc.gdx;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3Application;
import com.badlogic.gdx.backends.lwjgl3.Lwjgl3ApplicationConfiguration;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.GL20;
import com.badlogic.gdx.graphics.OrthographicCamera;
import com.badlogic.gdx.graphics.g3d.*;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.DirectionalLightsAttribute;
import com.badlogic.gdx.graphics.g3d.environment.DirectionalLight;
import com.badlogic.gdx.graphics.profiling.GLProfiler;
import com.badlogic.gdx.utils.viewport.ExtendViewport;
import com.badlogic.gdx.utils.viewport.Viewport;

/**
 * TODO: Pause and resume?
 */
public class GDX3DDemo extends Lwjgl3ApplicationAdapter
{
   private static final int INITIAL_WIDTH = 1100;
   private static final int INITIAL_HEIGHT = 800;
   private FocusBasedGDXCamera camera3D;
   private ModelBatch modelBatch;
   private Environment environment;
   private Viewport viewport;
   private Model rootModel;
   private ModelInstance rootModelInstance;

   private OrthographicCamera camera2D;

   public GDX3DDemo()
   {
      Lwjgl3ApplicationConfiguration applicationConfiguration = new Lwjgl3ApplicationConfiguration();
      applicationConfiguration.setTitle("GDX3DDemo");
      applicationConfiguration.setWindowedMode(INITIAL_WIDTH, INITIAL_HEIGHT);
      applicationConfiguration.useVsync(true);

      new Lwjgl3Application(this, applicationConfiguration);
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

      camera3D = new FocusBasedGDXCamera();
      viewport = new ExtendViewport(INITIAL_WIDTH, INITIAL_HEIGHT, camera3D);

      rootModel = new Model();

//      rootModel.nodes.addAll(camera.getFocusPointSphere().nodes);

      float distance = 5.0f;
      rootModel.nodes.addAll(GDXModelPrimitives.createBox(distance, distance, distance, Color.GREEN).nodes);
      rootModel.nodes.addAll(GDXModelPrimitives.createBox(-distance, distance, distance, Color.DARK_GRAY).nodes);
      rootModel.nodes.addAll(GDXModelPrimitives.createBox(distance, -distance, distance, Color.RED).nodes);
      rootModel.nodes.addAll(GDXModelPrimitives.createBox(-distance, -distance, distance, Color.ORANGE).nodes);
      rootModel.nodes.addAll(GDXModelPrimitives.createBox(distance, distance, -distance, Color.BLUE).nodes);
      rootModel.nodes.addAll(GDXModelPrimitives.createBox(-distance, distance, -distance, Color.BLACK).nodes);
      rootModel.nodes.addAll(GDXModelPrimitives.createBox(distance, -distance, -distance, Color.WHITE).nodes);
      rootModel.nodes.addAll(GDXModelPrimitives.createBox(-distance, -distance, -distance, Color.YELLOW).nodes);

      rootModel.nodes.addAll(GDXModelPrimitives.createCoordinateFrame(0.3).nodes);

      rootModelInstance = new ModelInstance(rootModel);
      rootModelInstance.nodes.addAll(camera3D.getFocusPointSphere().nodes);


//      camera2D.position.set(camera3D.viewportWidth / 3f, camera3D.viewportHeight / 3f, 0);
//      camera2D.update();
   }

   @Override
   public void resize(int width, int height)
   {
      viewport.update(width, height);
   }

   @Override
   public void render()
   {
      Gdx.gl.glClearColor(0.5019608f, 0.5019608f, 0.5019608f, 1.0f);
      Gdx.gl.glClear(GL20.GL_COLOR_BUFFER_BIT | GL20.GL_DEPTH_BUFFER_BIT);
      Gdx.gl.glClear(GL20.GL_COLOR_BUFFER_BIT | GL20.GL_DEPTH_BUFFER_BIT);
      Gdx.gl.glEnable(GL20.GL_TEXTURE_2D);

      camera3D.render();

      modelBatch.begin(camera3D);
      modelBatch.render(rootModelInstance, environment);
      // TODO add more render calls here
      modelBatch.end();

//      camera2D.update();
   }

   @Override
   public void dispose()
   {
      modelBatch.dispose();
      rootModel.dispose();
   }

   @Override
   public boolean closeRequested()
   {
      return true;
   }

   public static void main(String[] args)
   {
      new GDX3DDemo();
   }
}