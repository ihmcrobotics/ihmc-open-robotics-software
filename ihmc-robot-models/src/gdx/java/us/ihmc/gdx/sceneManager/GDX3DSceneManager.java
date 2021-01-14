package us.ihmc.gdx.sceneManager;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.InputMultiplexer;
import com.badlogic.gdx.InputProcessor;
import com.badlogic.gdx.graphics.Camera;
import com.badlogic.gdx.graphics.g3d.*;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.DirectionalLightsAttribute;
import com.badlogic.gdx.graphics.g3d.environment.DirectionalLight;
import com.badlogic.gdx.graphics.profiling.GLProfiler;
import com.badlogic.gdx.utils.viewport.ScreenViewport;
import com.badlogic.gdx.utils.viewport.Viewport;
import org.lwjgl.opengl.GL32;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.gdx.FocusBasedGDXCamera;
import us.ihmc.gdx.tools.GDXModelPrimitives;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.log.LogTools;

import java.util.HashSet;

/**
 * TODO: Pause and resume?
 */
public class GDX3DSceneManager
{
   public static final float CLEAR_COLOR = 0.5019608f;
   private InputMultiplexer inputMultiplexer;
   private FocusBasedGDXCamera camera3D;
   private Environment environment;
   private Viewport viewport;
   private ModelBatch modelBatch;

   private int x = 0;
   private int y = 0;
   private int width = -1;
   private int height = -1;

   private final HashSet<ModelInstance> modelInstances = new HashSet<>();
   private final HashSet<GDXRenderable> renderables = new HashSet<>();

   private boolean firstRenderStarted = false;

   public void create()
   {
      new GLProfiler(Gdx.graphics).enable();
      GDXTools.syncLogLevelWithLogTools();

      environment = new Environment();
      float ambientColor = 0.7f;
      float pointColor = 0.07f;
      float pointDistance = 2.0f;
      float pointIntensity = 1.0f;
      environment.set(new ColorAttribute(ColorAttribute.AmbientLight, ambientColor, ambientColor, ambientColor, 1.0f));
      // Point lights not working; not sure why @dcalvert
      //      PointLightsAttribute pointLights = new PointLightsAttribute();
      //      pointLights.lights.add(new PointLight().set(pointColor, pointColor, pointColor, pointDistance, pointDistance, pointDistance, pointIntensity));
      //      pointLights.lights.add(new PointLight().set(pointColor, pointColor, pointColor, -pointDistance, pointDistance, pointDistance, pointIntensity));
      //      pointLights.lights.add(new PointLight().set(pointColor, pointColor, pointColor, -pointDistance, -pointDistance, pointDistance, pointIntensity));
      //      pointLights.lights.add(new PointLight().set(pointColor, pointColor, pointColor, pointDistance, -pointDistance, pointDistance, pointIntensity));
      //      environment.set(pointLights);
      DirectionalLightsAttribute directionalLights = new DirectionalLightsAttribute();
      directionalLights.lights.add(new DirectionalLight().set(pointColor, pointColor, pointColor, -pointDistance, -pointDistance, -pointDistance));
      directionalLights.lights.add(new DirectionalLight().set(pointColor, pointColor, pointColor, pointDistance, -pointDistance, -pointDistance));
      directionalLights.lights.add(new DirectionalLight().set(pointColor, pointColor, pointColor, pointDistance, pointDistance, -pointDistance));
      directionalLights.lights.add(new DirectionalLight().set(pointColor, pointColor, pointColor, -pointDistance, pointDistance, -pointDistance));
      environment.set(directionalLights);

      modelBatch = new ModelBatch();

      inputMultiplexer = new InputMultiplexer();
      Gdx.input.setInputProcessor(inputMultiplexer);

      camera3D = new FocusBasedGDXCamera();
      addModelInstance(camera3D.getFocusPointSphere());
      inputMultiplexer.addProcessor(camera3D.getInputProcessor());
      viewport = new ScreenViewport(camera3D);

      glClearGray();
      Gdx.gl.glEnable(GL32.GL_TEXTURE_2D);
   }

   public void renderBefore()
   {
      renderBefore(GDXSceneLevel.VIRTUAL);
   }

   public void renderBefore(GDXSceneLevel sceneLevel)
   {
      if (!firstRenderStarted)
      {
         firstRenderStarted = true;
         LogTools.info("Starting first render.");
      }

      if (width < 0)
         width = getCurrentWindowWidth();
      if (height < 0)
         height = getCurrentWindowHeight();

      viewport.update(width, height);

      modelBatch.begin(camera3D);

      Gdx.gl.glViewport(x, y, width, height);

      renderRegisteredObjectsWithEnvironment(modelBatch, sceneLevel);
   }

   public void renderRegisteredObjectsWithEnvironment(ModelBatch modelBatch)
   {
      renderRegisteredObjectsWithEnvironment(modelBatch, GDXSceneLevel.VIRTUAL);
   }

   public void renderRegisteredObjectsWithEnvironment(ModelBatch modelBatch, GDXSceneLevel sceneLevel)
   {
      for (GDXRenderable renderable : renderables)
      {
         if (sceneLevel.ordinal() >= renderable.getSceneType().ordinal())
         {
            modelBatch.render(renderable.getRenderableProvider(), environment);
         }
      }
   }

   public void renderAfter()
   {
      modelBatch.end();
   }

   public void renderToCamera(Camera camera)
   {
      modelBatch.begin(camera);
      renderRegisteredObjectsWithEnvironment(modelBatch);
      renderAfter();
   }

   public void render()
   {
      render(GDXSceneLevel.VIRTUAL);
   }

   public void render(GDXSceneLevel sceneLevel)
   {
      renderBefore(sceneLevel);
      renderAfter();
   }

   public void dispose()
   {
      for (ModelInstance modelInstance : modelInstances)
      {
         ExceptionTools.handle(modelInstance.model::dispose, DefaultExceptionHandler.PRINT_MESSAGE);
      }

      ExceptionTools.handle(() -> camera3D.dispose(), DefaultExceptionHandler.PRINT_MESSAGE);
      modelBatch.dispose();
   }

   public boolean closeRequested()
   {
      return true;
   }

   public void addModelInstance(ModelInstance modelInstance)
   {
      addModelInstance(modelInstance, GDXSceneLevel.REAL_ENVIRONMENT);
   }

   public void addModelInstance(ModelInstance modelInstance, GDXSceneLevel sceneLevel)
   {
      addRenderableProvider(modelInstance, sceneLevel);
      modelInstances.add(modelInstance);
   }

   public void addCoordinateFrame(double size)
   {
      addModelInstance(GDXModelPrimitives.createCoordinateFrameInstance(size), GDXSceneLevel.VIRTUAL);
   }

   public void addRenderableProvider(RenderableProvider renderableProvider)
   {
      addRenderableProvider(renderableProvider, GDXSceneLevel.REAL_ENVIRONMENT);
   }

   public void addRenderableProvider(RenderableProvider renderableProvider, GDXSceneLevel sceneLevel)
   {
      renderables.add(new GDXRenderable(renderableProvider, sceneLevel));
   }

   public void addInputProcessor(InputProcessor inputProcessor)
   {
      inputMultiplexer.addProcessor(inputProcessor);
   }

   /**
    * Coordinates in xy bottom left
    */
   public void setViewportBounds(int x, int y, int width, int height)
   {
      this.x = x;
      this.y = y;
      this.width = width;
      this.height = height;

      camera3D.setInputBounds(x, x + width, getCurrentWindowHeight() - y - height, getCurrentWindowHeight() - y);
   }

   public void glClearGray()
   {
      glClearGray(CLEAR_COLOR);
   }

   public void glClearGray(float color)
   {
      Gdx.gl.glClearColor(color, color, color, 1.0f);
      Gdx.gl.glClear(GL32.GL_COLOR_BUFFER_BIT | GL32.GL_DEPTH_BUFFER_BIT);
   }

   public int getCurrentWindowWidth()
   {
      return Gdx.graphics.getWidth();
   }

   public int getCurrentWindowHeight()
   {
      return Gdx.graphics.getHeight();
   }

   public FocusBasedGDXCamera getCamera3D()
   {
      return camera3D;
   }

   public ModelBatch getModelBatch()
   {
      return modelBatch;
   }

   public Environment getEnvironment()
   {
      return environment;
   }
}
