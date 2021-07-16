package us.ihmc.gdx.sceneManager;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.InputMultiplexer;
import com.badlogic.gdx.InputProcessor;
import com.badlogic.gdx.graphics.Camera;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.GL30;
import com.badlogic.gdx.graphics.g3d.*;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.environment.DirectionalLight;
import com.badlogic.gdx.graphics.g3d.environment.DirectionalShadowLight;
import com.badlogic.gdx.graphics.g3d.environment.PointLight;
import com.badlogic.gdx.graphics.g3d.utils.DepthShaderProvider;
import com.badlogic.gdx.graphics.profiling.GLProfiler;
import com.badlogic.gdx.math.Vector3;
import com.badlogic.gdx.tests.g3d.shadows.system.ShadowSystem;
import com.badlogic.gdx.tests.g3d.shadows.system.classical.ClassicalShadowSystem;
import com.badlogic.gdx.tests.g3d.shadows.utils.AABBNearFarAnalyzer;
import com.badlogic.gdx.tests.g3d.shadows.utils.BoundingSphereDirectionalAnalyzer;
import com.badlogic.gdx.tests.g3d.shadows.utils.FixedShadowMapAllocator;
import com.badlogic.gdx.tests.g3d.shadows.utils.FrustumLightFilter;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.BufferUtils;
import com.badlogic.gdx.utils.viewport.ScreenViewport;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.gdx.FocusBasedGDXCamera;
import us.ihmc.gdx.input.GDXInputMode;
import us.ihmc.gdx.tools.GDXModelPrimitives;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.log.LogTools;

import java.nio.IntBuffer;
import java.util.HashSet;

/**
 * TODO: Pause and resume?
 */
public class GDX3DSceneManager
{
   private InputMultiplexer inputMultiplexer;
   private FocusBasedGDXCamera camera3D;
   private Environment environment;
   private ScreenViewport viewport;
   private ModelBatch modelBatch;

   private ShadowSystem shadowSystem;
   private Array<ModelBatch> shadowPassBatches = new Array<ModelBatch>();

   private int x = 0;
   private int y = 0;
   private int width = -1;
   private int height = -1;

   private final HashSet<ModelInstance> modelInstances = new HashSet<>();
   private final HashSet<GDXRenderable> renderables = new HashSet<>();

   private boolean firstRenderStarted = false;
   private boolean addFocusSphere = true;

   public void create()
   {
      create(GDXInputMode.libGDX);
   }

   public void create(GDXInputMode inputMode)
   {
      new GLProfiler(Gdx.graphics).enable();
      GDXTools.syncLogLevelWithLogTools();

      camera3D = new FocusBasedGDXCamera();
      if (inputMode == GDXInputMode.libGDX)
      {
         inputMultiplexer = new InputMultiplexer();
         Gdx.input.setInputProcessor(inputMultiplexer);

         inputMultiplexer.addProcessor(camera3D.setInputForLibGDX());
      }

      if (addFocusSphere)
         addModelInstance(camera3D.getFocusPointSphere(), GDXSceneLevel.VIRTUAL);
      viewport = new ScreenViewport(camera3D);
      viewport.setUnitsPerPixel(1.0f); // TODO: Is this relevant for high DPI displays?

      environment = new Environment();
      environment.set(new ColorAttribute(ColorAttribute.AmbientLight, 1, 1, 1, 0.2f));

      PointLight light = new PointLight();
      light.set(1, 1, 1, 0, 0, 20, 100);
      environment.add(light);

      shadowSystem = new ClassicalShadowSystem(new AABBNearFarAnalyzer(), new FixedShadowMapAllocator(2048, 4),
                                               new BoundingSphereDirectionalAnalyzer(), new FrustumLightFilter());
      shadowSystem.init();
      shadowSystem.addLight(light);

      for (int i = 0; i < shadowSystem.getPassQuantity(); i++) {
         shadowPassBatches.add(new ModelBatch(shadowSystem.getPassShaderProvider(i)));
      }

      modelBatch = new ModelBatch(shadowSystem.getShaderProvider());
   }

   private void preRender()
   {
      preRender(GDXSceneLevel.VIRTUAL);
   }

   private void preRender(GDXSceneLevel sceneLevel)
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

      IntBuffer buffer = BufferUtils.newIntBuffer(1);
      Gdx.gl.glGetIntegerv(GL30.GL_DRAW_FRAMEBUFFER_BINDING, buffer);
      int bufferID = buffer.get();

      shadowSystem.begin(camera3D, renderables);
      shadowSystem.update();

      for (int i = 0; i < shadowSystem.getPassQuantity(); i++) {
         shadowSystem.begin(i);
         Camera camera;
         while ((camera = shadowSystem.next()) != null) {
            ModelBatch passBatch = shadowPassBatches.get(i);

            passBatch.begin(camera);
            passBatch.render(renderables, environment);
            passBatch.end();
         }
         shadowSystem.end(i);
      }

      shadowSystem.end();

      Gdx.gl.glBindFramebuffer(GL30.GL_DRAW_FRAMEBUFFER, bufferID); //manually bind framebuffer here even though it shouldn't be necessary because shadows mess it up somehow

      viewport.update(width, height);
      viewport.apply();

      GDX3DSceneTools.glClearGray();

      modelBatch.begin(camera3D);
   }

   private void renderInternal(ModelBatch modelBatch)
   {
      renderInternal(modelBatch, GDXSceneLevel.VIRTUAL);
   }

   private void renderInternal(ModelBatch modelBatch, GDXSceneLevel sceneLevel)
   {
      //All rendering except modelBatch.begin() and end()

      for (GDXRenderable renderable : renderables)
      {
         if (sceneLevel.ordinal() >= renderable.getSceneType().ordinal())
            modelBatch.render(renderable.getRenderableProvider(), environment);
      }
   }

   private void postRender()
   {
      modelBatch.end(); //this is actually where all the rendering happens despite the function name
   }

   //Render public API
   public void renderToCamera(Camera camera)
   {
      modelBatch.begin(camera);
      renderInternal(modelBatch);
      postRender();
   }

   public void render()
   {
      render(GDXSceneLevel.VIRTUAL);
   }

   public void render(GDXSceneLevel sceneLevel)
   {
      preRender(sceneLevel);
      renderInternal(modelBatch);
      postRender();
   }

   public void renderFromBatch(ModelBatch batch, GDXSceneLevel sceneLevel) {
      renderInternal(batch, sceneLevel);
   }
   //End render public API

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

   public void setViewportBoundsToWindow()
   {
      setViewportBounds(0, 0, Gdx.graphics.getWidth(), Gdx.graphics.getHeight());
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

   public void addLibGDXInputProcessor(InputProcessor inputProcessor)
   {
      if (inputMultiplexer != null)
      {
         inputMultiplexer.addProcessor(inputProcessor);
      }
      else
      {
         LogTools.error(1, "libGDX is not being used for input!");
      }
   }

   public Environment getEnvironment()
   {
      return environment;
   }

   public void setAddFocusSphere(boolean addFocusSphere)
   {
      this.addFocusSphere = addFocusSphere;
   }
}
