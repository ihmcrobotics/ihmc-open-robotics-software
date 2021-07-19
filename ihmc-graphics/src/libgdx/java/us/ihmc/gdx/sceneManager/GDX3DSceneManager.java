package us.ihmc.gdx.sceneManager;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.InputMultiplexer;
import com.badlogic.gdx.InputProcessor;
import com.badlogic.gdx.graphics.*;
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
   private Array<ModelBatch> passBatches = new Array<ModelBatch>();

   private int x = 0;
   private int y = 0;
   private int width = -1;
   private int height = -1;

   private final HashSet<ModelInstance> modelInstances = new HashSet<>();
   private final HashSet<GDXRenderable> renderables = new HashSet<>();

   private boolean firstRenderStarted = false;
   private boolean addFocusSphere = true;

   private static int getFramebufferID()
   {
      IntBuffer buffer = BufferUtils.newIntBuffer(1);
      Gdx.gl.glGetIntegerv(GL30.GL_DRAW_FRAMEBUFFER_BINDING, buffer);
      return buffer.get();
   }

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

      shadowSystem = new ClassicalShadowSystem(new AABBNearFarAnalyzer(), new FixedShadowMapAllocator(2048, 4), new BoundingSphereDirectionalAnalyzer(), new FrustumLightFilter());

      // Environment setup
      environment = new Environment();
      environment.set(new ColorAttribute(ColorAttribute.AmbientLight, 0.4f, 0.4f, 0.4f, 1.0f));

      PointLight light = new PointLight();
      light.set(1, 1, 1, 0, 0, 20, 0);
      environment.add(light);
      shadowSystem.addLight(light);

      // ShadowSystem init must come after lights are added
      shadowSystem.init();
      for (int i = 0; i < shadowSystem.getPassQuantity(); i++) {
         passBatches.add(new ModelBatch(shadowSystem.getPassShaderProvider(i)));
      }

      modelBatch = new ModelBatch(shadowSystem.getShaderProvider());
   }

   public void renderShadowMap()
   {
      shadowSystem.begin(camera3D, renderables); //maybe change
      shadowSystem.update();
      for (int i = 0; i < shadowSystem.getPassQuantity(); i++) {
         shadowSystem.begin(i);
         Camera camera;
         while ((camera = shadowSystem.next()) != null) {
            passBatches.get(i).begin(camera);
            renderFromBatch(passBatches.get(i), GDXSceneLevel.REAL_ENVIRONMENT);
            passBatches.get(i).end();
         }
         shadowSystem.end(i);
      }
      shadowSystem.end();
   }

   private void preRender()
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
      GDX3DSceneTools.glClearGray();
   }

   private void renderInternal(ModelBatch modelBatch)
   {
      renderInternal(modelBatch, GDXSceneLevel.VIRTUAL);
   }

   private void renderInternal(ModelBatch modelBatch, GDXSceneLevel sceneLevel)
   {
      // All rendering except modelBatch.begin() and end()

      for (GDXRenderable renderable : renderables)
      {
         if (sceneLevel.ordinal() >= renderable.getSceneType().ordinal())
            modelBatch.render(renderable.getRenderableProvider(), environment);
      }
   }

   private void postRender()
   {
      modelBatch.end(); // This is actually where all the rendering happens despite the method name
   }

   // Render public API
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
      preRender();
      renderInternal(modelBatch, sceneLevel);
      postRender();
   }

   public void renderFromBatch(ModelBatch batch, GDXSceneLevel sceneLevel) {
      renderInternal(batch, sceneLevel);
   }
   // End render public API

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
