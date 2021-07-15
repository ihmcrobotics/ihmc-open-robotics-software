package us.ihmc.gdx.sceneManager;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.InputMultiplexer;
import com.badlogic.gdx.InputProcessor;
import com.badlogic.gdx.graphics.Camera;
import com.badlogic.gdx.graphics.g3d.*;
import com.badlogic.gdx.graphics.g3d.attributes.DirectionalLightsAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.PointLightsAttribute;
import com.badlogic.gdx.graphics.g3d.environment.DirectionalLight;
import com.badlogic.gdx.graphics.g3d.environment.PointLight;
import com.badlogic.gdx.graphics.g3d.shaders.DefaultShader;
import com.badlogic.gdx.graphics.g3d.utils.DefaultShaderProvider;
import com.badlogic.gdx.graphics.profiling.GLProfiler;
import com.badlogic.gdx.tests.g3d.shadows.system.ShadowSystem;
import com.badlogic.gdx.tests.g3d.shadows.system.classical.ClassicalShadowSystem;
import com.badlogic.gdx.tests.g3d.shadows.utils.AABBNearFarAnalyzer;
import com.badlogic.gdx.tests.g3d.shadows.utils.BoundingSphereDirectionalAnalyzer;
import com.badlogic.gdx.tests.g3d.shadows.utils.FixedShadowMapAllocator;
import com.badlogic.gdx.tests.g3d.shadows.utils.FrustumLightFilter;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.viewport.ScreenViewport;
import org.lwjgl.opengl.GL20;
import org.lwjgl.opengl.GL32;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.gdx.FocusBasedGDXCamera;
import us.ihmc.gdx.input.GDXInputMode;
import us.ihmc.gdx.tools.GDXModelPrimitives;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.log.LogTools;

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

   private ShadowSystem shadowSystem;
   private final Array<ModelBatch> passBatches = new Array<ModelBatch>();
   private ModelBatch modelBatch;

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

      this.environment = GDX3DSceneTools.createDefaultEnvironment();

      DefaultShader.Config defaultShaderConfig = new DefaultShader.Config();
      // we could set shader options or even swap out the shader here
      modelBatch = new ModelBatch(new DefaultShaderProvider(defaultShaderConfig));

      this.shadowSystem = new ClassicalShadowSystem(new AABBNearFarAnalyzer(), new FixedShadowMapAllocator(2048, 4),
                                                    new BoundingSphereDirectionalAnalyzer(), new FrustumLightFilter());
      shadowSystem.init();
      for (int i = 0; i < shadowSystem.getPassQuantity(); i++) {
         passBatches.add(new ModelBatch(shadowSystem.getPassShaderProvider(i)));
      }

      //modelBatch = new ModelBatch(shadowSystem.getShaderProvider());

      for (Attribute attribute : environment) {
         if (attribute instanceof PointLightsAttribute) {
            PointLightsAttribute pointLights = (PointLightsAttribute) attribute;
            for (PointLight light : pointLights.lights) {
               shadowSystem.addLight(light);
            }
         } else if (attribute instanceof DirectionalLightsAttribute) {
            DirectionalLightsAttribute directionalLights = (DirectionalLightsAttribute) attribute;
            for (DirectionalLight light : directionalLights.lights) {
               shadowSystem.addLight(light);
            }
         }
      }

      if (inputMode == GDXInputMode.libGDX)
      {
         inputMultiplexer = new InputMultiplexer();
         Gdx.input.setInputProcessor(inputMultiplexer);
      }

      camera3D = new FocusBasedGDXCamera();
      if (inputMode == GDXInputMode.libGDX)
      {
         inputMultiplexer.addProcessor(camera3D.setInputForLibGDX());
      }

      if (addFocusSphere)
         addModelInstance(camera3D.getFocusPointSphere(), GDXSceneLevel.VIRTUAL);
      viewport = new ScreenViewport(camera3D);
      viewport.setUnitsPerPixel(1.0f); // TODO: Is this relevant for high DPI displays?

      GDX3DSceneTools.glClearGray();
      Gdx.gl.glEnable(GL32.GL_TEXTURE_2D);
   }

   public void renderBefore()
   {
      renderBefore(GDXSceneLevel.VIRTUAL);
   }

   public void renderBefore(GDXSceneLevel sceneLevel)
   {
      sceneLevel = GDXSceneLevel.REAL_ENVIRONMENT;

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

      shadowSystem.begin(camera3D, modelInstances);
      shadowSystem.update();

      for (int i = 0; i < shadowSystem.getPassQuantity(); i++) {
         shadowSystem.begin(i);
         Camera camera;
         while ((camera = shadowSystem.next()) != null) {
            passBatches.get(i).begin(camera);
            passBatches.get(i).render(modelInstances, environment);
            passBatches.get(i).end();
         }
         shadowSystem.end(i);
      }

      shadowSystem.end();

      Gdx.gl.glViewport(x, y, width, height);
      GDX3DSceneTools.glClearGray(0);

      modelBatch.begin(camera3D);

      renderRegisteredObjectsWithEnvironment(modelBatch, sceneLevel);
   }

   public void renderRegisteredObjectsWithEnvironment(ModelBatch modelBatch)
   {
      renderRegisteredObjectsWithEnvironment(modelBatch, GDXSceneLevel.VIRTUAL);
   }

   public void renderRegisteredObjectsWithEnvironment(ModelBatch modelBatch, GDXSceneLevel sceneLevel)
   {
      //environment = GDX3DSceneTools.createDefaultEnvironment();

      for (GDXRenderable renderable : renderables)
      {
         modelBatch.render(renderable.getRenderableProvider(), environment);

//         if (sceneLevel.ordinal() >= renderable.getSceneType().ordinal())
//         {
//            modelBatch.render(renderable.getRenderableProvider(), environment);
//         }
      }
   }

   public void renderAfter()
   {
      modelBatch.end();
   }

   public void renderToCamera(Camera camera)
   {
      //modelBatch.begin(camera);
      renderBefore();
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

      for (ModelBatch batch : passBatches) {
         batch.dispose();
      }

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

   public ModelBatch getModelBatch()
   {
      return modelBatch;
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
