package us.ihmc.gdx.sceneManager;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.InputMultiplexer;
import com.badlogic.gdx.InputProcessor;
import com.badlogic.gdx.graphics.Camera;
import com.badlogic.gdx.graphics.g3d.*;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.DirectionalLightsAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.PointLightsAttribute;
import com.badlogic.gdx.graphics.g3d.environment.DirectionalLight;
import com.badlogic.gdx.graphics.g3d.environment.PointLight;
import com.badlogic.gdx.graphics.profiling.GLProfiler;
import com.badlogic.gdx.utils.viewport.ScreenViewport;
import org.lwjgl.opengl.GL41;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.gdx.FocusBasedGDXCamera;
import us.ihmc.gdx.input.GDXInputMode;
import us.ihmc.gdx.lighting.*;
import us.ihmc.gdx.tools.GDXModelPrimitives;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.log.LogTools;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;

/**
 * TODO: Pause and resume?
 */
public class GDX3DSceneManager
{
   private final HashSet<ModelInstance> modelInstances = new HashSet<>();
   private final HashMap<GDXSceneLevel, ArrayList<RenderableProvider>> renderables = new HashMap<>();
   private InputMultiplexer inputMultiplexer;
   private FocusBasedGDXCamera camera3D;
   private ScreenViewport viewport;

   private boolean shadowsEnabled = false;
   private GDXShadowManager shadowManager;
   private ModelBatch shadowsDisabledModelBatch;
   private Environment shadowsDisabledEnvironment;
   private final PointLightsAttribute shadowsDisabledPointLights = new PointLightsAttribute();
   private final DirectionalLightsAttribute shadowsDisabledDirectionalLights = new DirectionalLightsAttribute();

   private int x = 0;
   private int y = 0;
   private int width = -1;
   private int height = -1;
   private boolean firstRenderStarted = false;
   private boolean addFocusSphere = true;
   private float ambientLight = 0.4f;
   private Runnable onCreate;
   private GLProfiler glProfiler;

   // ReferenceFrame.getWorldFrame() is Z-up frame
   // Finger axis definition is right hand, Thumb +Z, Index +X, Middle +Y
   // The default orientation of the libGDX frame is such that
   // your thumb is pointing forward away from your face and your index finger pointing left
   // The default orientation of the IHMC Zup frame is such that
   // your thumb is up and your index finger is pointing away from you
   private final RigidBodyTransformReadOnly libGDXYUpToIHMCZUpSpace = new RigidBodyTransform(
         new YawPitchRoll(          // For this transformation, we start with IHMC ZUp with index forward and thumb up
            Math.toRadians(90.0),   // rotating around thumb, index goes forward to left
            Math.toRadians(0.0),    // no rotation about middle finger
            Math.toRadians(-90.0)   // rotating about index finger, thumb goes away from you
         ),
         new Point3D()
   );
   private final ReferenceFrame libGDXYUpFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("libGDXFrame",
                                                                                                                   ReferenceFrame.getWorldFrame(),
                                                                                                                   libGDXYUpToIHMCZUpSpace);

   public void create()
   {
      create(GDXInputMode.libGDX);
   }

   public void create(GDXInputMode inputMode)
   {
      if (GDXTools.ENABLE_OPENGL_DEBUGGER)
         glProfiler = GDXTools.createGLProfiler();

      GDXTools.syncLogLevelWithLogTools();

      camera3D = new FocusBasedGDXCamera(libGDXYUpFrame);
      if (inputMode == GDXInputMode.libGDX)
      {
         inputMultiplexer = new InputMultiplexer();
         Gdx.input.setInputProcessor(inputMultiplexer);

         inputMultiplexer.addProcessor(camera3D.setInputForLibGDX());
      }

      renderables.put(GDXSceneLevel.REAL_ENVIRONMENT, new ArrayList<>());
      renderables.put(GDXSceneLevel.VIRTUAL, new ArrayList<>());

      if (addFocusSphere)
         addModelInstance(camera3D.getFocusPointSphere(), GDXSceneLevel.VIRTUAL);
      viewport = new ScreenViewport(camera3D);
      viewport.setUnitsPerPixel(1.0f); // TODO: Is this relevant for high DPI displays?

      shadowsDisabledModelBatch = new ModelBatch();
      shadowsDisabledEnvironment = new Environment();
      shadowsDisabledEnvironment.set(ColorAttribute.createAmbientLight(ambientLight, ambientLight, ambientLight, 1.0f));
      shadowsDisabledEnvironment.set(shadowsDisabledPointLights);
      shadowsDisabledEnvironment.set(shadowsDisabledDirectionalLights);

      shadowManager = new GDXShadowManager(GDXImGuiBasedUI.ANTI_ALIASING, ambientLight);

      addDefaultLighting();
      if (onCreate != null)
         onCreate.run();
   }

   public void renderShadowMap()
   {
      renderShadowMap(width, height);
   }

   public void renderShadowMap(int x, int y)
   {
      if (shadowsEnabled)
      {
         shadowManager.renderShadows(camera3D, renderables.get(GDXSceneLevel.REAL_ENVIRONMENT), x, y);
      }
   }

   public void render()
   {
      preRender();
      if (shadowsEnabled)
      {
         renderInternal(shadowManager.getShadowSceneBatch(), GDXSceneLevel.REAL_ENVIRONMENT);
      }
      else
      {
         renderInternal(shadowsDisabledModelBatch, GDXSceneLevel.VIRTUAL);
      }
      postRender(GDXSceneLevel.VIRTUAL);

      if (GDXTools.ENABLE_OPENGL_DEBUGGER)
         glProfiler.reset();
   }

   // For simulated sensors
   public void renderExternalBatch(ModelBatch batch, GDXSceneLevel sceneLevel)
   {
      renderInternal(batch, sceneLevel);
   }

   // For VR
   public void renderToCamera(Camera camera)
   {
      if (shadowsEnabled)
      {
         shadowManager.preRender(camera);
         renderInternal(shadowManager.getShadowSceneBatch(), GDXSceneLevel.REAL_ENVIRONMENT);
      }
      else
      {
         shadowsDisabledModelBatch.begin(camera);
         renderInternal(shadowsDisabledModelBatch, GDXSceneLevel.VIRTUAL);
      }
      postRender(GDXSceneLevel.VIRTUAL);
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

      if (shadowsEnabled)
      {
         shadowManager.preRender(camera3D);
      }
      else
      {
         shadowsDisabledModelBatch.begin(camera3D);
      }

      GL41.glViewport(x, y, width, height);
      GDX3DSceneTools.glClearGray();
   }

   private void renderInternal(ModelBatch modelBatch, GDXSceneLevel sceneLevel)
   {
      // All rendering except modelBatch.begin() and end()

      int level = sceneLevel.ordinal();
      while (level >= 0)
      {
         GDXSceneLevel levelToRender = GDXSceneLevel.values()[level];
         renderInternal(modelBatch, renderables.get(levelToRender));
         --level;
      }
   }

   private void renderInternal(ModelBatch modelBatch, Iterable<RenderableProvider> renderables)
   {
      for (RenderableProvider renderable : renderables)
      {
         if (shadowsEnabled)
            modelBatch.render(renderable);
         else
            modelBatch.render(renderable, shadowsDisabledEnvironment);
      }
   }

   private void postRender(GDXSceneLevel sceneLevel)
   {
      if (shadowsEnabled)
      {
         shadowManager.postRender();
      }
      else
      {
         shadowsDisabledModelBatch.end();
      }

      if (shadowsEnabled && sceneLevel == GDXSceneLevel.VIRTUAL)
      {
         // Render all virtual objects using the primary model batch
         shadowsDisabledModelBatch.begin(camera3D);
         shadowsDisabledModelBatch.render(renderables.get(GDXSceneLevel.VIRTUAL));
         shadowsDisabledModelBatch.end();
      }
   }

   public void dispose()
   {
      for (ModelInstance modelInstance : modelInstances)
      {
         ExceptionTools.handle(modelInstance.model::dispose, DefaultExceptionHandler.PRINT_MESSAGE);
      }

      ExceptionTools.handle(() -> camera3D.dispose(), DefaultExceptionHandler.PRINT_MESSAGE);
      shadowManager.dispose();
      shadowsDisabledModelBatch.dispose();
   }
   // End render public API

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
      renderables.get(sceneLevel).add(renderableProvider);
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

      shadowManager.setViewportBounds(x, y, width, height);
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

   public void setAddFocusSphere(boolean addFocusSphere)
   {
      this.addFocusSphere = addFocusSphere;
   }

   public void setShadowsEnabled(boolean shadowsEnabled)
   {
      this.shadowsEnabled = shadowsEnabled;
   }

   public void clearLights()
   {
      shadowsDisabledPointLights.lights.clear();
      shadowsDisabledDirectionalLights.lights.clear();
      shadowManager.getPointLights().clear();
      shadowManager.getDirectionalLights().clear();
   }

   public void setOnCreate(Runnable onCreate)
   {
      this.onCreate = onCreate;
   }

   public void addDefaultLighting()
   {
      setAmbientLight(0.914f);
      GDXPointLight pointLight = new GDXPointLight();
      pointLight.getPosition().set(10.0, 10.0, 10.0);
      addPointLight(pointLight);
      pointLight = new GDXPointLight();
      pointLight.getPosition().set(10.0, -10.0, 10.0);
      addPointLight(pointLight);
      pointLight = new GDXPointLight();
      pointLight.getPosition().set(-10.0, 10.0, 10.0);
      addPointLight(pointLight);
      pointLight = new GDXPointLight();
      pointLight.getPosition().set(-10.0, -10.0, 10.0);
      addPointLight(pointLight);
   }

   public void addPointLight(GDXPointLight pointLight)
   {
      PointLight pointLightAttribute = GDX3DSceneTools.createPointLight(pointLight.getPosition().getX32(),
                                                                        pointLight.getPosition().getY32(),
                                                                        pointLight.getPosition().getZ32());
      pointLight.setAttribute(pointLightAttribute);
      shadowsDisabledPointLights.lights.add(pointLightAttribute);
      shadowManager.getPointLights().add(pointLight);
   }

   public void addDirectionalLight(GDXDirectionalLight directionalLight)
   {
      DirectionalLight directionalLightAttribute = GDX3DSceneTools.createDirectionalLight(directionalLight.getDirection().getX32(),
                                                                                          directionalLight.getDirection().getY32(),
                                                                                          directionalLight.getDirection().getZ32());
      directionalLight.setAttribute(directionalLightAttribute);
      shadowsDisabledDirectionalLights.lights.add(directionalLightAttribute);
      shadowManager.getDirectionalLights().add(directionalLight);
   }

   public void removePointLight(GDXPointLight pointLight)
   {
      shadowsDisabledPointLights.lights.removeValue(pointLight.getAttribute(), true);
      shadowManager.getPointLights().remove(pointLight);
   }

   public void removeDirectionalLight(GDXDirectionalLight directionalLight)
   {
      shadowsDisabledDirectionalLights.lights.removeValue(directionalLight.getAttribute(), true);
      shadowManager.getDirectionalLights().remove(directionalLight);
   }

   public float getAmbientLight()
   {
      return ambientLight;
   }

   public void setAmbientLight(float ambientLight)
   {
      this.ambientLight = ambientLight;
      shadowsDisabledEnvironment.set(ColorAttribute.createAmbientLight(ambientLight, ambientLight, ambientLight, 1.0f));
      shadowManager.setAmbientLight(ambientLight);
   }
}
