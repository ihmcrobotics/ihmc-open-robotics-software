package us.ihmc.gdx.sceneManager;

import com.badlogic.gdx.graphics.Camera;
import com.badlogic.gdx.graphics.g3d.Environment;
import com.badlogic.gdx.graphics.g3d.ModelBatch;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.DirectionalLightsAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.PointLightsAttribute;
import com.badlogic.gdx.graphics.g3d.environment.DirectionalLight;
import com.badlogic.gdx.graphics.g3d.environment.PointLight;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.gdx.lighting.GDXDirectionalLight;
import us.ihmc.gdx.lighting.GDXPointLight;
import us.ihmc.gdx.lighting.GDXShadowManager;
import us.ihmc.gdx.tools.GDXModelBuilder;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;

import java.util.*;

public class GDX3DScene
{
   public static final Set<GDXSceneLevel> REAL_ENVIRONMENT_ONLY = Collections.singleton(GDXSceneLevel.REAL_ENVIRONMENT);

   private final HashSet<ModelInstance> modelInstances = new HashSet<>();
   private final HashMap<GDXSceneLevel, ArrayList<RenderableProvider>> renderables = new HashMap<>();
   private final HashSet<RenderableProvider> uniqueRenderables = new HashSet<>();

   private TreeSet<GDXSceneLevel> sceneLevelsToRender;
   private float ambientLight = 0.4f;
   private boolean shadowsEnabled = false;
   private GDXShadowManager shadowManager;
   private ModelBatch shadowsDisabledModelBatch;
   private Environment shadowsDisabledEnvironment;
   private final PointLightsAttribute shadowsDisabledPointLights = new PointLightsAttribute();
   private final DirectionalLightsAttribute shadowsDisabledDirectionalLights = new DirectionalLightsAttribute();

   public void create()
   {
      create(GDXSceneLevel.REAL_ENVIRONMENT, GDXSceneLevel.VIRTUAL);
   }

   public void create(GDXSceneLevel... sceneLevelsToRender)
   {
      this.sceneLevelsToRender = new TreeSet<>();
      Collections.addAll(this.sceneLevelsToRender, sceneLevelsToRender);

      renderables.put(GDXSceneLevel.REAL_ENVIRONMENT, new ArrayList<>());
      renderables.put(GDXSceneLevel.VIRTUAL, new ArrayList<>());

      shadowsDisabledModelBatch = new ModelBatch();
      shadowsDisabledEnvironment = new Environment();
      shadowsDisabledEnvironment.set(ColorAttribute.createAmbientLight(ambientLight, ambientLight, ambientLight, 1.0f));
      shadowsDisabledEnvironment.set(shadowsDisabledPointLights);
      shadowsDisabledEnvironment.set(shadowsDisabledDirectionalLights);

      shadowManager = new GDXShadowManager(GDXImGuiBasedUI.ANTI_ALIASING, ambientLight);
   }

   public void preRender(Camera camera)
   {
      if (shadowsEnabled)
      {
         shadowManager.preRender(camera);
      }
      else
      {
         shadowsDisabledModelBatch.begin(camera);
      }
   }

   public void render()
   {
      if (shadowsEnabled)
      {
         renderInternal(shadowManager.getShadowSceneBatch(), REAL_ENVIRONMENT_ONLY);
      }
      else
      {
         renderInternal(shadowsDisabledModelBatch, sceneLevelsToRender);
      }
   }

   // For testing shadows in particular
   public void renderShadowMap(Camera camera, int x, int y)
   {
      if (shadowsEnabled)
      {
         shadowManager.renderShadows(camera, renderables.get(GDXSceneLevel.REAL_ENVIRONMENT), x, y);
      }
   }

   // For simulated sensors in particular
   public void renderExternalBatch(ModelBatch batch, Set<GDXSceneLevel> sceneLevel)
   {
      renderInternal(batch, sceneLevel);
   }

   // For VR in particular
   public void renderToCamera(Camera camera)
   {
      if (shadowsEnabled)
      {
         shadowManager.preRender(camera);
         renderInternal(shadowManager.getShadowSceneBatch(), REAL_ENVIRONMENT_ONLY);
      }
      else
      {
         shadowsDisabledModelBatch.begin(camera);
         renderInternal(shadowsDisabledModelBatch, sceneLevelsToRender);
      }
      postRender(camera, GDXSceneLevel.VIRTUAL);
   }

   private void renderInternal(ModelBatch modelBatch, Set<GDXSceneLevel> sceneLevels)
   {
      // All rendering except modelBatch.begin() and end()
      // Avoid rendering things twice
      uniqueRenderables.clear();
      for (GDXSceneLevel sceneLevel : sceneLevels)
      {
         uniqueRenderables.addAll(renderables.get(sceneLevel));
      }

      renderInternal(modelBatch, uniqueRenderables);
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

   public void postRender(Camera camera, GDXSceneLevel sceneLevel)
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
         shadowsDisabledModelBatch.begin(camera);
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

      shadowManager.dispose();
      shadowsDisabledModelBatch.dispose();
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
      addModelInstance(GDXModelBuilder.createCoordinateFrameInstance(size), GDXSceneLevel.VIRTUAL);
   }

   public void addRenderableProvider(RenderableProvider renderableProvider)
   {
      addRenderableProvider(renderableProvider, GDXSceneLevel.REAL_ENVIRONMENT);
   }

   public void addRenderableProvider(RenderableProvider renderableProvider, GDXSceneLevel sceneLevel)
   {
      renderables.get(sceneLevel).add(renderableProvider);
   }

   public void removeRenderableProvider(RenderableProvider renderableProvider, GDXSceneLevel sceneLevel)
   {
      renderables.get(sceneLevel).remove(renderableProvider);
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

   public void clearLights()
   {
      shadowsDisabledPointLights.lights.clear();
      shadowsDisabledDirectionalLights.lights.clear();
      shadowManager.getPointLights().clear();
      shadowManager.getDirectionalLights().clear();
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

   public void setAmbientLight(float ambientLight)
   {
      this.ambientLight = ambientLight;
      shadowsDisabledEnvironment.set(ColorAttribute.createAmbientLight(ambientLight, ambientLight, ambientLight, 1.0f));
      shadowManager.setAmbientLight(ambientLight);
   }

   public void setShadowsEnabled(boolean shadowsEnabled)
   {
      this.shadowsEnabled = shadowsEnabled;
   }

   public float getAmbientLight()
   {
      return ambientLight;
   }

   public GDXShadowManager getShadowManager()
   {
      return shadowManager;
   }

   public TreeSet<GDXSceneLevel> getSceneLevelsToRender()
   {
      return sceneLevelsToRender;
   }
}
