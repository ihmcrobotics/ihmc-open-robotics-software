package us.ihmc.rdx.sceneManager;

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
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.rdx.lighting.RDXDirectionalLight;
import us.ihmc.rdx.lighting.RDXPointLight;
import us.ihmc.rdx.simulation.DepthSensorShaderProvider;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.vr.RDXVREye;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.*;

public class RDX3DScene
{
   private final HashSet<ModelInstance> modelInstances = new HashSet<>();
   private final Set<RDXRenderableAdapter> renderables = new HashSet<>();
   private final Map<Object, RDXRenderableAdapter> renderableOwnerKeyMap = new HashMap<>();

   private TreeSet<RDXSceneLevel> sceneLevelsToRender;
   private float ambientLight = 0.4f;
   private ModelBatch modelBatch;
   private Environment environment;
   private final PointLightsAttribute pointLights = new PointLightsAttribute();
   private final DirectionalLightsAttribute directionalLights = new DirectionalLightsAttribute();

   public void create()
   {
      create(RDXSceneLevel.MODEL, RDXSceneLevel.VIRTUAL);
   }

   public void create(RDXSceneLevel... sceneLevelsToRender)
   {
      this.sceneLevelsToRender = new TreeSet<>();
      Collections.addAll(this.sceneLevelsToRender, sceneLevelsToRender);

      Pair<String, String> shaderStrings = LibGDXTools.loadCombinedShader(getClass().getName().replace(".", "/") + ".glsl");
      String vertexShader = shaderStrings.getLeft();
      String fragmentShader = shaderStrings.getRight();
      modelBatch = new ModelBatch(null, new DepthSensorShaderProvider(vertexShader, fragmentShader), null);
      environment = new Environment();
      environment.set(ColorAttribute.createAmbientLight(ambientLight, ambientLight, ambientLight, 1.0f));
      environment.set(pointLights);
      environment.set(directionalLights);
   }

   public void preRender(Camera camera)
   {
      modelBatch.begin(camera);
   }

   public void render()
   {
      renderInternal(modelBatch, sceneLevelsToRender);
   }

   public void render(RDXSceneLevel exclusiveSceneLevel)
   {
      renderInternal(modelBatch, exclusiveSceneLevel.SINGLETON_SET);
   }

   public void render(Set<RDXSceneLevel> sceneLevels)
   {
      renderInternal(modelBatch, sceneLevels);
   }

   // For simulated sensors in particular
   public void renderExternalBatch(ModelBatch batch, Set<RDXSceneLevel> sceneLevel)
   {
      renderInternal(batch, sceneLevel);
   }

   // For VR in particular
   public void renderToCamera(Camera camera)
   {
      if (camera instanceof RDXVREye eye)
      {
         if (eye.getSide() == RobotSide.LEFT)
         {
            sceneLevelsToRender.add(RDXSceneLevel.VR_EYE_LEFT);
         }

         if (eye.getSide() == RobotSide.RIGHT)
         {
            sceneLevelsToRender.add(RDXSceneLevel.VR_EYE_RIGHT);
         }
      }

      modelBatch.begin(camera);
      renderInternal(modelBatch, sceneLevelsToRender);

      if (camera instanceof RDXVREye eye)
      {
         if (eye.getSide() == RobotSide.LEFT)
         {
            sceneLevelsToRender.remove(RDXSceneLevel.VR_EYE_LEFT);
         }

         if (eye.getSide() == RobotSide.RIGHT)
         {
            sceneLevelsToRender.remove(RDXSceneLevel.VR_EYE_RIGHT);
         }
      }

      postRender();
   }

   private void renderInternal(ModelBatch modelBatch, Set<RDXSceneLevel> sceneLevelsToRender)
   {
      // All rendering except modelBatch.begin() and end()
      // Avoid rendering things twice
      for (RDXRenderableAdapter renderable : renderables)
      {
         renderable.setSceneLevelsToRender(sceneLevelsToRender);

         modelBatch.render(renderable, environment);
      }
   }

   public void postRender()
   {
      modelBatch.end();
   }

   public void dispose()
   {
      for (ModelInstance modelInstance : modelInstances)
      {
         ExceptionTools.handle(modelInstance.model::dispose, DefaultExceptionHandler.PRINT_MESSAGE);
      }

      modelBatch.dispose();
   }

   public RDXRenderableAdapter addModelInstance(ModelInstance modelInstance)
   {
      return addModelInstance(modelInstance, RDXSceneLevel.MODEL);
   }

   public RDXRenderableAdapter addModelInstance(ModelInstance modelInstance, RDXSceneLevel sceneLevel)
   {
      modelInstances.add(modelInstance);
      return addRenderableProvider(modelInstance, sceneLevel);
   }

   public RDXRenderableAdapter addCoordinateFrame(double size)
   {
      return addModelInstance(RDXModelBuilder.createCoordinateFrameInstance(size), RDXSceneLevel.VIRTUAL);
   }

   public RDXRenderableAdapter addRenderableProvider(RenderableProvider renderableProvider)
   {
      return addRenderableProvider(renderableProvider, RDXSceneLevel.MODEL);
   }

   public RDXRenderableAdapter addRenderableProvider(RenderableProvider renderableProvider, RDXSceneLevel sceneLevel)
   {
      RDXRenderableAdapter renderableAdapter = new RDXRenderableAdapter(renderableProvider, sceneLevel);
      renderables.add(renderableAdapter);
      return renderableAdapter;
   }

   public RDXRenderableAdapter addRenderableProvider(RDXRenderableProvider renderableProvider)
   {
      RDXRenderableAdapter renderableAdapter = new RDXRenderableAdapter(renderableProvider);
      renderables.add(renderableAdapter);
      return renderableAdapter;
   }

   public void addRenderableProvider(Object ownerKey, RDXRenderableProvider renderableProvider)
   {
      RDXRenderableAdapter renderableAdapter = new RDXRenderableAdapter(renderableProvider);
      renderableOwnerKeyMap.put(ownerKey, renderableAdapter);
      renderables.add(renderableAdapter);
   }

   public void addRenderableProvider(Object ownerKey, RenderableProvider renderableProvider)
   {
      addRenderableProvider(ownerKey, renderableProvider, RDXSceneLevel.MODEL);
   }

   public void addRenderableProvider(Object ownerKey, RenderableProvider renderableProvider, RDXSceneLevel sceneLevel)
   {
      RDXRenderableAdapter renderableAdapter = new RDXRenderableAdapter(renderableProvider, sceneLevel);
      renderableOwnerKeyMap.put(ownerKey, renderableAdapter);
      renderables.add(renderableAdapter);
   }

   public void addRenderableAdapter(RDXRenderableAdapter renderableAdapter)
   {
      renderables.add(renderableAdapter);
   }

   public void removeRenderable(Object ownerKey)
   {
      renderables.remove(renderableOwnerKeyMap.remove(ownerKey));
   }

   public void removeRenderableAdapter(RDXRenderableAdapter renderableAdapter)
   {
      renderables.remove(renderableAdapter);
   }

   public void addDefaultLighting()
   {
      setAmbientLight(0.914f);
      RDXPointLight pointLight = new RDXPointLight();
      pointLight.getPosition().set(10.0, 10.0, 10.0);
      addPointLight(pointLight);
      pointLight = new RDXPointLight();
      pointLight.getPosition().set(10.0, -10.0, 10.0);
      addPointLight(pointLight);
      pointLight = new RDXPointLight();
      pointLight.getPosition().set(-10.0, 10.0, 10.0);
      addPointLight(pointLight);
      pointLight = new RDXPointLight();
      pointLight.getPosition().set(-10.0, -10.0, 10.0);
      addPointLight(pointLight);
   }

   public void clearLights()
   {
      pointLights.lights.clear();
      directionalLights.lights.clear();
   }

   public void addPointLight(RDXPointLight pointLight)
   {
      PointLight pointLightAttribute = RDX3DSceneTools.createPointLight(pointLight.getPosition().getX32(),
                                                                        pointLight.getPosition().getY32(),
                                                                        pointLight.getPosition().getZ32());
      pointLight.setAttribute(pointLightAttribute);
      pointLights.lights.add(pointLightAttribute);
   }

   public void addDirectionalLight(RDXDirectionalLight directionalLight)
   {
      DirectionalLight directionalLightAttribute = RDX3DSceneTools.createDirectionalLight(directionalLight.getDirection().getX32(),
                                                                                          directionalLight.getDirection().getY32(),
                                                                                          directionalLight.getDirection().getZ32());
      directionalLight.setAttribute(directionalLightAttribute);
      directionalLights.lights.add(directionalLightAttribute);
   }

   public void removePointLight(RDXPointLight pointLight)
   {
      pointLights.lights.removeValue(pointLight.getAttribute(), true);
   }

   public void removeDirectionalLight(RDXDirectionalLight directionalLight)
   {
      directionalLights.lights.removeValue(directionalLight.getAttribute(), true);
   }

   public void setAmbientLight(float ambientLight)
   {
      this.ambientLight = ambientLight;
      environment.set(ColorAttribute.createAmbientLight(ambientLight, ambientLight, ambientLight, 1.0f));
   }

   public float getAmbientLight()
   {
      return ambientLight;
   }

   public TreeSet<RDXSceneLevel> getSceneLevelsToRender()
   {
      return sceneLevelsToRender;
   }
}