package us.ihmc.rdx.sceneManager;

import com.badlogic.gdx.graphics.Camera;
import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.Environment;
import com.badlogic.gdx.graphics.g3d.ModelBatch;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.RenderableProvider;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.DirectionalLightsAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.PointLightsAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.SpotLightsAttribute;
import com.badlogic.gdx.graphics.g3d.shaders.DepthShader;
import com.badlogic.gdx.graphics.g3d.utils.DepthShaderProvider;
import com.badlogic.gdx.math.Vector3;
import net.mgsx.gltf.scene3d.lights.DirectionalLightEx;
import net.mgsx.gltf.scene3d.lights.PointLightEx;
import net.mgsx.gltf.scene3d.scene.SceneRenderableSorter;
import net.mgsx.gltf.scene3d.shaders.PBRShaderConfig;
import net.mgsx.gltf.scene3d.shaders.PBRShaderProvider;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.rdx.lighting.RDXDirectionalLight;
import us.ihmc.rdx.lighting.RDXPointLight;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.vr.RDXVREye;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.*;

public class RDX3DScene
{
   private final HashSet<ModelInstance> modelInstances = new HashSet<>();
   private final Set<RDXRenderableAdapter> renderables = new HashSet<>();
   private final Map<Object, RDXRenderableAdapter> renderableOwnerKeyMap = new HashMap<>();

   private TreeSet<RDXSceneLevel> sceneLevelsToRender;
   private ColorAttribute ambientLight;
   private float pointLightIntensity = 660.0f;
   private float directionalLightIntensity = 5.0f;
   private ModelBatch colorModelBatch;
   private ModelBatch depthModelBatch;
   private Environment environment;
   private final PointLightsAttribute pointLights = new PointLightsAttribute();
   private final DirectionalLightsAttribute directionalLights = new DirectionalLightsAttribute();
   private final SpotLightsAttribute spotLights = new SpotLightsAttribute();

   public void create()
   {
      create(RDXSceneLevel.MODEL, RDXSceneLevel.VIRTUAL);
   }

   public void create(RDXSceneLevel... sceneLevelsToRender)
   {
      this.sceneLevelsToRender = new TreeSet<>();
      Collections.addAll(this.sceneLevelsToRender, sceneLevelsToRender);

      int maxBones = 0; // We aren't using bones
      PBRShaderConfig pbrColorShaderConfig = new PBRShaderConfig();
      pbrColorShaderConfig.numBones = maxBones;
      // pbrColorShaderConfig.numSpotLights = X  <-- Use this to enable spot lights

      DepthShader.Config depthShaderConfig = new DepthShader.Config();
      depthShaderConfig.numBones = maxBones;

      PBRShaderProvider pbrColorShader = PBRShaderProvider.createDefault(pbrColorShaderConfig);
      DepthShaderProvider pbrDepthShader = PBRShaderProvider.createDefaultDepth(depthShaderConfig);
      SceneRenderableSorter sceneRenderableSorter = new SceneRenderableSorter();

      colorModelBatch = new ModelBatch(pbrColorShader, sceneRenderableSorter);
      depthModelBatch = new ModelBatch(pbrDepthShader);

      environment = new Environment();
      float ambientLightIntensity = 0.01f;
      ambientLight = ColorAttribute.createAmbientLight(ambientLightIntensity, ambientLightIntensity, ambientLightIntensity, 1.0f);
      environment.set(ambientLight);
      environment.set(pointLights);
      environment.set(directionalLights);
      environment.set(spotLights);
   }

   public void preRender(Camera camera)
   {
      colorModelBatch.begin(camera);
   }

   public void render()
   {
      renderInternal(colorModelBatch, sceneLevelsToRender);
   }

   public void render(RDXSceneLevel exclusiveSceneLevel)
   {
      renderInternal(colorModelBatch, exclusiveSceneLevel.SINGLETON_SET);
   }

   public void render(Set<RDXSceneLevel> sceneLevels)
   {
      renderInternal(colorModelBatch, sceneLevels);
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

      colorModelBatch.begin(camera);
      renderInternal(colorModelBatch, sceneLevelsToRender);

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
      colorModelBatch.end();
   }

   public void dispose()
   {
      for (ModelInstance modelInstance : modelInstances)
      {
         ExceptionTools.handle(modelInstance.model::dispose, DefaultExceptionHandler.PRINT_MESSAGE);
      }

      colorModelBatch.dispose();
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
      addRenderableInternal(renderableAdapter);
      return renderableAdapter;
   }

   public RDXRenderableAdapter addRenderableProvider(RDXRenderableProvider renderableProvider)
   {
      RDXRenderableAdapter renderableAdapter = new RDXRenderableAdapter(renderableProvider);
      addRenderableInternal(renderableAdapter);
      return renderableAdapter;
   }

   public void addRenderableProvider(Object ownerKey, RDXRenderableProvider renderableProvider)
   {
      RDXRenderableAdapter renderableAdapter = new RDXRenderableAdapter(renderableProvider);
      renderableOwnerKeyMap.put(ownerKey, renderableAdapter);
      addRenderableInternal(renderableAdapter);
   }

   public void addRenderableProvider(Object ownerKey, RenderableProvider renderableProvider)
   {
      addRenderableProvider(ownerKey, renderableProvider, RDXSceneLevel.MODEL);
   }

   public void addRenderableProvider(Object ownerKey, RenderableProvider renderableProvider, RDXSceneLevel sceneLevel)
   {
      RDXRenderableAdapter renderableAdapter = new RDXRenderableAdapter(renderableProvider, sceneLevel);
      renderableOwnerKeyMap.put(ownerKey, renderableAdapter);
      addRenderableInternal(renderableAdapter);
   }

   public void addRenderableAdapter(RDXRenderableAdapter renderableAdapter)
   {
      addRenderableInternal(renderableAdapter);
   }

   private void addRenderableInternal(RDXRenderableAdapter renderableAdapter)
   {
      // Used to debug ClassCastException thrown by PBRShader
      // System.err.println("Adding %s@%d".formatted(renderableAdapter.getClass().getName(), renderableAdapter.hashCode()));
      // new Throwable().printStackTrace();

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
      environment.add(new DirectionalLightEx().set(Color.WHITE, new Vector3(-1.0f, -4.0f, -2.0f), directionalLightIntensity));

      Float range = null; // infinite range
      environment.add(new PointLightEx().set(Color.WHITE, new Vector3(10.0f, 10.0f, 10.0f), pointLightIntensity, range));
      environment.add(new PointLightEx().set(Color.WHITE, new Vector3(10.0f, -10.0f, 10.0f), pointLightIntensity, range));
      environment.add(new PointLightEx().set(Color.WHITE, new Vector3(-10.0f, 10.0f, 10.0f), pointLightIntensity, range));
      environment.add(new PointLightEx().set(Color.WHITE, new Vector3(-10.0f, -10.0f, 10.0f), pointLightIntensity, range));
   }

   public void clearLights()
   {
      pointLights.lights.clear();
      directionalLights.lights.clear();
   }

   public void addPointLight(RDXPointLight pointLight)
   {
      environment.add(pointLight.getPointLightEx());
   }

   public void addDirectionalLight(RDXDirectionalLight directionalLight)
   {
      environment.add(directionalLight.getDirectionalLightEx());
   }

   public void removePointLight(RDXPointLight pointLight)
   {
      pointLights.lights.removeValue(pointLight.getPointLightEx(), true);
   }

   public void removeDirectionalLight(RDXDirectionalLight directionalLight)
   {
      directionalLights.lights.removeValue(directionalLight.getDirectionalLightEx(), true);
   }

   public void setAmbientLightIntensity(float ambientLightIntensity)
   {
      ambientLight.color.r = ambientLightIntensity;
      ambientLight.color.g = ambientLightIntensity;
      ambientLight.color.b = ambientLightIntensity;
   }

   public float getAmbientLightIntensity()
   {
      return ambientLight.color.r;
   }

   public float getPointLightIntensity()
   {
      return pointLightIntensity;
   }

   public void setPointLightIntensity(float pointLightIntensity)
   {
      this.pointLightIntensity = pointLightIntensity;

      for (int i = 0; i < pointLights.lights.size; i++)
      {
         pointLights.lights.get(i).intensity = pointLightIntensity;
      }
   }

   public float getDirectionalLightIntensity()
   {
      return directionalLightIntensity;
   }

   public void setDirectionalLightIntensity(float directionalLightIntensity)
   {
      this.directionalLightIntensity = directionalLightIntensity;

      for (int i = 0; i < directionalLights.lights.size; i++)
      {
         if (directionalLights.lights.get(i) instanceof DirectionalLightEx directionalLightEx)
         {
            directionalLightEx.intensity = directionalLightIntensity;
         }
      }
   }

   public Environment getEnvironment()
   {
      return environment;
   }

   public TreeSet<RDXSceneLevel> getSceneLevelsToRender()
   {
      return sceneLevelsToRender;
   }
}