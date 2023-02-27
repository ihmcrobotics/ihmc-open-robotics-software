package us.ihmc.rdx.sceneManager;

import com.badlogic.gdx.graphics.g3d.Environment;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.DirectionalLightsAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.PointLightsAttribute;
import com.badlogic.gdx.graphics.g3d.environment.DirectionalLight;
import com.badlogic.gdx.graphics.g3d.environment.DirectionalShadowLight;
import com.badlogic.gdx.graphics.g3d.environment.PointLight;
import com.badlogic.gdx.utils.BufferUtils;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImFloat;
import org.lwjgl.opengl.GL41;

import java.nio.IntBuffer;

public class RDX3DSceneTools
{
   public static final float CLEAR_COLOR = 0.5019608f;
   public static final String TUNING_WINDOW_NAME = "Lighting";

   private static final ImBoolean ambientEnabled = new ImBoolean(true);
   private static final ImFloat ambientColor = new ImFloat(1.0f);
   private static final ImBoolean pointEnabled = new ImBoolean(true);
   private static final ImFloat pointColor = new ImFloat(1.0f);
   private static final ImFloat pointDistance = new ImFloat(10.0f);
   private static final ImFloat pointIntensity = new ImFloat(43.280f);
   private static final ImBoolean directionEnabled = new ImBoolean(false);
   private static final ImFloat directionColor = new ImFloat(0.025f);
   private static final ImFloat directionDistance = new ImFloat(20.0f);

   public static void glClearGray()
   {
      glClearGray(CLEAR_COLOR);
   }

   public static void glClearGray(float color)
   {
      GL41.glClearColor(color, color, color, 1.0f);
      GL41.glClear(GL41.GL_COLOR_BUFFER_BIT | GL41.GL_DEPTH_BUFFER_BIT);
   }

   public static int getFramebufferID()
   {
      IntBuffer buffer = BufferUtils.newIntBuffer(1);
      GL41.glGetIntegerv(GL41.GL_DRAW_FRAMEBUFFER_BINDING, buffer);
      return buffer.get();
   }

   public static Environment createDefaultEnvironment()
   {
      Environment environment = new Environment();
      float ambientColor = RDX3DSceneTools.ambientColor.get();
      if (ambientEnabled.get())
      {
         environment.set(new ColorAttribute(ColorAttribute.AmbientLight, ambientColor, ambientColor, ambientColor, 1.0f));
      }
      float pointColor = RDX3DSceneTools.pointColor.get();
      float pointDistance = RDX3DSceneTools.pointDistance.get();
      float pointIntensity = RDX3DSceneTools.pointIntensity.get();
      if (pointEnabled.get())
      {
         PointLightsAttribute pointLights = new PointLightsAttribute();
         pointLights.lights.add(new PointLight().set(pointColor, pointColor, pointColor, pointDistance, pointDistance, pointDistance, pointIntensity));
         pointLights.lights.add(new PointLight().set(pointColor, pointColor, pointColor, -pointDistance, pointDistance, pointDistance, pointIntensity));
         pointLights.lights.add(new PointLight().set(pointColor, pointColor, pointColor, -pointDistance, -pointDistance, pointDistance, pointIntensity));
         pointLights.lights.add(new PointLight().set(pointColor, pointColor, pointColor, pointDistance, -pointDistance, pointDistance, pointIntensity));
         environment.set(pointLights);
      }
      float directionColor = RDX3DSceneTools.directionColor.get();
      float directionDistance = RDX3DSceneTools.directionDistance.get();
      if (directionEnabled.get())
      {
         DirectionalLightsAttribute directionalLights = new DirectionalLightsAttribute();
         directionalLights.lights.add(new DirectionalLight().set(directionColor, directionColor, directionColor, -directionDistance, -directionDistance, -directionDistance));
         directionalLights.lights.add(new DirectionalLight().set(directionColor, directionColor, directionColor,  directionDistance, -directionDistance, -directionDistance));
         directionalLights.lights.add(new DirectionalLight().set(directionColor, directionColor, directionColor,  directionDistance,  directionDistance, -directionDistance));
         directionalLights.lights.add(new DirectionalLight().set(directionColor, directionColor, directionColor, -directionDistance,  directionDistance, -directionDistance));
         environment.set(directionalLights);
      }
      //      DirectionalLightsAttribute directionalLights = new DirectionalLightsAttribute();
      //      directionalLights.lights.add(newShadowLight().set(pointColor, pointColor, pointColor, -pointDistance, -pointDistance, -pointDistance));
      //      directionalLights.lights.add(newShadowLight().set(pointColor, pointColor, pointColor, pointDistance, -pointDistance, -pointDistance));
      //      directionalLights.lights.add(newShadowLight().set(pointColor, pointColor, pointColor, pointDistance, pointDistance, -pointDistance));
      //      directionalLights.lights.add(newShadowLight().set(pointColor, pointColor, pointColor, -pointDistance, pointDistance, -pointDistance));
      //      environment.set(directionalLights);
      return environment;
   }

   public static PointLight createPointLight(float x, float y, float z)
   {
      float pointColor = RDX3DSceneTools.pointColor.get();
      float pointIntensity = RDX3DSceneTools.pointIntensity.get();
      return new PointLight().set(pointColor, pointColor, pointColor, x, y, z, pointIntensity);
   }

   public static DirectionalLight createDirectionalLight(float x, float y, float z)
   {
      float directionColor = RDX3DSceneTools.directionColor.get();
      return new DirectionalLight().set(directionColor, directionColor, directionColor, x, y, z);
   }

   private static DirectionalShadowLight newShadowLight()
   {
      return new DirectionalShadowLight(1024, 1024, 30f, 30f, 1f, 100f);
   }

   public static void renderTuningSliders()
   {
      ImGui.checkbox("Ambient enabled", ambientEnabled);
      ImGui.sliderFloat("Ambient color", ambientColor.getData(), 0.0f, 1.0f);
      ImGui.checkbox("Point enabled", pointEnabled);
      ImGui.sliderFloat("Point color", pointColor.getData(), 0.0f, 1.0f);
      ImGui.sliderFloat("Point distance", pointDistance.getData(), 0.0f, 500.0f);
      ImGui.sliderFloat("Point intensity", pointIntensity.getData(), 0.0f, 500.0f);
      ImGui.checkbox("Direction enabled", directionEnabled);
      ImGui.sliderFloat("Direction color", directionColor.getData(), 0.0f, 1.0f);
      ImGui.sliderFloat("Direction distance", directionDistance.getData(), 0.0f, 20.0f);
   }
}
