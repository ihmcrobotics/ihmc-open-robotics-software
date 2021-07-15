package us.ihmc.gdx.sceneManager;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.g3d.Environment;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.DirectionalLightsAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.PointLightsAttribute;
import com.badlogic.gdx.graphics.g3d.environment.DirectionalLight;
import com.badlogic.gdx.graphics.g3d.environment.DirectionalShadowLight;
import com.badlogic.gdx.graphics.g3d.environment.PointLight;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImFloat;
import org.lwjgl.opengl.GL32;

public class GDX3DSceneTools
{
   public static final float CLEAR_COLOR = 0.5019608f;
   public static final String TUNING_WINDOW_NAME = "Lighting";

   private static ImBoolean ambientEnabled = new ImBoolean(true);
   private static ImFloat ambientColor = new ImFloat(1.0f);
   private static ImBoolean pointEnabled = new ImBoolean(true);
   private static ImFloat pointColor = new ImFloat(1.0f);
   private static ImFloat pointDistance = new ImFloat(10.0f);
   private static ImFloat pointIntensity = new ImFloat(43.280f);
   private static ImBoolean directionEnabled = new ImBoolean(false);
   private static ImFloat directionColor = new ImFloat(0.025f);
   private static ImFloat directionDistance = new ImFloat(20.0f);

   public static void glClearGray()
   {
      glClearGray(CLEAR_COLOR);
   }

   public static void glClearGray(float color)
   {
      Gdx.gl.glClearColor(color, color, color, 1.0f);
      Gdx.gl.glClear(GL32.GL_COLOR_BUFFER_BIT | GL32.GL_DEPTH_BUFFER_BIT);
   }

   public static Environment createDefaultEnvironment()
   {
      Environment environment = new Environment();
      float ambientColor = GDX3DSceneTools.ambientColor.get();
      if (ambientEnabled.get())
      {
         environment.set(new ColorAttribute(ColorAttribute.AmbientLight, ambientColor, ambientColor, ambientColor, 1.0f));
      }
      float pointColor = GDX3DSceneTools.pointColor.get();
      float pointDistance = GDX3DSceneTools.pointDistance.get();
      float pointIntensity = GDX3DSceneTools.pointIntensity.get();
      if (pointEnabled.get())
      {
         PointLightsAttribute pointLights = new PointLightsAttribute();
         pointLights.lights.add(new PointLight().set(pointColor, pointColor, pointColor, pointDistance, pointDistance, pointDistance, pointIntensity));
         pointLights.lights.add(new PointLight().set(pointColor, pointColor, pointColor, -pointDistance, pointDistance, pointDistance, pointIntensity));
         pointLights.lights.add(new PointLight().set(pointColor, pointColor, pointColor, -pointDistance, -pointDistance, pointDistance, pointIntensity));
         pointLights.lights.add(new PointLight().set(pointColor, pointColor, pointColor, pointDistance, -pointDistance, pointDistance, pointIntensity));
         environment.set(pointLights);
      }
      float directionColor = GDX3DSceneTools.directionColor.get();
      float directionDistance = GDX3DSceneTools.directionDistance.get();
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
