package us.ihmc.gdx.sceneManager;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.g3d.Environment;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.DirectionalLightsAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.PointLightsAttribute;
import com.badlogic.gdx.graphics.g3d.environment.DirectionalLight;
import com.badlogic.gdx.graphics.g3d.environment.PointLight;
import imgui.ImGui;
import imgui.type.ImFloat;
import org.lwjgl.opengl.GL32;

public class GDX3DSceneTools
{
   public static final float CLEAR_COLOR = 0.5019608f;

   private static ImFloat ambientColor = new ImFloat(0.6f);
   private static ImFloat pointColor = new ImFloat(1.0f);
   private static ImFloat pointDistance = new ImFloat(17.0f);
   private static ImFloat pointIntensity = new ImFloat(155.0f);

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
      float pointColor = GDX3DSceneTools.pointColor.get();
      float pointDistance = GDX3DSceneTools.pointDistance.get();
      float pointIntensity = GDX3DSceneTools.pointIntensity.get();
      environment.set(new ColorAttribute(ColorAttribute.AmbientLight, ambientColor, ambientColor, ambientColor, 1.0f));
      // Point lights not working; not sure why @dcalvert
      PointLightsAttribute pointLights = new PointLightsAttribute();
      pointLights.lights.add(new PointLight().set(pointColor, pointColor, pointColor, pointDistance, pointDistance, pointDistance, pointIntensity));
      pointLights.lights.add(new PointLight().set(pointColor, pointColor, pointColor, -pointDistance, pointDistance, pointDistance, pointIntensity));
      pointLights.lights.add(new PointLight().set(pointColor, pointColor, pointColor, -pointDistance, -pointDistance, pointDistance, pointIntensity));
      pointLights.lights.add(new PointLight().set(pointColor, pointColor, pointColor, pointDistance, -pointDistance, pointDistance, pointIntensity));
      environment.set(pointLights);
//      DirectionalLightsAttribute directionalLights = new DirectionalLightsAttribute();
//      directionalLights.lights.add(new DirectionalLight().set(pointColor, pointColor, pointColor, -pointDistance, -pointDistance, -pointDistance));
//      directionalLights.lights.add(new DirectionalLight().set(pointColor, pointColor, pointColor, pointDistance, -pointDistance, -pointDistance));
//      directionalLights.lights.add(new DirectionalLight().set(pointColor, pointColor, pointColor, pointDistance, pointDistance, -pointDistance));
//      directionalLights.lights.add(new DirectionalLight().set(pointColor, pointColor, pointColor, -pointDistance, pointDistance, -pointDistance));
//      environment.set(directionalLights);
      return environment;
   }

   public static void renderTuningSliders()
   {
      ImGui.begin("Lighting");
      ImGui.sliderFloat("Ambient color", ambientColor.getData(), 0.0f, 1.0f);
      ImGui.sliderFloat("Point color", pointColor.getData(), 0.0f, 1.0f);
      ImGui.sliderFloat("Point distance", pointDistance.getData(), 0.0f, 500.0f);
      ImGui.sliderFloat("Point intensity", pointIntensity.getData(), 0.0f, 500.0f);
      ImGui.end();
   }
}
