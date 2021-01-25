package us.ihmc.gdx.sceneManager;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.g3d.Environment;
import com.badlogic.gdx.graphics.g3d.attributes.ColorAttribute;
import com.badlogic.gdx.graphics.g3d.attributes.DirectionalLightsAttribute;
import com.badlogic.gdx.graphics.g3d.environment.DirectionalLight;
import org.lwjgl.opengl.GL32;

public class GDX3DSceneTools
{
   public static final float CLEAR_COLOR = 0.5019608f;

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
      float ambientColor = 0.7f;
      float pointColor = 0.07f;
      float pointDistance = 2.0f;
      float pointIntensity = 1.0f;
      environment.set(new ColorAttribute(ColorAttribute.AmbientLight, ambientColor, ambientColor, ambientColor, 1.0f));
      // Point lights not working; not sure why @dcalvert
      //      PointLightsAttribute pointLights = new PointLightsAttribute();
      //      pointLights.lights.add(new PointLight().set(pointColor, pointColor, pointColor, pointDistance, pointDistance, pointDistance, pointIntensity));
      //      pointLights.lights.add(new PointLight().set(pointColor, pointColor, pointColor, -pointDistance, pointDistance, pointDistance, pointIntensity));
      //      pointLights.lights.add(new PointLight().set(pointColor, pointColor, pointColor, -pointDistance, -pointDistance, pointDistance, pointIntensity));
      //      pointLights.lights.add(new PointLight().set(pointColor, pointColor, pointColor, pointDistance, -pointDistance, pointDistance, pointIntensity));
      //      environment.set(pointLights);
      DirectionalLightsAttribute directionalLights = new DirectionalLightsAttribute();
      directionalLights.lights.add(new DirectionalLight().set(pointColor, pointColor, pointColor, -pointDistance, -pointDistance, -pointDistance));
      directionalLights.lights.add(new DirectionalLight().set(pointColor, pointColor, pointColor, pointDistance, -pointDistance, -pointDistance));
      directionalLights.lights.add(new DirectionalLight().set(pointColor, pointColor, pointColor, pointDistance, pointDistance, -pointDistance));
      directionalLights.lights.add(new DirectionalLight().set(pointColor, pointColor, pointColor, -pointDistance, pointDistance, -pointDistance));
      environment.set(directionalLights);
      return environment;
   }
}
