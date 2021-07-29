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

   public static void glClearGray()
   {
      glClearGray(CLEAR_COLOR);
   }

   public static void glClearGray(float color)
   {
      Gdx.gl.glClearColor(color, color, color, 1.0f);
      Gdx.gl.glClear(GL32.GL_COLOR_BUFFER_BIT | GL32.GL_DEPTH_BUFFER_BIT);
   }
}
