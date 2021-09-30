package us.ihmc.gdx.shader;

import com.badlogic.gdx.graphics.g3d.Attributes;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.shaders.BaseShader;

public interface GDXSetterSetInterface
{
   public void set (BaseShader shader, int inputID, Renderable renderable, Attributes combinedAttributes);
}
