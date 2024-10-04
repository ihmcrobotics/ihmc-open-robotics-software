package us.ihmc.rdx.shader;

import com.badlogic.gdx.graphics.g3d.Attributes;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.shaders.BaseShader;

public interface RDXSetterSetInterface
{
   public void set (BaseShader shader, int inputID, Renderable renderable, Attributes combinedAttributes);
}
