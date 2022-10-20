package us.ihmc.rdx.shader;

import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.Shader;
import com.badlogic.gdx.graphics.g3d.shaders.BaseShader;

public class GDXDefaultBaseShader extends BaseShader
{
   @Override
   public void init()
   {

   }

   @Override
   public int compareTo(Shader other)
   {
      if (other == null)
         return -1;
      else if (other == this)
         return 0;
      return 0;
   }

   @Override
   public boolean canRender(Renderable instance)
   {
      return true;
   }
}
