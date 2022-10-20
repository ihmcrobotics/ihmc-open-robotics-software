package us.ihmc.rdx.shader;

import com.badlogic.gdx.graphics.g3d.Attributes;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.shaders.BaseShader;

public class GDXUniform
{
   private final BaseShader.Uniform uniform;
   private final BaseShader.Setter setter;

   public static GDXUniform createGlobalUniform(String alias, GDXSetterSetInterface set)
   {
      return new GDXUniform(alias, true, set);
   }

   public static GDXUniform createLocalUniform(String alias, GDXSetterSetInterface set)
   {
      return new GDXUniform(alias, false, set);
   }

   private GDXUniform(String alias, boolean isGlobal, GDXSetterSetInterface set)
   {
      uniform = new BaseShader.Uniform(alias);
      setter = new BaseShader.Setter()
      {
         @Override
         public boolean isGlobal(BaseShader shader, int inputID)
         {
            return isGlobal;
         }

         @Override
         public void set(BaseShader shader, int inputID, Renderable renderable, Attributes combinedAttributes)
         {
            set.set(shader, inputID, renderable, combinedAttributes);
         }
      };
   }

   public BaseShader.Uniform getUniform()
   {
      return uniform;
   }

   public BaseShader.Setter getSetter()
   {
      return setter;
   }
}
