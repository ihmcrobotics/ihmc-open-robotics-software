package us.ihmc.rdx.shader;

import com.badlogic.gdx.graphics.g3d.Attributes;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.graphics.g3d.shaders.BaseShader;

public class RDXUniform
{
   private final BaseShader.Uniform uniform;
   private final BaseShader.Setter setter;

   public static RDXUniform createGlobalUniform(String alias, RDXSetterSetInterface set)
   {
      return new RDXUniform(alias, true, set);
   }

   public static RDXUniform createLocalUniform(String alias, RDXSetterSetInterface set)
   {
      return new RDXUniform(alias, false, set);
   }

   private RDXUniform(String alias, boolean isGlobal, RDXSetterSetInterface set)
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
