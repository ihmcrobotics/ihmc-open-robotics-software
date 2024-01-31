package us.ihmc.rdx;

import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.g3d.Model;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import org.bytedeco.javacpp.BytePointer;

public class RDX3DSituatedTextData
{
   private final Pixmap pixmap;
   private final BytePointer rgba8888BytePointer;
   private final Texture texture;
   private final Model model;
   private final ModelInstance modelInstance;

   public RDX3DSituatedTextData(Pixmap pixmap, BytePointer rgba8888BytePointer, Texture texture, Model model, ModelInstance modelInstance)
   {
      this.pixmap = pixmap;
      this.rgba8888BytePointer = rgba8888BytePointer;
      this.texture = texture;
      this.model = model;
      this.modelInstance = modelInstance;
   }

   public void dispose()
   {
      model.dispose();
      texture.dispose();
      rgba8888BytePointer.close();
      pixmap.dispose();
   }

   public ModelInstance getModelInstance()
   {
      return modelInstance;
   }
}
