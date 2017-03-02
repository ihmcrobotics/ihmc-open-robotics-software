package us.ihmc.graphicsDescription.instructions.primitives;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.graphicsDescription.instructions.Graphics3DPrimitiveInstruction;

public class Graphics3DTranslateInstruction implements Graphics3DPrimitiveInstruction
{
   private Vector3D translation = new Vector3D();

   public Graphics3DTranslateInstruction(Tuple3DReadOnly translation)
   {
      this.translation.set(translation);
   }

   public Graphics3DTranslateInstruction(double tx, double ty, double tz)
   {
      translation.set(tx, ty, tz);
   }

   public Vector3D getTranslation()
   {
      return translation;
   }


   public String toString()
   {
      return "\t\t\t<Translate>" + translation + "</Translate>\n";
   }

   public boolean hasChangedSinceLastCalled()
   {
      return false;
   }
}
