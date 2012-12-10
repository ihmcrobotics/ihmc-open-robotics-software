package us.ihmc.graphics3DAdapter.graphics.instructions;

import javax.vecmath.Vector3d;

public class LinkGraphicsScale implements LinkGraphicsPrimitiveInstruction
{
   private Vector3d scaleFactor;
   private boolean changed = false;

   public LinkGraphicsScale(double scale)
   {
      scaleFactor = new Vector3d(scale, scale, scale);
   }

   public LinkGraphicsScale(Vector3d scale)
   {
      scaleFactor = scale;
   }
   
   public void setScale(Vector3d scale)
   {
      scaleFactor = scale;
      changed = true;
   }
   
   public void setScale(double scale)
   {
      setScale(new Vector3d(scale, scale, scale));
   }
   
   public boolean hasChanged()
   {
      return changed;
   }
   
   public void reset()
   {
      changed = false;
   }

   public Vector3d getScaleFactor()
   {
      return scaleFactor;
   }

   public String toString()
   {

      return "\t\t\t<Scale>"+scaleFactor+"</Scale>\n";
   }

   public boolean hasChangedSinceLastCalled()
   {
      return false;
   }
}
