package us.ihmc.graphicsDescription.instructions.primitives;

import javax.vecmath.Vector3d;

import us.ihmc.graphicsDescription.instructions.Graphics3DPrimitiveInstruction;
import us.ihmc.graphicsDescription.instructions.listeners.ScaleChangedListener;

public class Graphics3DScaleInstruction implements Graphics3DPrimitiveInstruction
{
   private Vector3d scaleFactor;
   private ScaleChangedListener scaleChangedListener = null;

   public Graphics3DScaleInstruction(double scale)
   {
      scaleFactor = new Vector3d(scale, scale, scale);
   }

   public Graphics3DScaleInstruction(Vector3d scale)
   {
      scaleFactor = scale;
   }
   
   public void setScale(Vector3d scale)
   {
      scaleFactor = scale;
      if(scaleChangedListener != null)
      {
         scaleChangedListener.setScale(scale);
      }
   }
   
   public void setScale(double scale)
   {
      setScale(new Vector3d(scale, scale, scale));
   }

   public Vector3d getScaleFactor()
   {
      return scaleFactor;
   }

   @Override
   public String toString()
   {

      return "\t\t\t<Scale>"+scaleFactor+"</Scale>\n";
   }

   public void addChangeScaleListener(ScaleChangedListener scaleChangedListener)
   {
      this.scaleChangedListener = scaleChangedListener;
   }
}
