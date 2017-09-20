package us.ihmc.graphicsDescription.instructions.primitives;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.instructions.Graphics3DPrimitiveInstruction;
import us.ihmc.graphicsDescription.instructions.listeners.ScaleChangedListener;

public class Graphics3DScaleInstruction implements Graphics3DPrimitiveInstruction
{
   private Vector3D scaleFactor;
   private ScaleChangedListener scaleChangedListener = null;

   public Graphics3DScaleInstruction(double scale)
   {
      scaleFactor = new Vector3D(scale, scale, scale);
   }

   public Graphics3DScaleInstruction(Vector3D scale)
   {
      scaleFactor = scale;
   }
   
   public void setScale(Vector3D scale)
   {
      scaleFactor = scale;
      if(scaleChangedListener != null)
      {
         scaleChangedListener.setScale(scale);
      }
   }
   
   public void setScale(double scale)
   {
      setScale(new Vector3D(scale, scale, scale));
   }

   public Vector3D getScaleFactor()
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
