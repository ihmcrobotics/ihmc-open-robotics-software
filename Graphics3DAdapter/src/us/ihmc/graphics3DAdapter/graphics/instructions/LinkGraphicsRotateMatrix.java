package us.ihmc.graphics3DAdapter.graphics.instructions;

import javax.vecmath.Matrix3d;

public class LinkGraphicsRotateMatrix implements LinkGraphicsPrimitiveInstruction
{
   private Matrix3d rot;
   public LinkGraphicsRotateMatrix(Matrix3d rot)
   {
      this.rot = rot;
   }
   public Matrix3d getRotationMatrix()
   {
      return rot;
   }


   public String toString()
   {
      String matrix = rot.toString().replaceAll("\n", ", ");
      return "\t\t\t<RotateMatrix>"+matrix.substring(0, matrix.length() - 2)+"</RotateMatrix>\n";
   }
   public boolean hasChangedSinceLastCalled()
   {
      return false;
   }
}
