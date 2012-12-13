package us.ihmc.graphics3DAdapter.graphics.instructions;

import javax.vecmath.Vector3d;

public class Graphics3DRotateInstruction implements Graphics3DPrimitiveInstruction
{
   private double angle;
   private Vector3d axis;

   public Graphics3DRotateInstruction(double angle, Vector3d axis)
   {
      this.angle = angle;
      this.axis = axis;
   }
   public double getAngle()
   {
      return angle;
   }
   public Vector3d getAxis()
   {
      return axis;
   }

   public String toString()
   {
      return "\t\t\t<Rotate>"+angle+","+axis.x+","+axis.y+","+axis.z+"</Rotate>\n";
   }
   public boolean hasChangedSinceLastCalled()
   {
      return false;
   }
}
