package us.ihmc.robotics.interaction;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.matrix.RotationMatrix;

/**
 * Used for specifying 3-axis graphics, like position and orientation gizmos.
 */
public class Axis3DRotations
{
   private final RotationMatrix[] axisRotations = new RotationMatrix[3];

   public Axis3DRotations()
   {
      axisRotations[Axis3D.X.ordinal()] = new RotationMatrix(0.0, Math.PI / 2.0, 0.0);
      axisRotations[Axis3D.Y.ordinal()] = new RotationMatrix(0.0, 0.0, -Math.PI / 2.0);
      axisRotations[Axis3D.Z.ordinal()] = new RotationMatrix();
   }

   public RotationMatrix get(Axis3D axis3D)
   {
      return axisRotations[axis3D.ordinal()];
   }
}
