package us.ihmc.stateEstimation.humanoid.kinematicsBasedStateEstimation;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;

public class CompassYawRemover
{
   // Temporary variables for removeCompassYaw method.
   private final RotationMatrix tempRotationMatrix = new RotationMatrix();
   private final Quaternion tempQuaternion = new Quaternion();

   private final Vector3D xAxis = new Vector3D();
   private final Vector3D yAxis = new Vector3D();
   private final Vector3D zAxis = new Vector3D();

   /**
    * Removes the yaw of a orientation, assuming that the orientation goes through yaw, then roll, then
    * pitch. For example, with a humanoid robot, you might have an IMU attached to the shin and the
    * foot fixed to the ground. Or you might have a pelvis that is fixed to the ground and have an IMU
    * attached to the torso. What this means is that from the fixed link to this link, you first see
    * roll, then you see pitch, whereas euler angles assume you first see pitch, then you see roll.
    * Because of that, we don't actually want the final yaw (of euler angles) to be zero. We want the
    * final yAxis to not have an x component, and retain its z component. Zero yaw makes it so that the
    * final xAxis has no y component. So we keep yAxisZ, set yAxisX to 0, and solve for yAxisY, given
    * that yAxis is a unit vector, yAxisY = Math.sqrt(1.0 - yAxisZ * yAxisZ). Then using xAxis dot
    * yAxis = 0, and retaining xAxisZ, we solve for xAxisY = (-yAxisZ * xAxisZ)/yAxisY. Then we can
    * solve for xAxisX, given that xAxis is a unit vector, xAxisX = Math.sqrt(1.0 - xAxisY * xAxisY -
    * xAxisZ * xAxisZ). However, if the original zAxisZ is negative, then we assume that it got that
    * way by a large pitch motion, so that xAxisX is the negative solution of the square root. Finally
    * the zAxis is xAxis cross yAxis.
    */
   public void removeCompassYawAssumingYawRollPitchOrdering(Orientation3DBasics orientationToRemoveYawFrom)
   {
      orientationToRemoveYawFrom.get(tempRotationMatrix);

      double yAxisZ = tempRotationMatrix.getM21();
      double yAxisY = Math.sqrt(1.0 - yAxisZ * yAxisZ);
      yAxis.set(0.0, yAxisY, yAxisZ);

      double xAxisZ = tempRotationMatrix.getM20();
      double xAxisY = (-yAxisZ * xAxisZ) / yAxisY;
      double xAxisX = Math.sqrt(1.0 - xAxisY * xAxisY - xAxisZ * xAxisZ);

      if (tempRotationMatrix.getM22() < 0.0)
         xAxisX = -xAxisX;

      xAxis.set(xAxisX, xAxisY, xAxisZ);

      zAxis.cross(xAxis, yAxis);
      tempRotationMatrix.setColumns(xAxis, yAxis, zAxis);
      orientationToRemoveYawFrom.set(tempRotationMatrix);
   }

   /**
    * Removes the yaw of a orientation, assuming that the orientation goes through yaw, then pitch,
    * then roll. See removeCompassYawAssumingYawRollPitchOrdering() for more details. In this case,
    * Zero yaw makes it so that the final yAxis has no x component.
    **/
   public void removeCompassYawAssumingYawPitchRollOrdering(Orientation3DBasics orientationToRemoveYawFrom)
   {
      orientationToRemoveYawFrom.get(tempRotationMatrix);

      double xAxisZ = tempRotationMatrix.getM20();
      double xAxisX = Math.sqrt(1.0 - xAxisZ * xAxisZ);
      xAxis.set(xAxisX, 0.0, xAxisZ);

      double yAxisZ = tempRotationMatrix.getM21();
      double yAxisX = (-xAxisZ * yAxisZ) / xAxisX;
      double yAxisY = Math.sqrt(1.0 - yAxisX * yAxisX - yAxisZ * yAxisZ);

      if (tempRotationMatrix.getM22() < 0.0)
         yAxisY = -yAxisY;

      yAxis.set(yAxisX, yAxisY, yAxisZ);

      zAxis.cross(xAxis, yAxis);
      tempRotationMatrix.setColumns(xAxis, yAxis, zAxis);
      orientationToRemoveYawFrom.set(tempRotationMatrix);
   }

   /**
    * Removes the yaw (Rz) from the middle of Ra * Rz * Rb = R(x)R(y) when Ra and Rb are known and R(x)
    * is a rotation about the x axis and R(y) is a rotation about the y axis.
    */
   public void removeCompassYawFromMiddleAssumingRollPitchOrdering(Orientation3DReadOnly rotationA, Orientation3DReadOnly rotationB,
                                                                   Orientation3DBasics rollPitchSolutionToPack)
   {
      tempRotationMatrix.set(rotationA);
      double a11 = tempRotationMatrix.getElement(0, 0);
      double a12 = tempRotationMatrix.getElement(0, 1);
      double a13 = tempRotationMatrix.getElement(0, 2);

      tempRotationMatrix.set(rotationB);
      double b12 = tempRotationMatrix.getElement(0, 1);
      double b22 = tempRotationMatrix.getElement(1, 1);
      double b32 = tempRotationMatrix.getElement(2, 1);

      double a = b12 * a11 + b22 * a12;
      double b = b12 * a12 - b22 * a11;
      double c = -b32 * a13;

      double theta = solveACosThetaPlusBSinThetaEqualsC(a, b, c);
      tempQuaternion.setToYawOrientation(theta);

      rollPitchSolutionToPack.set(rotationA);
      rollPitchSolutionToPack.append(tempQuaternion);
      rollPitchSolutionToPack.append(rotationB);
   }

   private final Vector3D tempVector = new Vector3D();

   /**
    * Computes the roll and pitch between two vectors assuming that to get from the frame of the first vector
    * to the frame of the second vector, that the frame is first rolled about the X axis,
    * then pitched about the y axis. v1 = R(roll) * R(pitch) * v2. This method assumes that there is no scaling between 
    * the vectors (their lengths are the same). This method will break down if the vectors are in a singularity configuration.
    * Examples include if the vectors are: 
    * 
    */
   public void computeRollAndPitchGivenTwoVectors(Vector3DReadOnly vectorOne, Vector3DReadOnly vectorTwo, double[] rollAndPitchToPack)
   {
      double pitch = solveACosThetaPlusBSinThetaEqualsC(vectorTwo.getX(), vectorTwo.getZ(), vectorOne.getX());

      tempVector.set(vectorTwo);
      tempRotationMatrix.setYawPitchRoll(0.0, pitch, 0.0);

      tempRotationMatrix.transform(tempVector);
      double roll = -solveACosThetaPlusBSinThetaEqualsC(tempVector.getY(), tempVector.getZ(), vectorOne.getY());

      rollAndPitchToPack[0] = roll;
      rollAndPitchToPack[1] = pitch;
   }

   /**
    * From https://www.math-only-math.com/a-cos-theta-plus-b-sin-theta-equals-c.html c^2 must be less
    * than or equal to a^2+b^2. If it is not, it will be reduced so that it is.
    */
   public double solveACosThetaPlusBSinThetaEqualsC(double a, double b, double c)
   {
      double r = Math.sqrt(a * a + b * b);
      double alpha = Math.atan2(b, a);

      double cOverR = c / r;
      if (cOverR > 1.0)
         cOverR = 1.0;
      if (cOverR < -1.0)
         cOverR = -1.0;

      double beta = Math.acos(cOverR);
      return alpha + beta;
   }
}
