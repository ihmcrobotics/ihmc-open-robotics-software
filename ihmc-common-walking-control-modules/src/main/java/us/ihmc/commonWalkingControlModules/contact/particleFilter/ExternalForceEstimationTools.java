package us.ihmc.commonWalkingControlModules.contact.particleFilter;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.robotics.screwTheory.GeometricJacobian;

import java.util.function.IntUnaryOperator;
import java.util.function.ToIntFunction;
import java.util.stream.IntStream;

public class ExternalForceEstimationTools
{
   public static int[] createIndexMap(GeometricJacobian jacobian, JointBasics[] joints)
   {
      int[] indexMap = new int[jacobian.getJacobianMatrix().getNumCols()];

      JointBasics[] jointPath = jacobian.getJointsInOrder();
      ToIntFunction<JointBasics> jointIndexFunction = joint -> IntStream.range(0, joints.length)
                                                                        .filter(i -> joint == joints[i])
                                                                        .findFirst()
                                                                        .orElseThrow(() -> new RuntimeException("Could not find joint"));
      IntUnaryOperator indexOffset = i -> IntStream.range(0, i).map(j -> joints[j].getDegreesOfFreedom()).sum();

      for (int jointIndex = 0, mappedIndex = 0; jointIndex < jointPath.length; jointIndex++)
      {
         JointBasics joint = jointPath[jointIndex];
         int offset = indexOffset.applyAsInt(jointIndexFunction.applyAsInt(joint));

         for (int i = 0; i < joint.getDegreesOfFreedom(); i++)
         {
            indexMap[mappedIndex++] = offset + i;
         }
      }

      return indexMap;
   }

   public static void transformToSphericalCoordinates(Tuple3DReadOnly cartesianCoordinates, Tuple3DBasics sphericalCoordinates)
   {
      double r = EuclidCoreTools.norm(cartesianCoordinates.getX(), cartesianCoordinates.getY(), cartesianCoordinates.getZ());
      double theta = Math.atan2(cartesianCoordinates.getY(), cartesianCoordinates.getX());
      double phi = Math.acos(cartesianCoordinates.getZ() / r);

      sphericalCoordinates.set(r, theta, phi);
   }

   public static void transformToCartesianCoordinates(Tuple3DReadOnly sphericalCoordinates, Tuple3DBasics cartesianCoordinates)
   {
      double r = sphericalCoordinates.getX();
      double theta = sphericalCoordinates.getY();
      double phi = sphericalCoordinates.getZ();

      double x = r * Math.cos(theta) * Math.sin(phi);
      double y = r * Math.sin(theta) * Math.sin(phi);
      double z = r * Math.cos(phi);

      cartesianCoordinates.set(x, y, z);
   }
}
