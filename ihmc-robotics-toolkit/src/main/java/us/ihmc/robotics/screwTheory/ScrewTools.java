package us.ihmc.robotics.screwTheory;

import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.stream.Stream;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.tools.MultiBodySystemTools;

public class ScrewTools
{
   public static RigidBodyBasics[] computeSubtreeSuccessors(RigidBodyBasics... bodies)
   {
      return MultiBodySystemTools.collectSuccessors(MultiBodySystemTools.collectSubtreeJoints(bodies));
   }

   public static SpatialAcceleration createGravitationalSpatialAcceleration(RigidBodyBasics rootBody, double gravity)
   {
      Vector3D gravitationalAcceleration = new Vector3D(0.0, 0.0, gravity);
      Vector3D zero = new Vector3D();
      SpatialAcceleration rootAcceleration = new SpatialAcceleration(rootBody.getBodyFixedFrame(), ReferenceFrame.getWorldFrame(), rootBody.getBodyFixedFrame(),
                                                                     zero, gravitationalAcceleration);

      return rootAcceleration;
   }



   public static JointBasics[] findJointsWithNames(JointBasics[] allJoints, String... jointNames)
   {
      Set<String> jointNameSet = new HashSet<>(Arrays.asList(jointNames));
      JointBasics[] result = Stream.of(allJoints).distinct().filter(joint -> jointNameSet.contains(joint.getName())).toArray(JointBasics[]::new);

      if (result.length != jointNames.length)
         throw new RuntimeException("Not all joints could be found");

      return result;
   }

   public static RigidBodyBasics[] findRigidBodiesWithNames(RigidBodyBasics[] allBodies, String... bodyNames)
   {
      Set<String> bodyNameSet = new HashSet<>(Arrays.asList(bodyNames));
      RigidBodyBasics[] result = Stream.of(allBodies).distinct().filter(body -> bodyNameSet.contains(body.getName())).toArray(RigidBodyBasics[]::new);

      if (result.length != bodyNames.length)
         throw new RuntimeException("Not all bodies could be found");

      return result;
   }

   public static int computeGeometricJacobianHashCode(JointReadOnly[] joints, ReferenceFrame jacobianFrame, boolean allowChangeFrame)
   {
      int jointsHashCode = 1;
      for (JointReadOnly joint : joints)
      {
         jointsHashCode = 31 * jointsHashCode + joint.hashCode();
      }
      if (!allowChangeFrame)
         return 31 * jointsHashCode + jacobianFrame.hashCode();
      else
         return jointsHashCode;
   }

   public static int computeGeometricJacobianHashCode(JointReadOnly[] joints, int firstIndex, int lastIndex, ReferenceFrame jacobianFrame,
                                                      boolean allowChangeFrame)
   {
      int jointsHashCode = 1;
      for (int i = firstIndex; i <= lastIndex; i++)
      {
         jointsHashCode = 31 * jointsHashCode + joints[i].hashCode();
      }
      if (!allowChangeFrame)
         return 31 * jointsHashCode + jacobianFrame.hashCode();
      else
         return jointsHashCode;
   }
}