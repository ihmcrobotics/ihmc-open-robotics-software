package us.ihmc.robotics.screwTheory;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.stream.Stream;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.multiBodySystem.OneDoFJoint;
import us.ihmc.mecano.multiBodySystem.PrismaticJoint;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.tools.MultiBodySystemFactories;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.geometry.TransformTools;

public class ScrewTools
{
   public static PassiveRevoluteJoint addPassiveRevoluteJoint(String jointName, RigidBodyBasics parentBody, Vector3D jointOffset, Vector3D jointAxis,
                                                              boolean isPartOfClosedKinematicLoop)
   {
      return addPassiveRevoluteJoint(jointName, parentBody, TransformTools.createTranslationTransform(jointOffset), jointAxis, isPartOfClosedKinematicLoop);
   }

   public static PassiveRevoluteJoint addPassiveRevoluteJoint(String jointName, RigidBodyBasics parentBody, RigidBodyTransform transformToParent,
                                                              Vector3D jointAxis, boolean isPartOfClosedKinematicLoop)
   {
      return new PassiveRevoluteJoint(jointName, parentBody, transformToParent, jointAxis, isPartOfClosedKinematicLoop);
   }

   public static RigidBodyBasics[] computeSubtreeSuccessors(JointBasics... joints)
   {
      ArrayList<RigidBodyBasics> rigidBodySuccessors = new ArrayList<RigidBodyBasics>();
      ArrayList<RigidBodyBasics> rigidBodyStack = new ArrayList<RigidBodyBasics>();
      for (JointBasics joint : joints)
      {
         rigidBodyStack.add(joint.getPredecessor());
      }
      while (!rigidBodyStack.isEmpty())
      {
         RigidBodyBasics currentBody = rigidBodyStack.remove(0);
         List<? extends JointBasics> childrenJoints = currentBody.getChildrenJoints();
         for (JointBasics joint : childrenJoints)
         {
            rigidBodyStack.add(joint.getSuccessor());
            rigidBodySuccessors.add(joint.getSuccessor());
         }
      }
      RigidBodyBasics[] ret = new RigidBodyBasics[rigidBodySuccessors.size()];

      return rigidBodySuccessors.toArray(ret);
   }

   public static RigidBodyBasics[] computeRigidBodiesAfterThisJoint(JointBasics... joints)
   {
      ArrayList<RigidBodyBasics> rigidBodySuccessors = new ArrayList<RigidBodyBasics>();
      computeRigidBodiesAfterThisJoint(rigidBodySuccessors, joints);
      RigidBodyBasics[] ret = new RigidBodyBasics[rigidBodySuccessors.size()];

      return rigidBodySuccessors.toArray(ret);
   }

   public static void computeRigidBodiesAfterThisJoint(ArrayList<RigidBodyBasics> rigidBodySuccessorsToPack, JointBasics... joints)
   {
      ArrayList<JointBasics> jointStack = new ArrayList<JointBasics>();
      for (JointBasics joint : joints)
      {
         jointStack.add(joint);
      }
      while (!jointStack.isEmpty())
      {
         JointBasics currentJoint = jointStack.remove(0);
         rigidBodySuccessorsToPack.add(currentJoint.getSuccessor());
         RigidBodyBasics currentBody = currentJoint.getSuccessor();
         List<? extends JointBasics> childrenJoints = currentBody.getChildrenJoints();
         for (JointBasics joint : childrenJoints)
         {
            jointStack.add(joint);
         }
      }
   }

   public static void computeRigidBodiesFromRootToThisJoint(ArrayList<RigidBodyBasics> rigidBodySuccessorsToPack, JointBasics joint)
   {
      RigidBodyBasics predecessorBody = joint.getPredecessor();
      if (predecessorBody == null)
         return;
      if (predecessorBody.isRootBody())
         return;

      rigidBodySuccessorsToPack.add(predecessorBody);
      JointBasics parentJoint = predecessorBody.getParentJoint();
      if (parentJoint == null)
         return;

      computeRigidBodiesFromRootToThisJoint(rigidBodySuccessorsToPack, parentJoint);
   }

   public static RigidBodyBasics[] computeSubtreeSuccessors(RigidBodyBasics... bodies)
   {
      return MultiBodySystemTools.collectSuccessors(computeSubtreeJoints(bodies));
   }

   public static RigidBodyBasics[] computeSubtreeSuccessors(Set<JointBasics> jointsToExclude, RigidBodyBasics... bodies)
   {
      ArrayList<RigidBodyBasics> rigidBodySuccessors = new ArrayList<RigidBodyBasics>();
      ArrayList<RigidBodyBasics> rigidBodyStack = new ArrayList<RigidBodyBasics>();
      for (RigidBodyBasics body : bodies)
      {
         rigidBodyStack.add(body);
      }
      while (!rigidBodyStack.isEmpty())
      {
         RigidBodyBasics currentBody = rigidBodyStack.remove(0);
         List<? extends JointBasics> childrenJoints = currentBody.getChildrenJoints();
         for (JointBasics joint : childrenJoints)
         {
            if (!jointsToExclude.contains(joint))
            {
               rigidBodyStack.add(joint.getSuccessor());
               rigidBodySuccessors.add(joint.getSuccessor());
            }
         }
      }

      RigidBodyBasics[] ret = new RigidBodyBasics[rigidBodySuccessors.size()];

      return rigidBodySuccessors.toArray(ret);
   }

   public static RigidBodyBasics[] computeSupportAndSubtreeSuccessors(RigidBodyBasics... bodies)
   {
      return MultiBodySystemTools.collectSuccessors(computeSupportAndSubtreeJoints(bodies));
   }

   public static JointBasics[] computeSupportAndSubtreeJoints(RigidBodyBasics... bodies)
   {
      Set<JointBasics> ret = new LinkedHashSet<JointBasics>();
      for (RigidBodyBasics body : bodies)
      {
         ret.addAll(Arrays.asList(computeSupportJoints(body)));
         ret.addAll(Arrays.asList(computeSubtreeJoints(body)));
      }
      return ret.toArray(new JointBasics[ret.size()]);
   }

   public static JointBasics[] computeSupportJoints(RigidBodyBasics... bodies)
   {
      Set<JointBasics> supportSet = new LinkedHashSet<JointBasics>();
      for (RigidBodyBasics rigidBody : bodies)
      {
         RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(rigidBody);
         JointBasics[] jointPath = createJointPath(rootBody, rigidBody);
         supportSet.addAll(Arrays.asList(jointPath));
      }

      return supportSet.toArray(new JointBasics[supportSet.size()]);
   }

   public static JointBasics[] computeSubtreeJoints(RigidBodyBasics... rootBodies)
   {
      return computeSubtreeJoints(Arrays.asList(rootBodies));
   }

   public static JointBasics[] computeSubtreeJoints(List<RigidBodyBasics> rootBodies)
   {
      ArrayList<JointBasics> subtree = new ArrayList<JointBasics>();
      ArrayList<RigidBodyBasics> rigidBodyStack = new ArrayList<RigidBodyBasics>();
      rigidBodyStack.addAll(rootBodies);

      while (!rigidBodyStack.isEmpty())
      {
         RigidBodyBasics currentBody = rigidBodyStack.remove(0);
         List<? extends JointBasics> childrenJoints = currentBody.getChildrenJoints();
         for (JointBasics joint : childrenJoints)
         {
            RigidBodyBasics successor = joint.getSuccessor();
            rigidBodyStack.add(successor);
            subtree.add(joint);
         }
      }

      JointBasics[] ret = new JointBasics[subtree.size()];
      return subtree.toArray(ret);
   }

   public static OneDoFJoint[] createOneDoFJointPath(RigidBodyBasics start, RigidBodyBasics end)
   {
      return MultiBodySystemTools.filterJoints(createJointPath(start, end), OneDoFJoint.class);
   }

   public static JointBasics[] createJointPath(RigidBodyBasics start, RigidBodyBasics end)
   {
      boolean flip = false;
      RigidBodyBasics descendant = start;
      RigidBodyBasics ancestor = end;
      int pathLength = MultiBodySystemTools.computeDistanceToAncestor(descendant, ancestor);
      if (pathLength < 0)
      {
         flip = true;
         descendant = end;
         ancestor = start;
         pathLength = MultiBodySystemTools.computeDistanceToAncestor(end, start);
      }

      JointBasics[] ret = new JointBasics[pathLength];
      RigidBodyBasics currentBody = descendant;
      int i = 0;
      while (currentBody != ancestor)
      {
         int j = flip ? pathLength - 1 - i : i;
         JointBasics parentJoint = currentBody.getParentJoint();
         ret[j] = parentJoint;
         currentBody = parentJoint.getPredecessor();
         i++;
      }

      return ret;
   }

   /**
    * Compute and pack the joint path between two RigidBody in the jointPathToPack. Use the method
    * {@link #computeDistanceToAncestor(RigidBodyBasics, RigidBodyBasics)} to get the size of the Array
    * to provide.
    * 
    * @param jointPathToPack
    * @param start
    * @param end
    * @return the length of the joint path, returns -1 if the the given jointPathToPack is too small.
    */
   public static int createJointPath(JointBasics[] jointPathToPack, RigidBodyBasics start, RigidBodyBasics end)
   {
      boolean flip = false;
      RigidBodyBasics descendant = start;
      RigidBodyBasics ancestor = end;
      int pathLength = MultiBodySystemTools.computeDistanceToAncestor(descendant, ancestor);
      if (pathLength < 0)
      {
         flip = true;
         descendant = end;
         ancestor = start;
         pathLength = MultiBodySystemTools.computeDistanceToAncestor(end, start);
      }

      if (jointPathToPack == null || jointPathToPack.length < pathLength)
         return -1;

      RigidBodyBasics currentBody = descendant;
      int i = 0;
      while (currentBody != ancestor)
      {
         int j = flip ? pathLength - 1 - i : i;
         JointBasics parentJoint = currentBody.getParentJoint();
         jointPathToPack[j] = parentJoint;
         currentBody = parentJoint.getPredecessor();
         i++;
      }

      for (int k = pathLength; k < jointPathToPack.length; k++)
         jointPathToPack[k] = null;

      return pathLength;
   }

   public static OneDoFJointBasics[] cloneOneDoFJointPath(RigidBodyBasics start, RigidBodyBasics end)
   {
      return MultiBodySystemFactories.cloneOneDoFJointKinematicChain(start, end);
   }

   public static OneDoFJoint[] cloneOneDoFJointPath(OneDoFJoint[] oneDoFJoints)
   {
      return cloneJointPathAndFilter(oneDoFJoints, OneDoFJoint.class);
   }

   public static <T extends JointBasics> T[] cloneJointPathAndFilter(T[] joints, Class<T> clazz)
   {
      return MultiBodySystemTools.filterJoints(cloneJointPath(joints), clazz);
   }

   public static <T extends JointBasics> T[] cloneJointPathAndFilter(T[] joints, Class<T> clazz, String suffix)
   {
      return MultiBodySystemTools.filterJoints(cloneJointPath(joints, suffix), clazz);
   }

   public static JointBasics[] cloneJointPath(JointBasics[] inverseDynamicsJoints)
   {
      String clonedJointNameSuffix = "Copy";

      return cloneJointPath(inverseDynamicsJoints, clonedJointNameSuffix);
   }

   public static JointBasics[] cloneJointPath(JointBasics[] inverseDynamicsJoints, String suffix)
   {
      JointBasics[] cloned = new JointBasics[inverseDynamicsJoints.length];
      Map<RigidBodyBasics, RigidBodyBasics> originalToClonedRigidBodies = new HashMap<>();

      for (int i = 0; i < inverseDynamicsJoints.length; i++)
      {
         if (inverseDynamicsJoints[i] instanceof OneDoFJoint)
         {
            OneDoFJoint jointOriginal = (OneDoFJoint) inverseDynamicsJoints[i];

            RigidBodyBasics predecessorOriginal = jointOriginal.getPredecessor();
            RigidBodyBasics predecessorCopy = originalToClonedRigidBodies.get(predecessorOriginal);

            if (predecessorCopy == null)
            {
               if (predecessorOriginal.isRootBody())
               {
                  predecessorCopy = predecessorOriginal;
                  originalToClonedRigidBodies.put(predecessorOriginal, predecessorCopy);
               }
               else if (originalToClonedRigidBodies.isEmpty())
               {
                  String predecessorNameOriginal = predecessorOriginal.getName();
                  ReferenceFrame predecessorFrameAfterParentJointOriginal = predecessorOriginal.getParentJoint().getFrameAfterJoint();
                  predecessorCopy = new RigidBody(predecessorNameOriginal + suffix, predecessorFrameAfterParentJointOriginal);
                  originalToClonedRigidBodies.put(predecessorOriginal, predecessorCopy);
               }
               else
               {
                  throw new RuntimeException("Unexpected state during cloning operation.");
               }
            }

            cloned[i] = cloneOneDoFJoint(jointOriginal, suffix, predecessorCopy);
         }
         else if (inverseDynamicsJoints[i] instanceof SixDoFJoint)
         {
            SixDoFJoint jointOriginal = (SixDoFJoint) inverseDynamicsJoints[i];
            RigidBodyBasics rootBody = jointOriginal.getPredecessor();

            if (rootBody.getParentJoint() != null)
               throw new RuntimeException("The SixDoFJoint predecessor is not the root body. Case not handled.");

            String rootBodyNameOriginal = rootBody.getName();
            ReferenceFrame rootBodyFrame = rootBody.getBodyFixedFrame();
            RigidBodyBasics rootBodyCopy = new RigidBody(rootBodyNameOriginal + suffix, rootBodyFrame);
            originalToClonedRigidBodies.put(rootBody, rootBodyCopy);

            String jointNameOriginal = jointOriginal.getName();
            SixDoFJoint jointCopy = new SixDoFJoint(jointNameOriginal + suffix, rootBodyCopy);
            cloned[i] = jointCopy;
         }
         else
         {
            throw new RuntimeException("Not implemented for joints of the type: " + inverseDynamicsJoints[i].getClass().getSimpleName());
         }

         RigidBodyBasics successorOriginal = inverseDynamicsJoints[i].getSuccessor();
         RigidBodyBasics successorCopy = originalToClonedRigidBodies.get(successorOriginal);
         if (successorCopy == null)
         {
            successorCopy = cloneRigidBody(successorOriginal, suffix, cloned[i]);
            originalToClonedRigidBodies.put(successorOriginal, successorCopy);
         }

         cloned[i].setSuccessor(successorCopy);
      }
      return cloned;
   }

   public static <T extends JointBasics> T[] cloneJointPathDisconnectedFromOriginalRobot(T[] joints, Class<T> clazz, String suffix,
                                                                                         ReferenceFrame rootBodyFrame)
   {
      return MultiBodySystemTools.filterJoints(cloneJointPathDisconnectedFromOriginalRobot(joints, suffix, rootBodyFrame), clazz);
   }

   public static JointBasics[] cloneJointPathDisconnectedFromOriginalRobot(JointBasics[] inverseDynamicsJoints, String suffix, ReferenceFrame rootBodyFrame)
   {
      JointBasics[] cloned = new JointBasics[inverseDynamicsJoints.length];

      for (int i = 0; i < inverseDynamicsJoints.length; i++)
      {
         if (inverseDynamicsJoints[i] instanceof RevoluteJoint)
         {
            RevoluteJoint jointOriginal = (RevoluteJoint) inverseDynamicsJoints[i];

            RigidBodyBasics predecessorOriginal = jointOriginal.getPredecessor();
            RigidBodyBasics predecessorCopy;

            if (i > 0)
            {
               predecessorCopy = cloned[i - 1].getSuccessor();
            }
            else
            {
               String predecessorNameOriginal = predecessorOriginal.getName();
               predecessorCopy = new RigidBody(predecessorNameOriginal + suffix, rootBodyFrame);
            }

            cloned[i] = cloneOneDoFJoint(jointOriginal, suffix, predecessorCopy);
         }
         else
         {
            throw new RuntimeException("Not implemented for joints of the type: " + inverseDynamicsJoints[i].getClass().getSimpleName());
         }

         cloneRigidBody(inverseDynamicsJoints[i].getSuccessor(), suffix, cloned[i]);
      }
      return cloned;
   }

   private static OneDoFJoint cloneOneDoFJoint(OneDoFJoint original, String cloneSuffix, RigidBodyBasics clonePredecessor)
   {
      String jointNameOriginal = original.getName();
      RigidBodyTransform jointTransform = new RigidBodyTransform();
      original.getJointOffset(jointTransform);
      Vector3D jointAxisCopy = new Vector3D(original.getJointAxis());
      OneDoFJoint clone;

      if (original instanceof RevoluteJoint)
         clone = new RevoluteJoint(jointNameOriginal + cloneSuffix, clonePredecessor, jointTransform, jointAxisCopy);
      else if (original instanceof PrismaticJoint)
         clone = new PrismaticJoint(jointNameOriginal + cloneSuffix, clonePredecessor, jointTransform, jointAxisCopy);
      else
         throw new RuntimeException("Unhandled type of " + OneDoFJoint.class.getSimpleName() + ": " + original.getClass().getSimpleName());

      clone.setJointLimitLower(original.getJointLimitLower());
      clone.setJointLimitUpper(original.getJointLimitUpper());
      clone.setVelocityLimits(original.getVelocityLimitLower(), original.getVelocityLimitUpper());
      clone.setEffortLimits(original.getEffortLimitLower(), original.getEffortLimitUpper());
      return clone;
   }

   private static RigidBodyBasics cloneRigidBody(RigidBodyBasics original, String cloneSuffix, JointBasics parentJointOfClone)
   {
      FramePoint3D comOffset = new FramePoint3D();
      original.getCenterOfMass(comOffset);
      comOffset.changeFrame(original.getParentJoint().getFrameAfterJoint());
      String nameOriginal = original.getName();
      Matrix3D massMomentOfInertiaPartCopy = new Matrix3D(original.getInertia().getMomentOfInertia());
      double mass = original.getInertia().getMass();
      Vector3D comOffsetCopy = new Vector3D(comOffset);
      RigidBodyBasics clone = new RigidBody(nameOriginal + cloneSuffix, parentJointOfClone, massMomentOfInertiaPartCopy, mass, comOffsetCopy);
      return clone;
   }

   public static SpatialAcceleration createGravitationalSpatialAcceleration(RigidBodyBasics rootBody, double gravity)
   {
      Vector3D gravitationalAcceleration = new Vector3D(0.0, 0.0, gravity);
      Vector3D zero = new Vector3D();
      SpatialAcceleration rootAcceleration = new SpatialAcceleration(rootBody.getBodyFixedFrame(), ReferenceFrame.getWorldFrame(), rootBody.getBodyFixedFrame(),
                                                                     zero, gravitationalAcceleration);

      return rootAcceleration;
   }

   public static void computeIndicesForJoint(JointBasics[] jointsInOrder, TIntArrayList listToPackIndices, JointBasics... jointsToComputeIndicesFor)
   {
      int startIndex = 0;
      for (int i = 0; i < jointsInOrder.length; i++)
      {
         int nDegreesOfFreedom = jointsInOrder[i].getDegreesOfFreedom();

         for (int j = 0; j < jointsToComputeIndicesFor.length; j++)
         {
            if (jointsInOrder[i] == jointsToComputeIndicesFor[j])
            {
               for (int k = startIndex; k < startIndex + nDegreesOfFreedom; k++)
               {
                  listToPackIndices.add(k);
               }
            }
         }

         startIndex += nDegreesOfFreedom;
      }
   }

   public static void computeIndexForJoint(List<? extends JointReadOnly> jointsInOrder, TIntArrayList listToPackIndices, JointReadOnly jointToComputeIndicesFor)
   {
      int startIndex = 0;
      for (int i = 0; i < jointsInOrder.size(); i++)
      {
         JointReadOnly joint = jointsInOrder.get(i);
         int nDegreesOfFreedom = joint.getDegreesOfFreedom();

         if (joint == jointToComputeIndicesFor)
         {
            for (int k = startIndex; k < startIndex + nDegreesOfFreedom; k++)
            {
               listToPackIndices.add(k);
            }
         }

         startIndex += nDegreesOfFreedom;
      }
   }

   public static void computeIndexForJoint(JointReadOnly[] jointsInOrder, TIntArrayList listToPackIndices, JointReadOnly jointToComputeIndicesFor)
   {
      int startIndex = 0;
      for (int i = 0; i < jointsInOrder.length; i++)
      {
         int nDegreesOfFreedom = jointsInOrder[i].getDegreesOfFreedom();

         if (jointsInOrder[i] == jointToComputeIndicesFor)
         {
            for (int k = startIndex; k < startIndex + nDegreesOfFreedom; k++)
            {
               listToPackIndices.add(k);
            }
         }

         startIndex += nDegreesOfFreedom;
      }
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

   public static int computeGeometricJacobianHashCode(JointBasics joints[], ReferenceFrame jacobianFrame, boolean allowChangeFrame)
   {
      int jointsHashCode = 1;
      for (JointBasics joint : joints)
      {
         jointsHashCode = 31 * jointsHashCode + joint.hashCode();
      }
      if (!allowChangeFrame)
         return 31 * jointsHashCode + jacobianFrame.hashCode();
      else
         return jointsHashCode;
   }

   public static int computeGeometricJacobianHashCode(JointBasics joints[], int firstIndex, int lastIndex, ReferenceFrame jacobianFrame,
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

   /**
    * Will return the {@code numberOfBodies}'eth parent of the provided {@code startBody}. E.g. if
    * {@code numberOfBodies == 1} this will return the parent of the {@code startBody} and so on.
    * 
    * @throws RuntimeException if the body chain is not long enough to reach the desired parent.
    * @param startBody the body to start at.
    * @param numberOfBodies the amount of steps to go up the body chain.
    * @return the {@link RigidBodyBasics} that is {@code numberOfBodies} higher up the rigid body chain
    *         then the {@code startBody}.
    */
   public static RigidBodyBasics goUpBodyChain(RigidBodyBasics startBody, int numberOfBodies)
   {
      if (numberOfBodies == 0)
      {
         return startBody;
      }
      JointBasics parentJoint = startBody.getParentJoint();
      if (parentJoint == null)
      {
         throw new RuntimeException("Reached root body. Can not move up the chain any further.");
      }
      return goUpBodyChain(parentJoint.getPredecessor(), numberOfBodies - 1);
   }
}