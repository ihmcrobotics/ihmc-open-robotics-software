package us.ihmc.robotics.screwTheory;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.ejml.data.DenseMatrix64F;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.TransformTools;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.nameBasedHashCode.NameBasedHashCodeTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class ScrewTools
{
   public static RevoluteJoint addRevoluteJoint(String jointName, RigidBody parentBody, Vector3D jointOffset, Vector3D jointAxis)
   {
      RigidBodyTransform transformToParent = new RigidBodyTransform();
      transformToParent.setTranslationAndIdentityRotation(jointOffset);

      return addRevoluteJoint(jointName, parentBody, transformToParent, jointAxis);
   }

   public static RevoluteJoint addRevoluteJoint(String jointName, RigidBody parentBody, RigidBodyTransform transformToParent, Vector3D jointAxis)
   {
      String beforeJointName = "before" + jointName;

      ReferenceFrame parentFrame;
      if (parentBody.isRootBody())
         parentFrame = parentBody.getBodyFixedFrame();
      else
         parentFrame = parentBody.getParentJoint().getFrameAfterJoint();

      ReferenceFrame frameBeforeJoint = createOffsetFrame(parentFrame, transformToParent, beforeJointName);

      String afterJointName = jointName;
      RevoluteJoint joint = new RevoluteJoint(afterJointName, parentBody, frameBeforeJoint, new FrameVector(frameBeforeJoint, jointAxis));

      return joint;
   }

   public static PassiveRevoluteJoint addPassiveRevoluteJoint(String jointName, RigidBody parentBody, Vector3D jointOffset, Vector3D jointAxis, boolean isPartOfClosedKinematicLoop)
   {
      return addPassiveRevoluteJoint(jointName, parentBody, TransformTools.createTranslationTransform(jointOffset), jointAxis, isPartOfClosedKinematicLoop);
   }

   public static PassiveRevoluteJoint addPassiveRevoluteJoint(String jointName, RigidBody parentBody, RigidBodyTransform transformToParent, Vector3D jointAxis, boolean isPartOfClosedKinematicLoop)
   {
      String beforeJointName = "before" + jointName;

      ReferenceFrame parentFrame;
      if (parentBody.isRootBody())
         parentFrame = parentBody.getBodyFixedFrame();
      else
         parentFrame = parentBody.getParentJoint().getFrameAfterJoint();

      ReferenceFrame frameBeforeJoint = createOffsetFrame(parentFrame, transformToParent, beforeJointName);

      String afterJointName = jointName;

      return new PassiveRevoluteJoint(afterJointName, parentBody, frameBeforeJoint, new FrameVector(frameBeforeJoint, jointAxis), isPartOfClosedKinematicLoop);
   }

   public static PrismaticJoint addPrismaticJoint(String jointName, RigidBody parentBody, Vector3D jointOffset, Vector3D parentJointAxis)
   {
      return addPrismaticJoint(jointName, parentBody, TransformTools.createTranslationTransform(jointOffset), parentJointAxis);
   }

   public static PrismaticJoint addPrismaticJoint(String jointName, RigidBody parentBody, RigidBodyTransform transformToParent, Vector3D jointAxis)
   {
      String beforeJointName = "before" + jointName;

      ReferenceFrame parentFrame;
      if (parentBody.isRootBody())
         parentFrame = parentBody.getBodyFixedFrame();
      else
         parentFrame = parentBody.getParentJoint().getFrameAfterJoint();

      ReferenceFrame frameBeforeJoint = createOffsetFrame(parentFrame, transformToParent, beforeJointName);

      String afterJointName = jointName;
      PrismaticJoint joint = new PrismaticJoint(afterJointName, parentBody, frameBeforeJoint, new FrameVector(frameBeforeJoint, jointAxis));

      return joint;
   }

   public static RigidBody addRigidBody(String name, InverseDynamicsJoint parentJoint, Matrix3D momentOfInertia, double mass, Vector3D centerOfMassOffset)
   {
      String comFrameName = name + "CoM";
      ReferenceFrame comFrame = createOffsetFrame(parentJoint.getFrameAfterJoint(), centerOfMassOffset, comFrameName);
      RigidBodyInertia inertia = new RigidBodyInertia(comFrame, momentOfInertia, mass);
      RigidBody ret = new RigidBody(name, inertia, parentJoint);

      return ret;
   }

   public static RigidBody addRigidBody(String name, InverseDynamicsJoint parentJoint, Matrix3D momentOfInertia, double mass, RigidBodyTransform inertiaPose)
   {
      String comFrameName = name + "CoM";
      ReferenceFrame comFrame = createOffsetFrame(parentJoint.getFrameAfterJoint(), inertiaPose, comFrameName);
      RigidBodyInertia inertia = new RigidBodyInertia(comFrame, momentOfInertia, mass);
      RigidBody ret = new RigidBody(name, inertia, parentJoint);

      return ret;
   }

   private static ReferenceFrame createOffsetFrame(ReferenceFrame parentFrame, Vector3D offset, String frameName)
   {
      RigidBodyTransform transformToParent = new RigidBodyTransform();
      transformToParent.setTranslationAndIdentityRotation(offset);

      return createOffsetFrame(parentFrame, transformToParent, frameName);
   }

   public static ReferenceFrame createOffsetFrame(ReferenceFrame parentFrame, RigidBodyTransform transformToParent, String frameName)
   {
      ReferenceFrame beforeJointFrame = ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent(frameName, parentFrame, transformToParent);

      return beforeJointFrame;
   }

   public static RigidBody[] computeSuccessors(InverseDynamicsJoint... joints)
   {
      RigidBody[] ret = new RigidBody[joints.length];
      for (int i = 0; i < joints.length; i++)
      {
         InverseDynamicsJoint joint = joints[i];
         ret[i] = joint.getSuccessor();
      }
      return ret;
   }

   public static RigidBody[] computeSubtreeSuccessors(InverseDynamicsJoint... joints)
   {
      ArrayList<RigidBody> rigidBodySuccessors = new ArrayList<RigidBody>();
      ArrayList<RigidBody> rigidBodyStack = new ArrayList<RigidBody>();
      for (InverseDynamicsJoint joint : joints)
      {
         rigidBodyStack.add(joint.getPredecessor());
      }
      while (!rigidBodyStack.isEmpty())
      {
         RigidBody currentBody = rigidBodyStack.remove(0);
         List<InverseDynamicsJoint> childrenJoints = currentBody.getChildrenJoints();
         for (InverseDynamicsJoint joint : childrenJoints)
         {
            rigidBodyStack.add(joint.getSuccessor());
            rigidBodySuccessors.add(joint.getSuccessor());
         }
      }
      RigidBody[] ret = new RigidBody[rigidBodySuccessors.size()];

      return rigidBodySuccessors.toArray(ret);
   }

   public static RigidBody[] computeRigidBodiesAfterThisJoint(InverseDynamicsJoint... joints)
   {
      ArrayList<RigidBody> rigidBodySuccessors = new ArrayList<RigidBody>();
      computeRigidBodiesAfterThisJoint(rigidBodySuccessors, joints);
      RigidBody[] ret = new RigidBody[rigidBodySuccessors.size()];

      return rigidBodySuccessors.toArray(ret);
   }

   public static void computeRigidBodiesAfterThisJoint(ArrayList<RigidBody> rigidBodySuccessorsToPack, InverseDynamicsJoint... joints)
   {
      ArrayList<InverseDynamicsJoint> jointStack = new ArrayList<InverseDynamicsJoint>();
      for (InverseDynamicsJoint joint : joints)
      {
         jointStack.add(joint);
      }
      while (!jointStack.isEmpty())
      {
         InverseDynamicsJoint currentJoint = jointStack.remove(0);
         rigidBodySuccessorsToPack.add(currentJoint.getSuccessor());
         RigidBody currentBody = currentJoint.getSuccessor();
         List<InverseDynamicsJoint> childrenJoints = currentBody.getChildrenJoints();
         for (InverseDynamicsJoint joint : childrenJoints)
         {
            jointStack.add(joint);
         }
      }
   }

   public static void computeRigidBodiesFromRootToThisJoint(ArrayList<RigidBody> rigidBodySuccessorsToPack, InverseDynamicsJoint joint)
   {
      RigidBody predecessorBody = joint.getPredecessor();
      if (predecessorBody == null)
         return;
      if (predecessorBody.isRootBody())
         return;

      rigidBodySuccessorsToPack.add(predecessorBody);
      InverseDynamicsJoint parentJoint = predecessorBody.getParentJoint();
      if (parentJoint == null)
         return;

      computeRigidBodiesFromRootToThisJoint(rigidBodySuccessorsToPack, parentJoint);
   }

   public static RigidBody[] computeSubtreeSuccessors(RigidBody... bodies)
   {
      return computeSuccessors(computeSubtreeJoints(bodies));
   }

   public static RigidBody[] computeSubtreeSuccessors(Set<InverseDynamicsJoint> jointsToExclude, RigidBody... bodies)
   {
      ArrayList<RigidBody> rigidBodySuccessors = new ArrayList<RigidBody>();
      ArrayList<RigidBody> rigidBodyStack = new ArrayList<RigidBody>();
      for (RigidBody body : bodies)
      {
         rigidBodyStack.add(body);
      }
      while (!rigidBodyStack.isEmpty())
      {
         RigidBody currentBody = rigidBodyStack.remove(0);
         List<InverseDynamicsJoint> childrenJoints = currentBody.getChildrenJoints();
         for (InverseDynamicsJoint joint : childrenJoints)
         {
            if (!jointsToExclude.contains(joint))
            {
               rigidBodyStack.add(joint.getSuccessor());
               rigidBodySuccessors.add(joint.getSuccessor());
            }
         }
      }

      RigidBody[] ret = new RigidBody[rigidBodySuccessors.size()];

      return rigidBodySuccessors.toArray(ret);
   }

   public static RigidBody[] computeSupportAndSubtreeSuccessors(RigidBody... bodies)
   {
      return computeSuccessors(computeSupportAndSubtreeJoints(bodies));
   }

   public static InverseDynamicsJoint[] computeSupportAndSubtreeJoints(RigidBody... bodies)
   {
      Set<InverseDynamicsJoint> ret = new LinkedHashSet<InverseDynamicsJoint>();
      for (RigidBody body : bodies)
      {
         ret.addAll(Arrays.asList(computeSupportJoints(body)));
         ret.addAll(Arrays.asList(computeSubtreeJoints(body)));
      }
      return ret.toArray(new InverseDynamicsJoint[ret.size()]);
   }

   public static InverseDynamicsJoint[] computeSupportJoints(RigidBody... bodies)
   {
      Set<InverseDynamicsJoint> supportSet = new LinkedHashSet<InverseDynamicsJoint>();
      for (RigidBody rigidBody : bodies)
      {
         RigidBody rootBody = getRootBody(rigidBody);
         InverseDynamicsJoint[] jointPath = createJointPath(rootBody, rigidBody);
         supportSet.addAll(Arrays.asList(jointPath));
      }

      return supportSet.toArray(new InverseDynamicsJoint[supportSet.size()]);
   }

   public static InverseDynamicsJoint[] computeSubtreeJoints(RigidBody... rootBodies)
   {
      return computeSubtreeJoints(Arrays.asList(rootBodies));
   }

   public static InverseDynamicsJoint[] computeSubtreeJoints(List<RigidBody> rootBodies)
   {
      ArrayList<InverseDynamicsJoint> subtree = new ArrayList<InverseDynamicsJoint>();
      ArrayList<RigidBody> rigidBodyStack = new ArrayList<RigidBody>();
      rigidBodyStack.addAll(rootBodies);

      while (!rigidBodyStack.isEmpty())
      {
         RigidBody currentBody = rigidBodyStack.remove(0);
         List<InverseDynamicsJoint> childrenJoints = currentBody.getChildrenJoints();
         for (InverseDynamicsJoint joint : childrenJoints)
         {
            RigidBody successor = joint.getSuccessor();
            rigidBodyStack.add(successor);
            subtree.add(joint);
         }
      }

      InverseDynamicsJoint[] ret = new InverseDynamicsJoint[subtree.size()];
      return subtree.toArray(ret);
   }

   public static RigidBody getRootBody(RigidBody body)
   {
      RigidBody ret = body;
      while (ret.getParentJoint() != null)
      {
         ret = ret.getParentJoint().getPredecessor();
      }
      return ret;
   }

   public static int[] createParentMap(RigidBody[] allRigidBodiesInOrder)
   {
      int[] parentMap = new int[allRigidBodiesInOrder.length];
      List<RigidBody> rigidBodiesInOrderList = Arrays.asList(allRigidBodiesInOrder); // this doesn't have to be fast
      for (int i = 0; i < allRigidBodiesInOrder.length; i++)
      {
         RigidBody currentBody = allRigidBodiesInOrder[i];
         if (currentBody.isRootBody())
         {
            parentMap[i] = -1;
         }
         else
         {
            RigidBody parentBody = currentBody.getParentJoint().getPredecessor();
            parentMap[i] = rigidBodiesInOrderList.indexOf(parentBody);
         }
      }

      return parentMap;
   }

   public static DenseMatrix64F getTauMatrix(InverseDynamicsJoint[] jointsInOrder)
   {
      int size = 0;
      for (InverseDynamicsJoint joint : jointsInOrder)
      {
         size += joint.getDegreesOfFreedom();
      }

      DenseMatrix64F tempMatrix = new DenseMatrix64F(InverseDynamicsJoint.maxDoF, 1);
      DenseMatrix64F ret = new DenseMatrix64F(size, 1);
      int startIndex = 0;
      for (InverseDynamicsJoint joint : jointsInOrder)
      {
         int endIndex = startIndex + joint.getDegreesOfFreedom() - 1;
         joint.getTauMatrix(tempMatrix);

         MatrixTools.setMatrixBlock(ret, startIndex, 0, tempMatrix, 0, 0, joint.getDegreesOfFreedom(), 1, 1.0);

         startIndex = endIndex + 1;
      }

      return ret;
   }

   public static OneDoFJoint[] createOneDoFJointPath(RigidBody start, RigidBody end)
   {
      return filterJoints(createJointPath(start, end), OneDoFJoint.class);
   }

   public static InverseDynamicsJoint[] createJointPath(RigidBody start, RigidBody end)
   {
      boolean flip = false;
      RigidBody descendant = start;
      RigidBody ancestor = end;
      int pathLength = computeDistanceToAncestor(descendant, ancestor);
      if (pathLength < 0)
      {
         flip = true;
         descendant = end;
         ancestor = start;
         pathLength = computeDistanceToAncestor(end, start);
      }

      InverseDynamicsJoint[] ret = new InverseDynamicsJoint[pathLength];
      RigidBody currentBody = descendant;
      int i = 0;
      while (currentBody != ancestor)
      {
         int j = flip ? pathLength - 1 - i : i;
         InverseDynamicsJoint parentJoint = currentBody.getParentJoint();
         ret[j] = parentJoint;
         currentBody = parentJoint.getPredecessor();
         i++;
      }

      return ret;
   }

   /**
    * Compute and pack the joint path between two RigidBody in the jointPathToPack.
    * Use the method {@link #computeDistanceToAncestor(RigidBody, RigidBody)} to get the size of the Array to provide.
    * @param jointPathToPack
    * @param start
    * @param end
    * @return the length of the joint path, returns -1 if the the given jointPathToPack is too small.
    */
   public static int createJointPath(InverseDynamicsJoint[] jointPathToPack, RigidBody start, RigidBody end)
   {
      boolean flip = false;
      RigidBody descendant = start;
      RigidBody ancestor = end;
      int pathLength = computeDistanceToAncestor(descendant, ancestor);
      if (pathLength < 0)
      {
         flip = true;
         descendant = end;
         ancestor = start;
         pathLength = computeDistanceToAncestor(end, start);
      }

      if (jointPathToPack == null || jointPathToPack.length < pathLength)
         return -1;

      RigidBody currentBody = descendant;
      int i = 0;
      while (currentBody != ancestor)
      {
         int j = flip ? pathLength - 1 - i : i;
         InverseDynamicsJoint parentJoint = currentBody.getParentJoint();
         jointPathToPack[j] = parentJoint;
         currentBody = parentJoint.getPredecessor();
         i++;
      }

      for (int k = pathLength; k < jointPathToPack.length; k++)
         jointPathToPack[k] = null;

      return pathLength;
   }

   public static OneDoFJoint[] cloneOneDoFJointPath(RigidBody start, RigidBody end)
   {
      return cloneJointPathAndFilter(createOneDoFJointPath(start, end), OneDoFJoint.class);
   }

   public static OneDoFJoint[] cloneOneDoFJointPath(OneDoFJoint[] oneDoFJoints)
   {
      return cloneJointPathAndFilter(oneDoFJoints, OneDoFJoint.class);
   }

   public static <T extends InverseDynamicsJoint> T[] cloneJointPathAndFilter(T[] joints, Class<T> clazz)
   {
      return filterJoints(cloneJointPath(joints), clazz);
   }

   public static <T extends InverseDynamicsJoint> T[] cloneJointPathAndFilter(T[] joints, Class<T> clazz, String suffix)
   {
      return filterJoints(cloneJointPath(joints, suffix), clazz);
   }

   public static InverseDynamicsJoint[] cloneJointPath(InverseDynamicsJoint[] inverseDynamicsJoints)
   {
      String clonedJointNameSuffix = "Copy";

      return cloneJointPath(inverseDynamicsJoints, clonedJointNameSuffix);
   }

   public static InverseDynamicsJoint[] cloneJointPath(InverseDynamicsJoint[] inverseDynamicsJoints, String suffix)
   {
      InverseDynamicsJoint[] cloned = new InverseDynamicsJoint[inverseDynamicsJoints.length];
      Map<RigidBody, RigidBody> originalToClonedRigidBodies = new HashMap<>();

      for (int i = 0; i < inverseDynamicsJoints.length; i++)
      {
         if (inverseDynamicsJoints[i] instanceof OneDoFJoint)
         {
            OneDoFJoint jointOriginal = (OneDoFJoint) inverseDynamicsJoints[i];

            RigidBody predecessorOriginal = jointOriginal.getPredecessor();
            RigidBody predecessorCopy = originalToClonedRigidBodies.get(predecessorOriginal);

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
            RigidBody rootBody = jointOriginal.getPredecessor();

            if (rootBody.getParentJoint() != null)
               throw new RuntimeException("The SixDoFJoint predecessor is not the root body. Case not handled.");

            String rootBodyNameOriginal = rootBody.getName();
            ReferenceFrame rootBodyFrame = rootBody.getBodyFixedFrame();
            RigidBody rootBodyCopy = new RigidBody(rootBodyNameOriginal + suffix, rootBodyFrame);
            originalToClonedRigidBodies.put(rootBody, rootBodyCopy);

            String jointNameOriginal = jointOriginal.getName();
            SixDoFJoint jointCopy = new SixDoFJoint(jointNameOriginal + suffix, rootBodyCopy, rootBodyFrame);
            cloned[i] = jointCopy;
         }
         else
         {
            throw new RuntimeException("Not implemented for joints of the type: " + inverseDynamicsJoints[i].getClass().getSimpleName());
         }

         RigidBody successorOriginal = inverseDynamicsJoints[i].getSuccessor();
         RigidBody successorCopy = originalToClonedRigidBodies.get(successorOriginal);
         if (successorCopy == null)
         {
            successorCopy = cloneRigidBody(successorOriginal, suffix, cloned[i]);
            originalToClonedRigidBodies.put(successorOriginal, successorCopy);
         }

         cloned[i].setSuccessor(successorCopy);
      }
      return cloned;
   }

   public static <T extends InverseDynamicsJoint> T[] cloneJointPathDisconnectedFromOriginalRobot(T[] joints, Class<T> clazz, String suffix, ReferenceFrame rootBodyFrame)
   {
      return filterJoints(cloneJointPathDisconnectedFromOriginalRobot(joints, suffix, rootBodyFrame), clazz);
   }

   public static InverseDynamicsJoint[] cloneJointPathDisconnectedFromOriginalRobot(InverseDynamicsJoint[] inverseDynamicsJoints, String suffix, ReferenceFrame rootBodyFrame)
   {
      InverseDynamicsJoint[] cloned = new InverseDynamicsJoint[inverseDynamicsJoints.length];

      for (int i = 0; i < inverseDynamicsJoints.length; i++)
      {
         if (inverseDynamicsJoints[i] instanceof RevoluteJoint)
         {
            RevoluteJoint jointOriginal = (RevoluteJoint) inverseDynamicsJoints[i];

            RigidBody predecessorOriginal = jointOriginal.getPredecessor();
            RigidBody predecessorCopy;

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

   private static OneDoFJoint cloneOneDoFJoint(OneDoFJoint original, String cloneSuffix, RigidBody clonePredecessor)
   {
      String jointNameOriginal = original.getName();
      RigidBodyTransform jointTransform = original.getOffsetTransform3D();
      Vector3D jointAxisCopy = original.getJointAxis().getVectorCopy();
      OneDoFJoint clone;

      if (original instanceof RevoluteJoint)
         clone = ScrewTools.addRevoluteJoint(jointNameOriginal + cloneSuffix, clonePredecessor, jointTransform, jointAxisCopy);
      else if (original instanceof PrismaticJoint)
         clone = ScrewTools.addPrismaticJoint(jointNameOriginal + cloneSuffix, clonePredecessor, jointTransform, jointAxisCopy);
      else
         throw new RuntimeException("Unhandled type of " + OneDoFJoint.class.getSimpleName() + ": " + original.getClass().getSimpleName());
         
      clone.setJointLimitLower(original.getJointLimitLower());
      clone.setJointLimitUpper(original.getJointLimitUpper());
      return clone;
   }

   private static RigidBody cloneRigidBody(RigidBody original, String cloneSuffix, InverseDynamicsJoint parentJointOfClone)
   {
      FramePoint comOffset = new FramePoint();
      original.getCoMOffset(comOffset);
      comOffset.changeFrame(original.getParentJoint().getFrameAfterJoint());
      String nameOriginal = original.getName();
      Matrix3D massMomentOfInertiaPartCopy = original.getInertia().getMassMomentOfInertiaPartCopy();
      double mass = original.getInertia().getMass();
      Vector3D comOffsetCopy = comOffset.getVectorCopy();
      RigidBody clone = ScrewTools.addRigidBody(nameOriginal + cloneSuffix, parentJointOfClone, massMomentOfInertiaPartCopy,
                                                        mass, comOffsetCopy);
      return clone;
   }

   public static boolean isAncestor(RigidBody candidateDescendant, RigidBody ancestor)
   {
      RigidBody currentBody = candidateDescendant;
      while (!currentBody.isRootBody())
      {
         if (currentBody == ancestor)
         {
            return true;
         }
         currentBody = currentBody.getParentJoint().getPredecessor();
      }

      return currentBody == ancestor;
   }

   public static int computeDistanceToAncestor(RigidBody descendant, RigidBody ancestor)
   {
      int ret = 0;
      RigidBody currentBody = descendant;
      while (!currentBody.isRootBody() && (currentBody != ancestor))
      {
         ret++;
         currentBody = currentBody.getParentJoint().getPredecessor();
      }

      if (currentBody != ancestor)
         ret = -1;

      return ret;
   }

   public static void getJointVelocitiesMatrix(InverseDynamicsJoint[] joints, DenseMatrix64F jointVelocitiesMatrixToPack)
   {
      int rowStart = 0;
      for (InverseDynamicsJoint joint : joints)
      {
         int dof = joint.getDegreesOfFreedom();
         joint.getVelocityMatrix(jointVelocitiesMatrixToPack, rowStart);
         rowStart += dof;
      }
   }

   public static void getJointVelocitiesMatrix(Iterable<? extends InverseDynamicsJoint> joints, DenseMatrix64F jointVelocitiesMatrixToPack)
   {
      int rowStart = 0;
      for (InverseDynamicsJoint joint : joints)
      {
         int dof = joint.getDegreesOfFreedom();
         joint.getVelocityMatrix(jointVelocitiesMatrixToPack, rowStart);
         rowStart += dof;
      }
   }

   public static void getDesiredJointAccelerationsMatrix(InverseDynamicsJoint[] joints, DenseMatrix64F desiredJointAccelerationsMatrixToPack)
   {
      int rowStart = 0;
      for (InverseDynamicsJoint joint : joints)
      {
         int dof = joint.getDegreesOfFreedom();
         joint.getDesiredAccelerationMatrix(desiredJointAccelerationsMatrixToPack, rowStart);
         rowStart += dof;
      }
   }

   public static int computeDegreesOfFreedom(InverseDynamicsJoint[] jointList)
   {
      int ret = 0;
      for (InverseDynamicsJoint joint : jointList)
      {
         ret += joint.getDegreesOfFreedom();
      }

      return ret;
   }

   public static int computeDegreesOfFreedom(Iterable<? extends InverseDynamicsJoint> jointList)
   {
      int ret = 0;
      for (InverseDynamicsJoint joint : jointList)
      {
         ret += joint.getDegreesOfFreedom();
      }

      return ret;
   }

   public static int computeDegreesOfFreedom(List<? extends InverseDynamicsJoint> jointList)
   {
      int ret = 0;
      for (int i = 0; i < jointList.size(); i++)
      {
         ret += jointList.get(i).getDegreesOfFreedom();
      }
      return ret;
   }

   public static SpatialAccelerationVector createGravitationalSpatialAcceleration(RigidBody rootBody, double gravity)
   {
      Vector3D gravitationalAcceleration = new Vector3D(0.0, 0.0, gravity);
      Vector3D zero = new Vector3D();
      SpatialAccelerationVector rootAcceleration = new SpatialAccelerationVector(rootBody.getBodyFixedFrame(), ReferenceFrame.getWorldFrame(), rootBody.getBodyFixedFrame(), gravitationalAcceleration, zero);

      return rootAcceleration;
   }

   public static void getJointPositions(InverseDynamicsJoint[] joints, DenseMatrix64F jointPositionsToPack)
   {
      int rowStart = 0;
      for (InverseDynamicsJoint joint : joints)
      {
         joint.getConfigurationMatrix(jointPositionsToPack, rowStart);
         rowStart += joint.getDegreesOfFreedom();
         if (joint instanceof SixDoFJoint || joint instanceof SphericalJoint)
            rowStart++; // Because of stupid quaternions
      }
   }

   public static void getJointDesiredPositions(OneDoFJoint[] joints, DenseMatrix64F jointPositionsToPack)
   {
      int rowStart = 0;
      for (OneDoFJoint joint : joints)
      {
         jointPositionsToPack.set(rowStart, 0, joint.getqDesired());
         rowStart += joint.getDegreesOfFreedom();
      }
   }

   public static void setJointPositions(InverseDynamicsJoint[] joints, DenseMatrix64F jointPositions)
   {
      int rowStart = 0;
      for (InverseDynamicsJoint joint : joints)
      {
         joint.setConfiguration(jointPositions, rowStart);
         rowStart += joint.getDegreesOfFreedom();
         if (joint instanceof SixDoFJoint || joint instanceof SphericalJoint)
            rowStart++; // Because of stupid quaternions
      }
   }

   public static void setDesiredJointPositions(OneDoFJoint[] joints, DenseMatrix64F jointPositions)
   {
      int rowStart = 0;
      for (OneDoFJoint joint : joints)
      {
         joint.setqDesired(jointPositions.get(rowStart, 0));
         rowStart += joint.getDegreesOfFreedom();
      }
   }

   public static void setDesiredJointVelocities(OneDoFJoint[] joints, DenseMatrix64F jointVelocities)
   {
      int rowStart = 0;
      for (OneDoFJoint joint : joints)
      {
         joint.setQdDesired(jointVelocities.get(rowStart, 0));
         rowStart += joint.getDegreesOfFreedom();
      }
   }

   public static void setDesiredAccelerations(InverseDynamicsJoint[] jointList, DenseMatrix64F jointAccelerations)
   {
      int rowStart = 0;
      for (InverseDynamicsJoint joint : jointList)
      {
         joint.setDesiredAcceleration(jointAccelerations, rowStart);
         rowStart += joint.getDegreesOfFreedom();
      }
   }

   public static void setVelocities(InverseDynamicsJoint[] jointList, DenseMatrix64F jointVelocities)
   {
      int rowStart = 0;
      for (InverseDynamicsJoint joint : jointList)
      {
         joint.setVelocity(jointVelocities, rowStart);
         rowStart += joint.getDegreesOfFreedom();
      }
   }

   public static void setJointAccelerations(OneDoFJoint[] jointList, DenseMatrix64F jointAccelerations)
   {
      int rowStart = 0;
      for (OneDoFJoint joint : jointList)
      {
         joint.setQdd(jointAccelerations.get(rowStart, 0));
         rowStart += joint.getDegreesOfFreedom();
      }
   }

   public static void computeIndicesForJoint(InverseDynamicsJoint[] jointsInOrder, TIntArrayList listToPackIndices, InverseDynamicsJoint... jointsToComputeIndicesFor)
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

   public static void computeIndexForJoint(InverseDynamicsJoint[] jointsInOrder, TIntArrayList listToPackIndices, InverseDynamicsJoint jointToComputeIndicesFor)
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

   public static RevoluteJoint[] extractRevoluteJoints(InverseDynamicsJoint[] allJoints)
   {
      if (allJoints == null)
         return null;

      ArrayList<RevoluteJoint> revoluteJointsList = new ArrayList<RevoluteJoint>();
      for (InverseDynamicsJoint joint : allJoints)
      {
         if (joint instanceof RevoluteJoint)
            revoluteJointsList.add((RevoluteJoint) joint);
      }

      RevoluteJoint[] revoluteJointArray = new RevoluteJoint[revoluteJointsList.size()];
      revoluteJointsList.toArray(revoluteJointArray);

      return revoluteJointArray;
   }

   public static <T extends InverseDynamicsJoint> int computeNumberOfJointsOfType(Class<T> clazz, InverseDynamicsJoint[] joints)
   {
      int ret = 0;
      for (InverseDynamicsJoint joint : joints)
      {
         if (clazz.isAssignableFrom(joint.getClass()))
            ret++;
      }

      return ret;
   }

   public static <T extends InverseDynamicsJoint> T[] filterJoints(InverseDynamicsJoint[] source, Class<T> clazz)
   {
      @SuppressWarnings("unchecked")
      T[] retArray = (T[]) Array.newInstance(clazz, ScrewTools.computeNumberOfJointsOfType(clazz, source));
      filterJoints(source, retArray, clazz);
      return retArray;
   }

   @SuppressWarnings("unchecked")
   public static <T extends InverseDynamicsJoint> void filterJoints(InverseDynamicsJoint[] source, T[] dest, Class<T> clazz)
   {
      int index = 0;
      for (InverseDynamicsJoint joint : source)
      {
         if (clazz.isAssignableFrom(joint.getClass()))
         {
            dest[index++] = (T) joint;
         }
      }
   }

   public static <T extends InverseDynamicsJoint> List<T> filterJoints(List<InverseDynamicsJoint> source, Class<T> clazz)
   {
      List<T> retList = new ArrayList<>();
      filterJoints(source, retList, clazz);
      return retList;
   }

   @SuppressWarnings("unchecked")
   public static <T extends InverseDynamicsJoint> void filterJoints(List<InverseDynamicsJoint> source, List<T> dest, Class<T> clazz)
   {
      for (InverseDynamicsJoint joint : source)
      {
         if (clazz.isAssignableFrom(joint.getClass()))
         {
            dest.add((T) joint);
         }
      }
   }

   public static InverseDynamicsJoint[] findJointsWithNames(InverseDynamicsJoint[] allJoints, String... jointNames)
   {
      if (jointNames == null)
         return null;

      InverseDynamicsJoint[] ret = new InverseDynamicsJoint[jointNames.length];
      int index = 0;
      for (InverseDynamicsJoint joint : allJoints)
      {
         for (String jointName : jointNames)
         {
            if (joint.getName().equals(jointName))
               ret[index++] = joint;
         }
      }

      if (index != jointNames.length)
         throw new RuntimeException("Not all joints could be found");

      return ret;
   }

   public static RigidBody[] findRigidBodiesWithNames(RigidBody[] allBodies, String... names)
   {
      RigidBody[] ret = new RigidBody[names.length];
      int index = 0;
      for (RigidBody body : allBodies)
      {
         for (String name : names)
         {
            if (body.getName().equals(name))
               ret[index++] = body;
         }
      }

      if (index != names.length)
         throw new RuntimeException("Not all bodies could be found");

      return ret;
   }

   public static void addExternalWrenches(Map<RigidBody, Wrench> externalWrenches, Map<RigidBody, Wrench> wrenchMapToAdd)
   {
      for (RigidBody rigidBody : wrenchMapToAdd.keySet())
      {
         Wrench externalWrenchToCompensateFor = wrenchMapToAdd.get(rigidBody);

         Wrench externalWrench = externalWrenches.get(rigidBody);
         if (externalWrench == null)
         {
            externalWrenches.put(rigidBody, new Wrench(externalWrenchToCompensateFor));
         }
         else
         {
            externalWrench.add(externalWrenchToCompensateFor);
         }
      }
   }

   public static long computeGeometricJacobianNameBasedHashCode(InverseDynamicsJoint joints[], ReferenceFrame jacobianFrame, boolean allowChangeFrame)
   {
      long jointsHashCode = NameBasedHashCodeTools.computeArrayHashCode(joints);
      if (!allowChangeFrame)
         return NameBasedHashCodeTools.combineHashCodes(jointsHashCode, jacobianFrame);
      else
         return jointsHashCode;
   }

   public static long computeGeometricJacobianNameBasedHashCode(InverseDynamicsJoint joints[], int firstIndex, int lastIndex, ReferenceFrame jacobianFrame, boolean allowChangeFrame)
   {
      long jointsHashCode = NameBasedHashCodeTools.computeSubArrayHashCode(joints, firstIndex, lastIndex);
      if (!allowChangeFrame)
         return NameBasedHashCodeTools.combineHashCodes(jointsHashCode, jacobianFrame);
      else
         return jointsHashCode;
   }
}