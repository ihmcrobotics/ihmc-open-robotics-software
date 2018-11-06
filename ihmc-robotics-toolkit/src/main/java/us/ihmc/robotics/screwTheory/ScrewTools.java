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
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.mecano.multiBodySystem.OneDoFJoint;
import us.ihmc.mecano.multiBodySystem.PrismaticJoint;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.SphericalJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialAcceleration;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.robotics.geometry.TransformTools;
import us.ihmc.robotics.linearAlgebra.MatrixTools;

public class ScrewTools
{
   public static RevoluteJoint addRevoluteJoint(String jointName, RigidBodyBasics parentBody, Vector3D jointOffset, Vector3D jointAxis)
   {
      RigidBodyTransform transformToParent = new RigidBodyTransform();
      transformToParent.setTranslationAndIdentityRotation(jointOffset);

      return addRevoluteJoint(jointName, parentBody, transformToParent, jointAxis);
   }

   public static RevoluteJoint addRevoluteJoint(String jointName, RigidBodyBasics parentBody, RigidBodyTransform transformToParent, Vector3D jointAxis)
   {
      return new RevoluteJoint(jointName, parentBody, transformToParent, jointAxis);
   }

   public static PassiveRevoluteJoint addPassiveRevoluteJoint(String jointName, RigidBodyBasics parentBody, Vector3D jointOffset, Vector3D jointAxis,
                                                              boolean isPartOfClosedKinematicLoop)
   {
      return addPassiveRevoluteJoint(jointName, parentBody, TransformTools.createTranslationTransform(jointOffset), jointAxis, isPartOfClosedKinematicLoop);
   }

   public static PassiveRevoluteJoint addPassiveRevoluteJoint(String jointName, RigidBodyBasics parentBody, RigidBodyTransform transformToParent, Vector3D jointAxis,
                                                              boolean isPartOfClosedKinematicLoop)
   {
      return new PassiveRevoluteJoint(jointName, parentBody, transformToParent, jointAxis, isPartOfClosedKinematicLoop);
   }

   public static PrismaticJoint addPrismaticJoint(String jointName, RigidBodyBasics parentBody, Vector3D jointOffset, Vector3D jointAxis)
   {
      return addPrismaticJoint(jointName, parentBody, TransformTools.createTranslationTransform(jointOffset), jointAxis);
   }

   public static PrismaticJoint addPrismaticJoint(String jointName, RigidBodyBasics parentBody, RigidBodyTransform transformToParent, Vector3D jointAxis)
   {
      return new PrismaticJoint(jointName, parentBody, transformToParent, jointAxis);
   }

   public static RigidBodyBasics addRigidBody(String name, JointBasics parentJoint, double Ixx, double Iyy, double Izz, double mass, Vector3D centerOfMassOffset)
   {
      Matrix3D momentOfInertia = new Matrix3D();
      momentOfInertia.setIdentity();
      momentOfInertia.setM00(Ixx);
      momentOfInertia.setM11(Iyy);
      momentOfInertia.setM22(Izz);
      return addRigidBody(name, parentJoint, momentOfInertia, mass, centerOfMassOffset);
   }

   public static RigidBodyBasics addRigidBody(String name, JointBasics parentJoint, Matrix3DReadOnly momentOfInertia, double mass,
                                        Vector3DReadOnly centerOfMassOffset)
   {
      RigidBodyTransform inertiaPose = new RigidBodyTransform();
      inertiaPose.setTranslation(centerOfMassOffset);
      return addRigidBody(name, parentJoint, momentOfInertia, mass, inertiaPose);
   }

   public static RigidBodyBasics addRigidBody(String name, JointBasics parentJoint, Matrix3DReadOnly momentOfInertia, double mass, RigidBodyTransform inertiaPose)
   {
      return new RigidBody(name, parentJoint, momentOfInertia, mass, inertiaPose);
   }

   public static RigidBodyBasics[] computeSuccessors(JointBasics... joints)
   {
      RigidBodyBasics[] ret = new RigidBodyBasics[joints.length];
      for (int i = 0; i < joints.length; i++)
      {
         JointBasics joint = joints[i];
         ret[i] = joint.getSuccessor();
      }
      return ret;
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
         List<JointBasics> childrenJoints = currentBody.getChildrenJoints();
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
         List<JointBasics> childrenJoints = currentBody.getChildrenJoints();
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
      return computeSuccessors(computeSubtreeJoints(bodies));
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
         List<JointBasics> childrenJoints = currentBody.getChildrenJoints();
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
      return computeSuccessors(computeSupportAndSubtreeJoints(bodies));
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
         RigidBodyBasics rootBody = getRootBody(rigidBody);
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
         List<JointBasics> childrenJoints = currentBody.getChildrenJoints();
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

   public static RigidBodyBasics getRootBody(RigidBodyBasics body)
   {
      RigidBodyBasics ret = body;
      while (ret.getParentJoint() != null)
      {
         ret = ret.getParentJoint().getPredecessor();
      }
      return ret;
   }

   public static int[] createParentMap(RigidBodyBasics[] allRigidBodiesInOrder)
   {
      int[] parentMap = new int[allRigidBodiesInOrder.length];
      List<RigidBodyBasics> rigidBodiesInOrderList = Arrays.asList(allRigidBodiesInOrder); // this doesn't have to be fast
      for (int i = 0; i < allRigidBodiesInOrder.length; i++)
      {
         RigidBodyBasics currentBody = allRigidBodiesInOrder[i];
         if (currentBody.isRootBody())
         {
            parentMap[i] = -1;
         }
         else
         {
            RigidBodyBasics parentBody = currentBody.getParentJoint().getPredecessor();
            parentMap[i] = rigidBodiesInOrderList.indexOf(parentBody);
         }
      }

      return parentMap;
   }

   public static DenseMatrix64F getTauMatrix(JointBasics[] jointsInOrder)
   {
      int size = 0;
      for (JointBasics joint : jointsInOrder)
      {
         size += joint.getDegreesOfFreedom();
      }

      DenseMatrix64F tempMatrix = new DenseMatrix64F(JointBasics.MAX_NUMBER_OF_DOFS, 1);
      DenseMatrix64F ret = new DenseMatrix64F(size, 1);
      int startIndex = 0;
      for (JointBasics joint : jointsInOrder)
      {
         int endIndex = startIndex + joint.getDegreesOfFreedom() - 1;
         joint.getTauMatrix(tempMatrix);

         MatrixTools.setMatrixBlock(ret, startIndex, 0, tempMatrix, 0, 0, joint.getDegreesOfFreedom(), 1, 1.0);

         startIndex = endIndex + 1;
      }

      return ret;
   }

   public static OneDoFJoint[] createOneDoFJointPath(RigidBodyBasics start, RigidBodyBasics end)
   {
      return filterJoints(createJointPath(start, end), OneDoFJoint.class);
   }

   public static JointBasics[] createJointPath(RigidBodyBasics start, RigidBodyBasics end)
   {
      boolean flip = false;
      RigidBodyBasics descendant = start;
      RigidBodyBasics ancestor = end;
      int pathLength = computeDistanceToAncestor(descendant, ancestor);
      if (pathLength < 0)
      {
         flip = true;
         descendant = end;
         ancestor = start;
         pathLength = computeDistanceToAncestor(end, start);
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
    * {@link #computeDistanceToAncestor(RigidBodyBasics, RigidBodyBasics)} to get the size of the Array to provide.
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

   public static OneDoFJoint[] cloneOneDoFJointPath(RigidBodyBasics start, RigidBodyBasics end)
   {
      return cloneJointPathAndFilter(createOneDoFJointPath(start, end), OneDoFJoint.class);
   }

   public static OneDoFJoint[] cloneOneDoFJointPath(OneDoFJoint[] oneDoFJoints)
   {
      return cloneJointPathAndFilter(oneDoFJoints, OneDoFJoint.class);
   }

   public static <T extends JointBasics> T[] cloneJointPathAndFilter(T[] joints, Class<T> clazz)
   {
      return filterJoints(cloneJointPath(joints), clazz);
   }

   public static <T extends JointBasics> T[] cloneJointPathAndFilter(T[] joints, Class<T> clazz, String suffix)
   {
      return filterJoints(cloneJointPath(joints, suffix), clazz);
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
      return filterJoints(cloneJointPathDisconnectedFromOriginalRobot(joints, suffix, rootBodyFrame), clazz);
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
      RigidBodyTransform jointTransform = original.getOffsetTransform3D();
      Vector3D jointAxisCopy = new Vector3D(original.getJointAxis());
      OneDoFJoint clone;

      if (original instanceof RevoluteJoint)
         clone = ScrewTools.addRevoluteJoint(jointNameOriginal + cloneSuffix, clonePredecessor, jointTransform, jointAxisCopy);
      else if (original instanceof PrismaticJoint)
         clone = ScrewTools.addPrismaticJoint(jointNameOriginal + cloneSuffix, clonePredecessor, jointTransform, jointAxisCopy);
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
      RigidBodyBasics clone = ScrewTools.addRigidBody(nameOriginal + cloneSuffix, parentJointOfClone, massMomentOfInertiaPartCopy, mass, comOffsetCopy);
      return clone;
   }

   /**
    * Traverses up the kinematic chain from the candidate descendant towards the root body, checking to
    * see if each parent body is the ancestor in question.
    * 
    * @param candidateDescendant
    * @param ancestor
    * @return
    */
   public static boolean isAncestor(RigidBodyBasics candidateDescendant, RigidBodyBasics ancestor)
   {
      RigidBodyBasics currentBody = candidateDescendant;
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

   public static int computeDistanceToAncestor(RigidBodyBasics descendant, RigidBodyBasics ancestor)
   {
      int ret = 0;
      RigidBodyBasics currentBody = descendant;
      while (!currentBody.isRootBody() && (currentBody != ancestor))
      {
         ret++;
         currentBody = currentBody.getParentJoint().getPredecessor();
      }

      if (currentBody != ancestor)
         ret = -1;

      return ret;
   }

   public static void getJointVelocitiesMatrix(JointBasics[] joints, DenseMatrix64F jointVelocitiesMatrixToPack)
   {
      int rowStart = 0;
      for (JointBasics joint : joints)
      {
         int dof = joint.getDegreesOfFreedom();
         joint.getVelocityMatrix(jointVelocitiesMatrixToPack, rowStart);
         rowStart += dof;
      }
   }

   public static void getJointVelocitiesMatrix(Iterable<? extends JointBasics> joints, DenseMatrix64F jointVelocitiesMatrixToPack)
   {
      int rowStart = 0;
      for (JointBasics joint : joints)
      {
         int dof = joint.getDegreesOfFreedom();
         joint.getVelocityMatrix(jointVelocitiesMatrixToPack, rowStart);
         rowStart += dof;
      }
   }

   public static void getDesiredJointAccelerationsMatrix(Iterable<? extends JointBasics> joints, DenseMatrix64F desiredJointAccelerationsMatrixToPack)
   {
      int rowStart = 0;
      for (JointBasics joint : joints)
      {
         int dof = joint.getDegreesOfFreedom();
         joint.getDesiredAccelerationMatrix(desiredJointAccelerationsMatrixToPack, rowStart);
         rowStart += dof;
      }
   }

   public static void getDesiredJointAccelerationsMatrix(JointBasics[] joints, DenseMatrix64F desiredJointAccelerationsMatrixToPack)
   {
      int rowStart = 0;
      for (JointBasics joint : joints)
      {
         int dof = joint.getDegreesOfFreedom();
         joint.getDesiredAccelerationMatrix(desiredJointAccelerationsMatrixToPack, rowStart);
         rowStart += dof;
      }
   }

   /**
    * Calculates the number of degrees of freedom of the kinematic chain that starts from
    * {@code ancestor} to end to {@code descendant}.
    * 
    * @param ancestor the base of the kinematic chain.
    * @param descendant the end-effector of the kinematic chain.
    * @return the number of degrees of freedom.
    * @throws RuntimeException if the given ancestor and descendant are swapped, or if the do not
    *            belong to the same system.
    * @throws RuntimeException this method does not support in kinematic trees to go through different
    *            branches.
    */
   public static int computeDegreesOfFreedom(RigidBodyBasics ancestor, RigidBodyBasics descendant)
   {
      int nDoFs = 0;

      RigidBodyBasics currentBody = descendant;

      while (currentBody != ancestor)
      {
         JointBasics parentJoint = currentBody.getParentJoint();

         if (parentJoint == null)
            throw new RuntimeException("Could not find the ancestor: " + ancestor.getName() + ", to the descendant: " + descendant.getName());

         nDoFs += parentJoint.getDegreesOfFreedom();
         currentBody = parentJoint.getPredecessor();
      }

      return nDoFs;
   }

   public static int computeDegreesOfFreedom(JointBasics[] jointList)
   {
      int ret = 0;
      for (JointBasics joint : jointList)
      {
         ret += joint.getDegreesOfFreedom();
      }

      return ret;
   }

   public static int computeDegreesOfFreedom(Iterable<? extends JointBasics> jointList)
   {
      int ret = 0;
      for (JointBasics joint : jointList)
      {
         ret += joint.getDegreesOfFreedom();
      }

      return ret;
   }

   public static int computeDegreesOfFreedom(List<? extends JointBasics> jointList)
   {
      int ret = 0;
      for (int i = 0; i < jointList.size(); i++)
      {
         ret += jointList.get(i).getDegreesOfFreedom();
      }
      return ret;
   }

   public static SpatialAcceleration createGravitationalSpatialAcceleration(RigidBodyBasics rootBody, double gravity)
   {
      Vector3D gravitationalAcceleration = new Vector3D(0.0, 0.0, gravity);
      Vector3D zero = new Vector3D();
      SpatialAcceleration rootAcceleration = new SpatialAcceleration(rootBody.getBodyFixedFrame(), ReferenceFrame.getWorldFrame(), rootBody.getBodyFixedFrame(),
                                                                     zero, gravitationalAcceleration);

      return rootAcceleration;
   }

   public static void getJointPositions(JointBasics[] joints, DenseMatrix64F jointPositionsToPack)
   {
      int rowStart = 0;
      for (JointBasics joint : joints)
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

   public static void setJointPositions(JointBasics[] joints, DenseMatrix64F jointPositions)
   {
      int rowStart = 0;
      for (JointBasics joint : joints)
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

   public static void setDesiredAccelerations(JointBasics[] jointList, DenseMatrix64F jointAccelerations)
   {
      int rowStart = 0;
      for (JointBasics joint : jointList)
      {
         joint.setDesiredAcceleration(jointAccelerations, rowStart);
         rowStart += joint.getDegreesOfFreedom();
      }
   }

   public static void setDesiredAccelerations(Iterable<? extends JointBasics> jointList, DenseMatrix64F jointAccelerations)
   {
      int rowStart = 0;
      for (JointBasics joint : jointList)
      {
         joint.setDesiredAcceleration(jointAccelerations, rowStart);
         rowStart += joint.getDegreesOfFreedom();
      }
   }

   public static void setJointTorques(JointBasics[] jointList, DenseMatrix64F jointTorques)
   {
      int rowStart = 0;
      for (JointBasics joint : jointList)
      {
         joint.setJointTorque(jointTorques, rowStart);
         rowStart += joint.getDegreesOfFreedom();
      }
   }

   public static void setVelocities(JointBasics[] jointList, DenseMatrix64F jointVelocities)
   {
      int rowStart = 0;
      for (JointBasics joint : jointList)
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

   public static void computeIndexForJoint(List<? extends JointBasics> jointsInOrder, TIntArrayList listToPackIndices, JointBasics jointToComputeIndicesFor)
   {
      int startIndex = 0;
      for (int i = 0; i < jointsInOrder.size(); i++)
      {
         JointBasics joint = jointsInOrder.get(i);
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

   public static void computeIndexForJoint(JointBasics[] jointsInOrder, TIntArrayList listToPackIndices, JointBasics jointToComputeIndicesFor)
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

   public static RevoluteJoint[] extractRevoluteJoints(JointBasics[] allJoints)
   {
      if (allJoints == null)
         return null;

      ArrayList<RevoluteJoint> revoluteJointsList = new ArrayList<RevoluteJoint>();
      for (JointBasics joint : allJoints)
      {
         if (joint instanceof RevoluteJoint)
            revoluteJointsList.add((RevoluteJoint) joint);
      }

      RevoluteJoint[] revoluteJointArray = new RevoluteJoint[revoluteJointsList.size()];
      revoluteJointsList.toArray(revoluteJointArray);

      return revoluteJointArray;
   }

   public static <T extends JointBasics> int computeNumberOfJointsOfType(Class<T> clazz, JointBasics[] joints)
   {
      int ret = 0;
      for (JointBasics joint : joints)
      {
         if (clazz.isAssignableFrom(joint.getClass()))
            ret++;
      }

      return ret;
   }

   public static <T extends JointBasics> T[] filterJoints(JointBasics[] source, Class<T> clazz)
   {
      @SuppressWarnings("unchecked")
      T[] retArray = (T[]) Array.newInstance(clazz, ScrewTools.computeNumberOfJointsOfType(clazz, source));
      filterJoints(source, retArray, clazz);
      return retArray;
   }

   @SuppressWarnings("unchecked")
   public static <T extends JointBasics> void filterJoints(JointBasics[] source, T[] dest, Class<T> clazz)
   {
      int index = 0;
      for (JointBasics joint : source)
      {
         if (clazz.isAssignableFrom(joint.getClass()))
         {
            dest[index++] = (T) joint;
         }
      }
   }

   public static <T extends JointBasics> List<T> filterJoints(List<JointBasics> source, Class<T> clazz)
   {
      List<T> retList = new ArrayList<>();
      filterJoints(source, retList, clazz);
      return retList;
   }

   @SuppressWarnings("unchecked")
   public static <T extends JointBasics> void filterJoints(List<JointBasics> source, List<T> dest, Class<T> clazz)
   {
      for (JointBasics joint : source)
      {
         if (clazz.isAssignableFrom(joint.getClass()))
         {
            dest.add((T) joint);
         }
      }
   }

   public static JointBasics[] findJointsWithNames(JointBasics[] allJoints, String... jointNames)
   {
      if (jointNames == null)
         return null;

      JointBasics[] ret = new JointBasics[jointNames.length];
      int index = 0;
      for (JointBasics joint : allJoints)
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

   public static RigidBodyBasics[] findRigidBodiesWithNames(RigidBodyBasics[] allBodies, String... names)
   {
      RigidBodyBasics[] ret = new RigidBodyBasics[names.length];
      int index = 0;
      for (RigidBodyBasics body : allBodies)
      {
         for (String name : names)
         {
            if (body.getName().equals(name))
            {
               ret[index++] = body;
               if (index == names.length)
                  return ret;
            }
         }
      }

      if (index != names.length)
         throw new RuntimeException("Not all bodies could be found");

      return ret;
   }

   public static void addExternalWrenches(Map<RigidBodyBasics, Wrench> externalWrenches, Map<RigidBodyBasics, Wrench> wrenchMapToAdd)
   {
      for (RigidBodyBasics rigidBody : wrenchMapToAdd.keySet())
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
    * @return the {@link RigidBodyBasics} that is {@code numberOfBodies} higher up the rigid body chain then
    *         the {@code startBody}.
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