package us.ihmc.robotics.screwTheory;

import java.util.Arrays;
import java.util.List;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.Bound;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.RevoluteJointBasics;
import us.ihmc.robotics.kinematics.fourbar.FourBar;
import us.ihmc.robotics.kinematics.fourbar.FourBarAngle;
import us.ihmc.robotics.kinematics.fourbar.FourBarVertex;
import us.ihmc.robotics.screwTheory.FourBarKinematicLoopFunctionTools.FourBarToJointConverter;

/**
 * This class can be used to help enforcing the physical constraint of a four bar linkage given its
 * four joints.
 * <p>
 * Upon construction, the given joints are named to follow one of these layouts:
 * 
 * <pre>
 *      root            root
 *        |               |
 *        |               |
 *   A O-----O B     A O-----O B
 *     |     |          \   /
 *     |     |           \ /
 *     |     |    or      X
 *     |     |           / \
 *     |     |          /   \
 *   D O-----O C     C O-----O D
 *        |               |
 *   end-effector    end-effector
 * </pre>
 * </p>
 */
public class FourBarKinematicLoopFunction implements KinematicLoopFunction
{
   private static final double EPSILON = 1.0e-7;

   private final String name;
   private final FourBarVertex masterVertex;
   private final FourBar fourBar = new FourBar();
   private final RevoluteJointBasics jointA;
   private final RevoluteJointBasics jointB;
   private final RevoluteJointBasics jointC;
   private final RevoluteJointBasics jointD;
   private final List<RevoluteJointBasics> joints;

   private final FourBarToJointConverter converterA = new FourBarToJointConverter();
   private final FourBarToJointConverter converterB = new FourBarToJointConverter();
   private final FourBarToJointConverter converterC = new FourBarToJointConverter();
   private final FourBarToJointConverter converterD = new FourBarToJointConverter();
   private final FourBarToJointConverter[] converters = {converterA, converterB, converterC, converterD};

   private final DMatrixRMaj loopJacobianMatrix = new DMatrixRMaj(4, 1);
   private final DMatrixRMaj loopConvectiveTermMatrix = new DMatrixRMaj(4, 1);

   private final int masterJointIndex;
   private final int[] actuatedJointIndices;

   /**
    * Creates a new function to manage the physical constraint of a four bar linkage given its 4
    * joints.
    * 
    * @param name             the name of this four bar.
    * @param joints           the four joints composing the four bar linkage.
    * @param masterJointIndex the index among {@code joints} of the master joint, i.e. the only joint
    *                         which state defines the entire state of the four bar and which is
    *                         expected to be the only actuated joint.
    * @throws IllegalArgumentException if a four bar linkage could not be recognized from the given
    *                                  joints.
    * @see FourBarKinematicLoopFunctionTools#configureFourBarKinematics(RevoluteJointBasics[],
    *      FourBarToJointConverter[], FourBar, int, double)
    */
   public FourBarKinematicLoopFunction(String name, List<? extends RevoluteJointBasics> joints, int masterJointIndex)
   {
      this(name, joints.toArray(new RevoluteJointBasics[0]), masterJointIndex);
   }

   /**
    * Creates a new function to manage the physical constraint of a four bar linkage given its 4
    * joints.
    * 
    * @param name             the name of this four bar.
    * @param joints           the four joints composing the four bar linkage.
    * @param masterJointIndex the index among {@code joints} of the master joint, i.e. the only joint
    *                         which state defines the entire state of the four bar and which is
    *                         expected to be the only actuated joint.
    * @throws IllegalArgumentException if a four bar linkage could not be recognized from the given
    *                                  joints.
    * @see FourBarKinematicLoopFunctionTools#configureFourBarKinematics(RevoluteJointBasics[],
    *      FourBarToJointConverter[], FourBar, int, double)
    */
   public FourBarKinematicLoopFunction(String name, RevoluteJointBasics[] joints, int masterJointIndex)
   {
      this.name = name;

      // Copy the array so it cannot modified externally and the argument doesn't get modified. 
      joints = Arrays.copyOf(joints, joints.length);
      this.masterJointIndex = FourBarKinematicLoopFunctionTools.configureFourBarKinematics(joints, converters, fourBar, masterJointIndex, EPSILON);
      actuatedJointIndices = new int[] {this.masterJointIndex};
      this.joints = Arrays.asList(joints);
      jointA = joints[0];
      jointB = joints[1];
      jointC = joints[2];
      jointD = joints[3];

      masterVertex = fourBar.getVertex(FourBarAngle.values[this.masterJointIndex]);
   }

   /**
    * Tests if this four bar represents an inverted four bar as follows:
    *
    * <pre>
    *    root
    *      |
    *      |
    * A O-----O B
    *    \   /
    *     \ /
    *      X
    *     / \
    *    /   \
    * C O-----O D
    *      |
    * end-effector
    * </pre>
    * 
    * @return {@code true} if this four bar is inverted, {@code false} otherwise.
    */
   public boolean isInverted()
   {
      return fourBar.isInverted();
   }

   /**
    * Assuming the state of the master joint has been set, this method computes and updates the state
    * of the other joints such that the kinematic loop represents a proper four bar linkage with
    * constant side lengths.
    * <p>
    * This method also updates the loop Jacobian and convective term.
    * </p>
    * 
    * @param updateVelocity     whether the joint velocities should be computed and updated.
    * @param updateAcceleration whether the joint accelerations should be computed and updated,
    *                           requires {@code updateVelocity = true}.
    */
   public Bound updateState(boolean updateVelocity, boolean updateAcceleration)
   {
      if (updateAcceleration && !updateVelocity)
         throw new IllegalArgumentException("updateVelocity needs to be true for updateAcceleration to be true.");

      clampMasterJointPosition();

      RevoluteJointBasics masterJoint = getMasterJoint();
      FourBarToJointConverter masterConverter = converters[masterJointIndex];
      double angle = masterConverter.toFourBarInteriorAngle(masterJoint.getQ());
      Bound limit;

      if (updateVelocity)
      {
         double angleDot = masterConverter.toFourBarInteriorAngularDerivative(masterJoint.getQd());

         if (updateAcceleration)
         {
            double angleDDot = masterConverter.toFourBarInteriorAngularDerivative(masterJoint.getQdd());
            limit = fourBar.update(FourBarAngle.values[masterJointIndex], angle, angleDot, angleDDot);
         }
         else
         {
            limit = fourBar.update(FourBarAngle.values[masterJointIndex], angle, angleDot);
         }
      }
      else
      {
         limit = fourBar.update(FourBarAngle.values[masterJointIndex], angle);
      }

      for (int i = 0; i < 4; i++)
      {
         if (i == masterJointIndex)
            continue;

         RevoluteJointBasics joint = joints.get(i);
         FourBarToJointConverter converter = converters[i];
         FourBarVertex fourBarVertex = fourBar.getVertex(FourBarAngle.values[i]);

         joint.setQ(converter.toJointAngle(fourBarVertex.getAngle()));
         if (updateVelocity)
            joint.setQd(converter.toJointDerivative(fourBarVertex.getAngleDot()));
         if (updateAcceleration)
            joint.setQdd(converter.toJointDerivative(fourBarVertex.getAngleDDot()));
      }

      updateLoopJacobian();
      updateLoopConvectiveTerm();
      return limit;
   }

   /**
    * Assuming the effort for each joint has been previously updated, this method centralizes the
    * efforts on the master joint while preserving the resulting dynamics of the linkage.
    */
   public void updateEffort()
   {
      double tau = 0.0;

      for (int i = 0; i < 4; i++)
      {
         RevoluteJointBasics joint = joints.get(i);
         tau += loopJacobianMatrix.get(i, 0) * joint.getTau();
         joint.setTau(0.0);
      }

      getMasterJoint().setTau(tau / loopJacobianMatrix.get(masterJointIndex, 0));
   }

   /** {@inheritDoc} */
   @Override
   public void adjustConfiguration(DMatrixRMaj jointConfigurations)
   {
      if (jointConfigurations.getNumRows() != 4 || jointConfigurations.getNumCols() != 1)
         throw new IllegalArgumentException("Unexpected matrix size. [row=" + jointConfigurations.getNumRows() + ", col=" + jointConfigurations.getNumCols()
               + "].");

      double angle = MathTools.clamp(jointConfigurations.get(masterJointIndex), getMasterJoint().getJointLimitLower(), getMasterJoint().getJointLimitUpper());
      fourBar.update(FourBarAngle.values[masterJointIndex], converters[masterJointIndex].toFourBarInteriorAngle(angle));

      for (int i = 0; i < 4; i++)
      {
         double q = converters[i].toJointAngle(fourBar.getVertex(FourBarAngle.values[i]).getAngle());
         jointConfigurations.set(i, q);
      }
   }

   /** {@inheritDoc} */
   @Override
   public void adjustTau(DMatrixRMaj tauJoints)
   {
      if (tauJoints.getNumRows() != 4 || tauJoints.getNumCols() != 1)
         throw new IllegalArgumentException("Unexpected matrix size. [row=" + tauJoints.getNumRows() + ", col=" + tauJoints.getNumCols() + "].");

      double tau = 0.0;

      for (int i = 0; i < 4; i++)
      {
         tau += loopJacobianMatrix.get(i, 0) * tauJoints.get(i);
         tauJoints.set(i, 0.0);
      }

      tauJoints.set(masterJointIndex, tau / loopJacobianMatrix.get(masterJointIndex, 0));
   }

   private void updateLoopJacobian()
   {
      // Definitely not the most effective to compute the Jacobian but should not matter compared to the rest of the controller's computational load.
      double angle = converters[masterJointIndex].toFourBarInteriorAngle(getMasterJoint().getQ());
      double angleDot = converters[masterJointIndex].toFourBarInteriorAngularDerivative(1.0);
      fourBar.update(FourBarAngle.values[masterJointIndex], angle, angleDot);

      for (int i = 0; i < 4; i++)
      {
         if (i == masterJointIndex)
         {
            loopJacobianMatrix.set(i, 0, 1.0);
         }
         else
         {
            FourBarVertex fourBarVertex = fourBar.getVertex(FourBarAngle.values[i]);
            loopJacobianMatrix.set(i, 0, converters[i].toJointDerivative(fourBarVertex.getAngleDot()));
         }
      }
   }

   private void updateLoopConvectiveTerm()
   {
      // Definitely not the most effective to compute the convective term but should not matter compared to the rest of the controller's computational load.
      double angle = converters[masterJointIndex].toFourBarInteriorAngle(getMasterJoint().getQ());
      double angleDot = converters[masterJointIndex].toFourBarInteriorAngularDerivative(getMasterJoint().getQd());
      double angleDDot = 0.0;
      fourBar.update(FourBarAngle.values[masterJointIndex], angle, angleDot, angleDDot);

      for (int i = 0; i < 4; i++)
      {
         if (i == masterJointIndex)
         {
            loopConvectiveTermMatrix.set(i, 0, 0.0);
         }
         else
         {
            FourBarVertex fourBarVertex = fourBar.getVertex(FourBarAngle.values[i]);
            loopConvectiveTermMatrix.set(i, 0, converters[i].toJointDerivative(fourBarVertex.getAngleDDot()));
         }
      }
   }

   /** {@inheritDoc} */
   @Override
   public List<RevoluteJointBasics> getLoopJoints()
   {
      return joints;
   }

   private void clampMasterJointPosition()
   {
      RevoluteJointBasics masterJoint = getMasterJoint();

      if (masterJoint.getQ() < masterJoint.getJointLimitLower())
      {
         LogTools.warn("{} is beyond its lower limit: {}, clamping it to {}.", masterJoint.getName(), masterJoint.getQ(), masterJoint.getJointLimitLower());
         masterJoint.setQ(masterJoint.getJointLimitLower());
      }
      else if (masterJoint.getQ() > masterJoint.getJointLimitUpper())
      {
         LogTools.warn("{} is beyond its upper limit: {}, clamping it to {}.", masterJoint.getName(), masterJoint.getQ(), masterJoint.getJointLimitUpper());
         masterJoint.setQ(masterJoint.getJointLimitUpper());
      }
   }

   public String getName()
   {
      return name;
   }

   /**
    * Returns one of the two joints starting the linkage:
    * 
    * <pre>
    *      root            root
    *        |               |
    *        |               |
    *   A O-----O B     A O-----O B
    *     |     |          \   /
    *     |     |           \ /
    *     |     |    or      X
    *     |     |           / \
    *     |     |          /   \
    *   D O-----O C     C O-----O D
    *        |               |
    *   end-effector    end-effector
    * </pre>
    * 
    * @return the reference to the joint A.
    */
   public RevoluteJointBasics getJointA()
   {
      return jointA;
   }

   /**
    * Returns one of the two joints starting the linkage:
    * 
    * <pre>
    *      root            root
    *        |               |
    *        |               |
    *   A O-----O B     A O-----O B
    *     |     |          \   /
    *     |     |           \ /
    *     |     |    or      X
    *     |     |           / \
    *     |     |          /   \
    *   D O-----O C     C O-----O D
    *        |               |
    *   end-effector    end-effector
    * </pre>
    * 
    * @return the reference to the joint B.
    */
   public RevoluteJointBasics getJointB()
   {
      return jointB;
   }

   /**
    * Returns one of the two joints ending the linkage:
    * 
    * <pre>
    *      root            root
    *        |               |
    *        |               |
    *   A O-----O B     A O-----O B
    *     |     |          \   /
    *     |     |           \ /
    *     |     |    or      X
    *     |     |           / \
    *     |     |          /   \
    *   D O-----O C     C O-----O D
    *        |               |
    *   end-effector    end-effector
    * </pre>
    * 
    * @return the reference to the joint C.
    */
   public RevoluteJointBasics getJointC()
   {
      return jointC;
   }

   /**
    * Returns one of the two joints ending the linkage:
    * 
    * <pre>
    *      root            root
    *        |               |
    *        |               |
    *   A O-----O B     A O-----O B
    *     |     |          \   /
    *     |     |           \ /
    *     |     |    or      X
    *     |     |           / \
    *     |     |          /   \
    *   D O-----O C     C O-----O D
    *        |               |
    *   end-effector    end-effector
    * </pre>
    * 
    * @return the reference to the joint D.
    */
   public RevoluteJointBasics getJointD()
   {
      return jointD;
   }

   public int getMasterJointIndex()
   {
      return masterJointIndex;
   }

   public FourBarVertex getMasterVertex()
   {
      return masterVertex;
   }

   /** {@inheritDoc} */
   @Override
   public int[] getActuatedJointIndices()
   {
      return actuatedJointIndices;
   }

   /**
    * Returns the reference to the master joint of this four bar, i.e. the only joint which state
    * defines the entire state of the four bar and which is expected to be the only actuated joint.
    * 
    * @return the reference to the master joint.
    */
   public RevoluteJointBasics getMasterJoint()
   {
      return joints.get(masterJointIndex);
   }

   /** {@inheritDoc} */
   @Override
   public DMatrixRMaj getLoopJacobian()
   {
      return loopJacobianMatrix;
   }

   /** {@inheritDoc} */
   @Override
   public DMatrixRMaj getLoopConvectiveTerm()
   {
      return loopConvectiveTermMatrix;
   }

   /**
    * Returns the reference to the internal calculator used for this kinematic loop.
    * <p>
    * Mainly used for testing.
    * </p>
    * 
    * @return the reference to the four bar geometry calculator.
    */
   public FourBar getFourBar()
   {
      return fourBar;
   }

   /**
    * Returns the converters used to switch between four bar interior angles and joint angles.
    * 
    * @return the converters used to switch between four bar interior angles and joint angles.
    */
   public FourBarToJointConverter[] getConverters()
   {
      return converters;
   }
}
