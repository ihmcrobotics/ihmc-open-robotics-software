package us.ihmc.robotics.screwTheory;

import java.util.Arrays;
import java.util.List;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.RevoluteJointBasics;
import us.ihmc.robotics.kinematics.fourbar.FourBar;
import us.ihmc.robotics.kinematics.fourbar.FourBarAngle;
import us.ihmc.robotics.kinematics.fourbar.FourBarVertex;
import us.ihmc.robotics.screwTheory.FourBarKinematicLoopTools.FourBarToJointConverter;

/**
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
 */
public class FourBarKinematicLoop implements KinematicLoopFunction
{
   private static final double EPSILON = 1.0e-7;

   private final String name;
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

   public FourBarKinematicLoop(String name, List<? extends RevoluteJointBasics> joints, int masterJointIndex)
   {
      this(name, joints.toArray(new RevoluteJointBasics[0]), masterJointIndex);
   }

   public FourBarKinematicLoop(String name, RevoluteJointBasics[] joints, int masterJointIndex)
   {
      this.name = name;

      this.masterJointIndex = FourBarKinematicLoopTools.configureFourBarKinematics(joints, converters, fourBar, masterJointIndex, EPSILON);
      this.joints = Arrays.asList(joints);
      jointA = joints[0];
      jointB = joints[1];
      jointC = joints[2];
      jointD = joints[3];
   }

   public void updateState(boolean updateVelocity, boolean updateAcceleration)
   {
      if (updateAcceleration && !updateVelocity)
         throw new IllegalArgumentException("updateVelocity needs to be true for updateAcceleration to be true.");

      clampMasterJointPosition();

      RevoluteJointBasics masterJoint = getMasterJoint();
      FourBarToJointConverter masterConverter = converters[masterJointIndex];
      double angle = masterConverter.toFourBarInteriorAngle(masterJoint.getQ());

      if (updateVelocity)
      {
         double angleDot = masterConverter.toFourBarInteriorAngularDerivative(masterJoint.getQd());

         if (updateAcceleration)
         {
            double angleDDot = masterConverter.toFourBarInteriorAngularDerivative(masterJoint.getQdd());
            fourBar.update(FourBarAngle.values[masterJointIndex], angle, angleDot, angleDDot);
         }
         else
         {
            fourBar.update(FourBarAngle.values[masterJointIndex], angle, angleDot);
         }
      }
      else
      {
         fourBar.update(FourBarAngle.values[masterJointIndex], angle);
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
   }

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

   @Override
   public void computeTau(DMatrixRMaj tauJoints)
   {
      if (tauJoints.getNumRows() != 4)
         throw new IllegalArgumentException("Unexpected matrix size. Expected 4 rows, was: " + tauJoints.getNumRows());

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
         FourBarVertex fourBarVertex = fourBar.getVertex(FourBarAngle.values[i]);
         loopJacobianMatrix.set(i, 0, converters[i].toJointDerivative(fourBarVertex.getAngleDot()));
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
         FourBarVertex fourBarVertex = fourBar.getVertex(FourBarAngle.values[i]);
         loopConvectiveTermMatrix.set(i, 0, converters[i].toJointDerivative(fourBarVertex.getAngleDDot()));
      }
   }

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

   public RevoluteJointBasics getJointA()
   {
      return jointA;
   }

   public RevoluteJointBasics getJointB()
   {
      return jointB;
   }

   public RevoluteJointBasics getJointC()
   {
      return jointC;
   }

   public RevoluteJointBasics getJointD()
   {
      return jointD;
   }

   public RevoluteJointBasics getMasterJoint()
   {
      return joints.get(masterJointIndex);
   }

   @Override
   public DMatrixRMaj getLoopJacobian()
   {
      return loopJacobianMatrix;
   }

   @Override
   public DMatrixRMaj getLoopConvectiveTerm()
   {
      return loopConvectiveTermMatrix;
   }

   public FourBar getFourBar()
   {
      return fourBar;
   }
}
