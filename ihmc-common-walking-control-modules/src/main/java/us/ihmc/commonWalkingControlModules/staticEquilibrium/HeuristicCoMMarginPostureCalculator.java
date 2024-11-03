package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import gnu.trove.list.array.TIntArrayList;
import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commonWalkingControlModules.controlModules.multiContact.PointJacobianMotionNumericalDerivativeCalculator;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.filters.RateLimitedYoVariable;
import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class HeuristicCoMMarginPostureCalculator
{
   private static final boolean CHECK_EPSILON_SATURATION = true;
   private static final int LINEAR_DIMENSIONS = 3;
   private static final int SPATIAL_DIMENSIONS = 6;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final CenterOfMassStabilityMarginRegionCalculator staticStabilityRegionCalculator;
   private final WholeBodyContactState contactState;

   private final TIntArrayList lowestMarginVertexIndices = new TIntArrayList();
   private final YoDouble marginEpsilon = new YoDouble("marginEpsilon", registry);

   // Debug variables
   private final YoInteger tickCount = new YoInteger("tickCount", registry);

   private final YoInteger numberOfSaturatedTorqueConstraints = new YoInteger("numberOfSaturatedTorqueConstraints", registry);
   private final YoDouble weightFactorTotal = new YoDouble("weightFactorTotal", registry);
   private final DMatrixRMaj graspMatrixMotionDerivative = new DMatrixRMaj(0);
   private final DMatrixRMaj graspMatrixTransposeMotionDerivative = new DMatrixRMaj(0);
   private final PointJacobianMotionNumericalDerivativeCalculator motionDerivativeCalculator = new PointJacobianMotionNumericalDerivativeCalculator();

   private final DMatrixRMaj qdTemp = new DMatrixRMaj(0);
   /* A joint velocity is computed for each saturated torque constraint, which are averaged and stored in this object */
   private final YoDouble[] qdOptimizedPosture;
   private final RateLimitedYoVariable[] qdOptimizedPostureRateLimited;
   private final AlphaFilteredYoVariable[] qdOptimizedPostureAlphaFiltered;

   // Create an alpha-to-alpha function for mapping joint feasibility proximity to joint limit
   private final YoDouble alphaTorqueMarginToOptimize = new YoDouble("alphaTorqueMarginToOptimize", registry);
   private final DMatrixRMaj actuationMargin = new DMatrixRMaj(0);
   private final DMatrixRMaj actuationMarginViolation = new DMatrixRMaj(0);

   public HeuristicCoMMarginPostureCalculator(CenterOfMassStabilityMarginRegionCalculator staticStabilityRegionCalculator,
                                              WholeBodyContactState contactState,
                                              double controlDT,
                                              YoRegistry parentRegistry)
   {
      this.staticStabilityRegionCalculator = staticStabilityRegionCalculator;
      this.contactState = contactState;

      marginEpsilon.set(0.01);
      qdTemp.reshape(contactState.getNumberOfJoints(), 1);

      qdOptimizedPosture = new YoDouble[contactState.getNumberOfJoints()];
      qdOptimizedPostureRateLimited = new RateLimitedYoVariable[contactState.getNumberOfJoints()];
      qdOptimizedPostureAlphaFiltered = new AlphaFilteredYoVariable[contactState.getNumberOfJoints()];

      for (int i = 0; i < qdOptimizedPosture.length; i++)
      {
         OneDoFJointBasics joint = contactState.getOneDoFJoints()[i];
         qdOptimizedPosture[i] = new YoDouble("qd_comMargin_" + joint.getName(), registry);
         qdOptimizedPostureRateLimited[i] = new RateLimitedYoVariable("qd_comMargin_rl_" + joint.getName(), registry, 20.0, qdOptimizedPosture[i], controlDT);
      }

      alphaTorqueMarginToOptimize.set(0.03);

      parentRegistry.addChild(registry);
   }

   public void computePostureAdjustment()
   {
      staticStabilityRegionCalculator.collectLowestMarginVertexIndices(lowestMarginVertexIndices, marginEpsilon.getValue());

      graspMatrixMotionDerivative.reshape(LINEAR_DIMENSIONS * contactState.getNumberOfContactPoints(), contactState.getNumberOfJoints());
      graspMatrixTransposeMotionDerivative.reshape(contactState.getNumberOfJoints(), LINEAR_DIMENSIONS * contactState.getNumberOfContactPoints());

      ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
      //////////////////////////////////// COMPUTE TORQUE REDUCTION OBJECTIVES //////////////////////////////////////
      ///////////////////////////////////////////////////////////////////////////////////////////////////////////////

      for (int i = 0; i < qdOptimizedPosture.length; i++)
      {
         qdOptimizedPosture[i].set(0.0);
      }

      numberOfSaturatedTorqueConstraints.set(0);
      weightFactorTotal.set(0.0);
      tickCount.increment();

      for (int vertexIndex = 0; vertexIndex < lowestMarginVertexIndices.size(); vertexIndex++)
      {
         computeTorqueReductionMotion(lowestMarginVertexIndices.get(vertexIndex));
      }

      for (int i = 0; i < qdOptimizedPosture.length; i++)
      {
         if (numberOfSaturatedTorqueConstraints.getValue() > 0)
         {
            qdOptimizedPosture[i].mul(1.0 / weightFactorTotal.getValue());
         }
         else
         {
            qdOptimizedPosture[i].set(0.0);
         }

         qdOptimizedPostureRateLimited[i].update();
      }
   }

   public boolean hasTorqueSaturatedConstraints()
   {
      return numberOfSaturatedTorqueConstraints.getValue() > 0;
   }

   public double getOptimizedPostureAdjustmentVelocity(int jointIndex)
   {
//      return qdOptimizedPostureRateLimited[jointIndex].getValue();
      return qdOptimizedPosture[jointIndex].getValue();
   }

   private void computeTorqueReductionMotion(int vertexIndex)
   {
      TIntArrayList activeSet = staticStabilityRegionCalculator.getSaturatedConstraintSet(vertexIndex);
      DMatrixRMaj resolvedForce = staticStabilityRegionCalculator.getResolvedForce(vertexIndex);
      DMatrixRMaj actuationConstraintMatrix = contactState.getActuationConstraintMatrix();
      DMatrixRMaj actuationConstraintVector = contactState.getActuationConstraintVector();

      if (CHECK_EPSILON_SATURATION)
      {
         CommonOps_DDRM.mult(actuationConstraintMatrix, resolvedForce, actuationMargin);
         CommonOps_DDRM.subtract(actuationConstraintVector, actuationMargin, actuationMarginViolation);

         for (int i = 0; i < actuationMarginViolation.getNumRows(); i++)
         {
            double actuationMarginViolationEntry = actuationMarginViolation.get(i, 0);
            OneDoFJointBasics joint = contactState.getJointFromActuationConstraintIndexZeroIndexed(i);
            double margin = alphaTorqueMarginToOptimize.getValue() * Math.abs(joint.getEffortLimitUpper());
            if (actuationMarginViolationEntry < margin)
            {
               boolean isUpperBound = contactState.isActuationConstraintUpperBoundZeroIndexed(i);
               computeTorqueReductionMotionForJoint(resolvedForce, joint, isUpperBound, 1.0 - Math.abs(actuationMarginViolationEntry / margin));
            }
         }
      }
      else
      {
         if (activeSet == null)
         {
            return;
         }

         for (int activeSetIdx = 0; activeSetIdx < activeSet.size(); activeSetIdx++)
         {
            int activeSetIndex = activeSet.get(activeSetIdx);
            if (!contactState.isJointTorqueActuationConstraint(activeSetIndex))
               continue;

            OneDoFJointBasics joint = contactState.getJointFromActuationConstraintIndex(activeSetIndex);
            boolean isUpperBound = contactState.isActuationConstraintUpperBound(activeSetIndex);
            computeTorqueReductionMotionForJoint(resolvedForce, joint, isUpperBound, 1.0);
         }
      }
   }

   private void computeTorqueReductionMotionForJoint(DMatrixRMaj resolvedForce, OneDoFJointBasics joint, boolean isUpperBound, double weightFactor)
   {
      for (int contactIdx = 0; contactIdx < contactState.getNumberOfContactPoints(); contactIdx++)
      {
         JointBasics[] contactKinematicChain = contactState.getKinematicChain(contactIdx);
         if (!isJointPresent(contactKinematicChain, joint))
            continue;

         GeometricJacobian jacobian = contactState.updateContactJacobian(contactIdx);
         int finalContactIdx = contactIdx;
         DMatrixRMaj jacobianMotionDerivative = motionDerivativeCalculator.compute(jacobian, joint,
                                                                                   () -> contactState.updateContactJacobian(finalContactIdx));

         JointBasics[] joints = jacobian.getJointsInOrder();
         for (int jointIdx = 1; jointIdx < joints.length; jointIdx++)
         {
            int jointSystemIndex = contactState.getJointIndex((OneDoFJointBasics) joints[jointIdx]);

            for (int axisIdx = 0; axisIdx < LINEAR_DIMENSIONS; axisIdx++)
            {
               double jacobianMotionDerivativeEntry = jacobianMotionDerivative.get(axisIdx, jointIdx - 1);
               graspMatrixMotionDerivative.set(LINEAR_DIMENSIONS * contactIdx + axisIdx, jointSystemIndex, jacobianMotionDerivativeEntry);
            }
         }
      }

      CommonOps_DDRM.transpose(graspMatrixMotionDerivative, graspMatrixTransposeMotionDerivative);
      CommonOps_DDRM.mult(graspMatrixTransposeMotionDerivative, resolvedForce, qdTemp);
      CommonOps_DDRM.scale(weightFactor * (isUpperBound ? 1.0 : -1.0), qdTemp);

      numberOfSaturatedTorqueConstraints.increment();
      weightFactorTotal.add(weightFactor);

      for (int i = 0; i < qdOptimizedPosture.length; i++)
      {
         qdOptimizedPosture[i].add(qdTemp.get(i, 0));
      }
   }

   public TIntArrayList getLowestMarginVertexIndices()
   {
      return lowestMarginVertexIndices;
   }

   private static boolean isJointPresent(JointBasics[] joints, JointBasics jointToCheckIfPresent)
   {
      for (int i = 0; i < joints.length; i++)
      {
         if (joints[i] == jointToCheckIfPresent)
            return true;
      }
      return false;
   }
}
