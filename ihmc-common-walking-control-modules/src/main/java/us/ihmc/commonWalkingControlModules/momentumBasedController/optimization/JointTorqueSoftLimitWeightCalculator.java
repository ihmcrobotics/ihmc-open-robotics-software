package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import org.ejml.data.DMatrix;

import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class JointTorqueSoftLimitWeightCalculator implements JointTorqueMinimizationWeightCalculator
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final JointBasics[] indexedJoints;
   private final OneDoFJointBasics[] indexedOneDoFJoints;
   private final YoDouble lowWeight = new YoDouble("lowJointTorqueWeight", registry);
   private final YoDouble highWeight = new YoDouble("highJointTorqueWeight", registry);
   private final YoDouble tauUnitThreshold = new YoDouble("unitTorqueThreshold", registry);

   private final YoDouble[] oneDoFJointTorqueWeights;

   public JointTorqueSoftLimitWeightCalculator(JointIndexHandler jointIndexHandler)
   {
      indexedJoints = jointIndexHandler.getIndexedJoints();
      indexedOneDoFJoints = jointIndexHandler.getIndexedOneDoFJoints();

      oneDoFJointTorqueWeights = new YoDouble[indexedOneDoFJoints.length];

      for (int i = 0; i < indexedOneDoFJoints.length; i++)
      {
         JointBasics joint = indexedOneDoFJoints[i];
         oneDoFJointTorqueWeights[i] = new YoDouble(joint.getName() + "TorqueWeight", registry);
      }
   }

   public void setParameters(double lowWeight, double highWeight, double tauPercentThreshold)
   {
      this.lowWeight.set(lowWeight);
      this.highWeight.set(highWeight);
      tauUnitThreshold.set(2.0 * tauPercentThreshold);
   }

   @Override
   public void computeWeightMatrix(DMatrix tauMatrix, DMatrix weightMatrixToPack)
   {
      int matrixIndex = 0;
      int oneDoFJointIndex = 0;

      for (int i = 0; i < indexedJoints.length; i++)
      {
         JointBasics joint = indexedJoints[i];

         if (joint instanceof OneDoFJointBasics)
         {
            OneDoFJointReadOnly oneDoFJoint = (OneDoFJointReadOnly) joint;

            double tau_min = oneDoFJoint.getEffortLimitLower();
            double tau_max = oneDoFJoint.getEffortLimitUpper();

            double w_lo = lowWeight.getValue();
            double w_hi = highWeight.getValue();
            double w;

            if (!Double.isFinite(tau_min) || !Double.isFinite(tau_max))
            {
               w = w_lo;
            }
            else
            {
               double tau = tauMatrix.get(matrixIndex, 0);

               if (tau <= tau_min || tau >= tau_max)
               {
                  w = w_hi;
               }
               else
               {
                  double tau_mid = 0.5 * (tau_max + tau_min);
                  double tau_range = tau_max - tau_min;
                  // Building tau_unit in [0, 1]
                  // - tau_unit = 0: in the middle of the range
                  // - tau_unit = 1: at lower/upper limit
                  double tau_unit = 2.0 * Math.abs(tau - tau_mid) / tau_range;

                  double tau_unit_thres = tauUnitThreshold.getValue();
                  if (tau_unit <= tau_unit_thres)
                  {
                     w = w_lo;
                  }
                  else
                  {
                     double alpha = (tau_unit - tau_unit_thres) / (1.0 - tau_unit_thres);
                     w = EuclidCoreTools.interpolate(w_lo, w_hi, alpha);
                  }
               }
            }

            oneDoFJointTorqueWeights[oneDoFJointIndex].set(w);
            weightMatrixToPack.set(matrixIndex, matrixIndex, w);
            oneDoFJointIndex++;
         }

         matrixIndex += joint.getDegreesOfFreedom();
      }
   }

   @Override
   public boolean isWeightZero()
   {
      return highWeight.getValue() <= 0.0;
   }

   @Override
   public YoRegistry getRegistry()
   {
      return registry;
   }
}
