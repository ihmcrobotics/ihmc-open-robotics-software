package us.ihmc.aware.controller.force.taskSpaceController;

import us.ihmc.aware.util.ContactState;
import us.ihmc.aware.util.DoubleWrapper;
import us.ihmc.aware.vmc.QuadrupedContactForceLimits;
import us.ihmc.aware.vmc.QuadrupedContactForceOptimizationSettings;
import us.ihmc.aware.vmc.QuadrupedVirtualModelControllerSettings;
import us.ihmc.robotics.controllers.YoAxisAngleOrientationGains;
import us.ihmc.robotics.controllers.YoEuclideanPositionGains;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedTaskSpaceControllerSettings
{
   // contact state
   private final QuadrantDependentList<ContactState> contactState;
   private final QuadrantDependentList<DoubleWrapper> contactPressureUpperLimit;
   private double jointDamping = 0.0;

   // command weights
   private final double[] comTorqueCommandWeights;
   private final double[] comForceCommandWeights;
   private final QuadrantDependentList<double[]> soleForceCommandWeights;

   // feedback gains
   private final double[] bodyOrientationProportionalGains;
   private final double[] bodyOrientationIntegralGains;
   private final double[] bodyOrientationDerivativeGains;
   private double bodyOrientationMaxIntegralError;
   private final double[] comPositionProportionalGains;
   private final double[] comPositionIntegralGains;
   private final double[] comPositionDerivativeGains;
   private double comPositionMaxIntegralError;
   private final QuadrantDependentList<double[]> solePositionProportionalGains;
   private final QuadrantDependentList<double[]> solePositionIntegralGains;
   private final QuadrantDependentList<double[]> solePositionDerivativeGains;
   private QuadrantDependentList<DoubleWrapper> solePositionMaxIntegralError;

   private final double[] zeroGains = new double[] {0.0, 0.0, 0.0};

   public QuadrupedTaskSpaceControllerSettings()
   {
      // contact settings
      contactState = new QuadrantDependentList<>();
      contactPressureUpperLimit = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         contactState.set(robotQuadrant, ContactState.NO_CONTACT);
         contactPressureUpperLimit.set(robotQuadrant, new DoubleWrapper());
      }

      // command weights
      comTorqueCommandWeights = new double[3];
      comForceCommandWeights = new double[3];
      soleForceCommandWeights = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         soleForceCommandWeights.set(robotQuadrant, new double[3]);
      }

      // feedback gains
      bodyOrientationProportionalGains = new double[3];
      bodyOrientationIntegralGains = new double[3];
      bodyOrientationDerivativeGains = new double[3];
      comPositionProportionalGains = new double[3];
      comPositionIntegralGains = new double[3];
      comPositionDerivativeGains = new double[3];
      solePositionProportionalGains = new QuadrantDependentList<>();
      solePositionIntegralGains = new QuadrantDependentList<>();
      solePositionDerivativeGains = new QuadrantDependentList<>();
      solePositionMaxIntegralError = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         solePositionProportionalGains.set(robotQuadrant, new double[3]);
         solePositionIntegralGains.set(robotQuadrant, new double[3]);
         solePositionDerivativeGains.set(robotQuadrant, new double[3]);
         solePositionMaxIntegralError.set(robotQuadrant, new DoubleWrapper());
      }

      initialize();
   }

   public void initialize()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         setContactState(robotQuadrant, ContactState.NO_CONTACT);
         setContactPressureUpperLimit(robotQuadrant, Double.MAX_VALUE);
      }
      setJointDamping(0.0);
      setComForceCommandWeights(1.0, 1.0, 1.0);
      setComTorqueCommandWeights(1.0, 1.0, 1.0);
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         setSoleForceCommandWeights(robotQuadrant, 0.0, 0.0, 0.0);
      }
      setBodyOrientationFeedbackGainsToZero();
      setComPositionFeedbackGainsToZero();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         setSolePositionFeedbackGainsToZero(robotQuadrant);
      }
   }

   public void setContactState(RobotQuadrant robotQuadrant, ContactState contactState)
   {
      this.contactState.set(robotQuadrant, contactState);
   }

   public void setContactPressureUpperLimit(RobotQuadrant robotQuadrant, double contactPressureUpperLimit)
   {
      this.contactPressureUpperLimit.get(robotQuadrant).setValue(contactPressureUpperLimit);
   }

   public void setJointDamping(double jointDamping)
   {
      this.jointDamping = jointDamping;
   }

   public void setComTorqueCommandWeights(double[] weights)
   {
      for (int i = 0; i < 3; i++)
      {
         comTorqueCommandWeights[i] = Math.max(weights[i], 0.0);
      }
   }

   public void setComForceCommandWeights(double[] weights)
   {
      for (int i = 0; i < 3; i++)
      {
         comForceCommandWeights[i] = weights[i];
      }
   }

   public void setSoleForceCommandWeights(RobotQuadrant robotQuadrant, double[] weights)
   {
      for (int i = 0; i < 3; i++)
      {
         soleForceCommandWeights.get(robotQuadrant)[i] = weights[i];
      }
   }

   public void setComTorqueCommandWeights(double weightX, double weightY, double weightZ)
   {
      comTorqueCommandWeights[0] = weightX;
      comTorqueCommandWeights[1] = weightY;
      comTorqueCommandWeights[2] = weightZ;
   }

   public void setComForceCommandWeights(double weightX, double weightY, double weightZ)
   {
      comForceCommandWeights[0] = weightX;
      comForceCommandWeights[1] = weightY;
      comForceCommandWeights[2] = weightZ;
   }

   public void setSoleForceCommandWeights(RobotQuadrant robotQuadrant, double weightX, double weightY, double weightZ)
   {
      soleForceCommandWeights.get(robotQuadrant)[0] = weightX;
      soleForceCommandWeights.get(robotQuadrant)[1] = weightY;
      soleForceCommandWeights.get(robotQuadrant)[2] = weightZ;
   }

   public void setBodyOrientationFeedbackGainsToZero()
   {
      setBodyOrientationFeedbackGains(zeroGains, zeroGains, zeroGains, Double.MAX_VALUE);
   }

   public void setBodyOrientationFeedbackGains(double[] proportionalGains)
   {
      setBodyOrientationFeedbackGains(proportionalGains, zeroGains, zeroGains, Double.MAX_VALUE);
   }

   public void setBodyOrientationFeedbackGains(double[] proportionalGains, double[] derivativeGains)
   {
      setBodyOrientationFeedbackGains(proportionalGains, derivativeGains, zeroGains, Double.MAX_VALUE);
   }

   public void setBodyOrientationFeedbackGains(double[] proportionalGains, double[] derivativeGains, double[] integralGains)
   {
      setBodyOrientationFeedbackGains(proportionalGains, derivativeGains, integralGains, Double.MAX_VALUE);
   }

   public void setBodyOrientationFeedbackGains(double[] proportionalGains, double[] derivativeGains, double[] integralGains, double maxIntegralError)
   {
      for (int i = 0; i < 3; i++)
      {
         bodyOrientationProportionalGains[i] = proportionalGains[i];
         bodyOrientationDerivativeGains[i] = derivativeGains[i];
         bodyOrientationIntegralGains[i] = integralGains[i];
      }
      bodyOrientationMaxIntegralError = maxIntegralError;
   }

   public void setComPositionFeedbackGainsToZero()
   {
      setComPositionFeedbackGains(zeroGains, zeroGains, zeroGains, Double.MAX_VALUE);
   }

   public void setComPositionFeedbackGains(double[] proportionalGains)
   {
      setComPositionFeedbackGains(proportionalGains, zeroGains, zeroGains, Double.MAX_VALUE);
   }

   public void setComPositionFeedbackGains(double[] proportionalGains, double[] derivativeGains)
   {
      setComPositionFeedbackGains(proportionalGains, derivativeGains, zeroGains, Double.MAX_VALUE);
   }

   public void setComPositionFeedbackGains(double[] proportionalGains, double[] derivativeGains, double[] integralGains)
   {
      setComPositionFeedbackGains(proportionalGains, derivativeGains, integralGains, Double.MAX_VALUE);
   }

   public void setComPositionFeedbackGains(double[] proportionalGains, double[] derivativeGains, double[] integralGains, double maxIntegralError)
   {
      for (int i = 0; i < 3; i++)
      {
         comPositionProportionalGains[i] = proportionalGains[i];
         comPositionDerivativeGains[i] = derivativeGains[i];
         comPositionIntegralGains[i] = integralGains[i];
      }
      comPositionMaxIntegralError = maxIntegralError;
   }

   public void setSolePositionFeedbackGainsToZero(RobotQuadrant robotQuadrant)
   {
      setSolePositionFeedbackGains(robotQuadrant, zeroGains, zeroGains, zeroGains, Double.MAX_VALUE);
   }

   public void setSolePositionFeedbackGains(RobotQuadrant robotQuadrant, double[] proportionalGains)
   {
      setSolePositionFeedbackGains(robotQuadrant, proportionalGains, zeroGains, zeroGains, Double.MAX_VALUE);
   }

   public void setSolePositionFeedbackGains(RobotQuadrant robotQuadrant, double[] proportionalGains, double[] derivativeGains)
   {
      setSolePositionFeedbackGains(robotQuadrant, proportionalGains, derivativeGains, zeroGains, Double.MAX_VALUE);
   }

   public void setSolePositionFeedbackGains(RobotQuadrant robotQuadrant, double[] proportionalGains, double[] derivativeGains, double[] integralGains)
   {
      setSolePositionFeedbackGains(robotQuadrant, proportionalGains, derivativeGains, integralGains, Double.MAX_VALUE);
   }

   public void setSolePositionFeedbackGains(RobotQuadrant robotQuadrant, double[] proportionalGains, double[] derivativeGains, double[] integralGains, double maxIntegralError)
   {
      for (int i = 0; i < 3; i++)
      {
         solePositionProportionalGains.get(robotQuadrant)[i] = proportionalGains[i];
         solePositionDerivativeGains.get(robotQuadrant)[i] = derivativeGains[i];
         solePositionIntegralGains.get(robotQuadrant)[i] = integralGains[i];
      }
      solePositionMaxIntegralError.get(robotQuadrant).setValue(maxIntegralError);
   }

   public ContactState getContactState(RobotQuadrant robotQuadrant)
   {
      return contactState.get(robotQuadrant);
   }

   public QuadrantDependentList<ContactState> getContactState()
   {
      return contactState;
   }

   public void getContactForceLimits(QuadrupedContactForceLimits contactForceLimits)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         contactForceLimits.setPressureUpperLimit(robotQuadrant, contactPressureUpperLimit.get(robotQuadrant).getValue());
      }
   }

   public void getContactForceOptimizationSettings(QuadrupedContactForceOptimizationSettings contactForceOptimizationSettings)
   {
      contactForceOptimizationSettings.setComForceCommandWeights(comForceCommandWeights);
      contactForceOptimizationSettings.setComTorqueCommandWeights(comTorqueCommandWeights);
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         contactForceOptimizationSettings.setContactForceCommandWeights(robotQuadrant, soleForceCommandWeights.get(robotQuadrant));
      }
   }

   public void getVirtualModelControllerSettings(QuadrupedVirtualModelControllerSettings virtualModelControllerSettings)
   {
      virtualModelControllerSettings.setJointDamping(jointDamping);
   }

   public void getBodyOrientationFeedbackGains(YoAxisAngleOrientationGains bodyOrientationFeedbackGains)
   {
      bodyOrientationFeedbackGains.setProportionalGains(bodyOrientationProportionalGains);
      bodyOrientationFeedbackGains.setIntegralGains(bodyOrientationIntegralGains, bodyOrientationMaxIntegralError);
      bodyOrientationFeedbackGains.setDerivativeGains(bodyOrientationDerivativeGains);
   }

   public void getComPositionFeedbackGains(YoEuclideanPositionGains comPositionFeedbackGains)
   {
      comPositionFeedbackGains.setProportionalGains(comPositionProportionalGains);
      comPositionFeedbackGains.setIntegralGains(comPositionIntegralGains, comPositionMaxIntegralError);
      comPositionFeedbackGains.setDerivativeGains(comPositionDerivativeGains);
   }

   public void getSolePositionFeedbackGains(RobotQuadrant robotQuadrant, YoEuclideanPositionGains solePositionFeedbackGains)
   {
      solePositionFeedbackGains.setProportionalGains(solePositionProportionalGains.get(robotQuadrant));
      solePositionFeedbackGains.setIntegralGains(solePositionIntegralGains.get(robotQuadrant), solePositionMaxIntegralError.get(robotQuadrant).getValue());
      solePositionFeedbackGains.setDerivativeGains(solePositionDerivativeGains.get(robotQuadrant));
   }
}

