package us.ihmc.valkyrieRosControl;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.valkyrieRosControl.dataHolders.YoPositionJointHandleHolder;

import java.util.Map;

public class ValkyrieRosControlPositionJointControlCommandCalculator
{
   private final YoPositionJointHandleHolder yoPositionJointHandleHolder;

   private final PIDController pidController;
   private final DoubleYoVariable standPrepAngle;

   private final double controlDT;

   public ValkyrieRosControlPositionJointControlCommandCalculator(YoPositionJointHandleHolder yoPositionJointHandleHolder, Map<String, Double> gains,
         double standPrepAngle, double controlDT, YoVariableRegistry parentRegistry)
   {
      this.yoPositionJointHandleHolder = yoPositionJointHandleHolder;

      this.controlDT = controlDT;

      String pdControllerBaseName = yoPositionJointHandleHolder.getName();
      YoVariableRegistry registry = new YoVariableRegistry(pdControllerBaseName + "Command");

      this.standPrepAngle = new DoubleYoVariable(pdControllerBaseName + "StandPrepAngle", registry);

      pidController = new PIDController(pdControllerBaseName + "StandPrep", registry);

      pidController.setProportionalGain(gains.get("kp"));
      pidController.setDerivativeGain(gains.get("kd"));
      pidController.setIntegralGain(gains.get("ki"));
      pidController.setMaxIntegralError(50.0);
      pidController.setCumulativeError(0.0);

      this.standPrepAngle.set(standPrepAngle);

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      pidController.setCumulativeError(0.0);
   }

   public void computeAndUpdateJointPosition(double inStateTime, double factor, double masterGain)
   {
      double standPrepFactor = 1.0 - factor;

      factor = MathTools.clipToMinMax(factor, 0.0, 1.0);

      double q = yoPositionJointHandleHolder.getQ();
      double qDesired = standPrepAngle.getDoubleValue();
      double qd = yoPositionJointHandleHolder.getQd();
      double qdDesired = 0.0;

      double standPrepPosition = standPrepFactor * masterGain * pidController.compute(q, qDesired, qd, qdDesired, controlDT);
      double controllerPosition = factor * yoPositionJointHandleHolder.getControllerPositionDesired();

      double desiredPosition = standPrepPosition + controllerPosition;
      yoPositionJointHandleHolder.setDesiredPosition(desiredPosition);
   }
}
