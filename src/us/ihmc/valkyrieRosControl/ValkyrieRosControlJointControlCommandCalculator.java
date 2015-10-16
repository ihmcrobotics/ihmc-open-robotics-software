package us.ihmc.valkyrieRosControl;

import java.util.Map;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.simulationconstructionset.util.math.functionGenerator.YoFunctionGenerator;
import us.ihmc.simulationconstructionset.util.math.functionGenerator.YoFunctionGeneratorMode;
import us.ihmc.valkyrieRosControl.dataHolders.YoJointHandleHolder;

public class ValkyrieRosControlJointControlCommandCalculator
{
   private static final double standPrepTrajectoryTime = 5.0;

   private final YoJointHandleHolder yoJointHandleHolder;

   private final YoPolynomial trajectory;

   private final PIDController pidController;
   private final DoubleYoVariable tauOff;
   private final DoubleYoVariable standPrepAngle, trajectoryTime;
   private final DoubleYoVariable desiredPosition, desiredVelocity;
   private final YoFunctionGenerator functionGenerator;

   private final double controlDT;

   public ValkyrieRosControlJointControlCommandCalculator(YoJointHandleHolder yoJointHandleHolder, Map<String, Double> gains, Map<String, Double> offsets,
         double standPrepAngle, double controlDT, YoVariableRegistry parentRegistry)
   {
      this.yoJointHandleHolder = yoJointHandleHolder;

      this.controlDT = controlDT;

      String pdControllerBaseName = yoJointHandleHolder.getName();
      YoVariableRegistry registry = new YoVariableRegistry(pdControllerBaseName + "Command");
      trajectory = new YoPolynomial(pdControllerBaseName + "StandPrepTrajectory", 4, registry);

      this.standPrepAngle = new DoubleYoVariable(pdControllerBaseName + "StandPrepAngle", registry);
      this.trajectoryTime = new DoubleYoVariable(pdControllerBaseName + "StandPrepTrajectoryTime", registry);

      pidController = new PIDController(pdControllerBaseName + "StandPrep", registry);
      this.tauOff = new DoubleYoVariable(pdControllerBaseName + "StandPrep_tauOff", registry);
      this.functionGenerator = new YoFunctionGenerator(pdControllerBaseName + "StandPrep_", registry);

      this.desiredPosition = new DoubleYoVariable(pdControllerBaseName + "StandPrep_q_d", registry);
      this.desiredVelocity = new DoubleYoVariable(pdControllerBaseName + "StandPrep_qd_d", registry);

      pidController.setProportionalGain(gains.get("kp"));
      pidController.setDerivativeGain(gains.get("kd"));
      pidController.setIntegralGain(gains.get("ki"));
      pidController.setMaxIntegralError(50.0);
      pidController.setCumulativeError(0.0);
      this.tauOff.set(offsets.get("tauOff"));

      this.standPrepAngle.set(standPrepAngle);
      this.trajectoryTime.set(standPrepTrajectoryTime);

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      double desiredAngle = standPrepAngle.getDoubleValue();
      double desiredVelocity = 0.0;

      double currentAngle = yoJointHandleHolder.getQ();
      double currentVelocity = yoJointHandleHolder.getQd();

      functionGenerator.setMode(YoFunctionGeneratorMode.DC);
      pidController.setCumulativeError(0.0);
      trajectory.setCubic(0.0, trajectoryTime.getDoubleValue(), currentAngle, currentVelocity, desiredAngle, desiredVelocity);
   }

   public void computeAndUpdateJointTorque(double inStateTime, double factor, double masterGain)
   {

      double standPrepFactor = 1.0 - factor;
      double trajectoryTime = MathTools.clipToMinMax(inStateTime, 0.0, this.trajectoryTime.getDoubleValue());

      trajectory.compute(trajectoryTime);

      desiredPosition.set(trajectory.getPosition() + functionGenerator.getValue(inStateTime));

      desiredVelocity.set(trajectory.getVelocity());

      factor = MathTools.clipToMinMax(factor, 0.0, 1.0);

      double q = yoJointHandleHolder.getQ();
      double qDesired = desiredPosition.getDoubleValue();
      double qd = yoJointHandleHolder.getQd();
      double qdDesired = desiredVelocity.getDoubleValue();

      double standPrepTau = standPrepFactor * masterGain * pidController.compute(q, qDesired, qd, qdDesired, controlDT);
      double controllerTau = factor * yoJointHandleHolder.getControllerTauDesired();

      double desiredEffort = standPrepTau + controllerTau;
      yoJointHandleHolder.setDesiredEffort(desiredEffort);

   }
}
