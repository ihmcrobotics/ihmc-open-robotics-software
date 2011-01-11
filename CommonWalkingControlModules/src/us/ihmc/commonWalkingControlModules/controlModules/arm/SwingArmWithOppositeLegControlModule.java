package us.ihmc.commonWalkingControlModules.controlModules.armControlModules;

import us.ihmc.commonWalkingControlModules.RobotSide;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.ArmJointName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointName;
import us.ihmc.commonWalkingControlModules.sensors.ProcessedSensorsInterface;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class SwingArmWithOppositeLegControlModule extends PDArmControlModule
{
   private final DoubleYoVariable armLegCoupling = new DoubleYoVariable("armLegCoupling", "Coupling amplitude between arm and opposite leg motion", registry);

   public SwingArmWithOppositeLegControlModule(ProcessedSensorsInterface processedSensors, double controlDT, YoVariableRegistry parentRegistry)
   {
      super(processedSensors, controlDT, parentRegistry);
      armLegCoupling.set(0.5);
   }

   protected void computeDesireds()
   {
      for (RobotSide robotSide : RobotSide.values())
      {
         for (ArmJointName armJointName : ArmJointName.values())
         {
            double desiredPosition = 0.0;
            double desiredVelocity = 0.0;

            if (armJointName == ArmJointName.SHOULDER_PITCH)
            {
               LegJointName hipPitch = LegJointName.HIP_PITCH;

               double hipPitchPosition = processedSensors.getLegJointPosition(robotSide.getOppositeSide(), hipPitch);
               desiredPosition = hipPitchPosition * armLegCoupling.getDoubleValue();

               double hipPitchVelocity = processedSensors.getLegJointVelocity(robotSide.getOppositeSide(), hipPitch);
               desiredVelocity = hipPitchVelocity * armLegCoupling.getDoubleValue();
            }

            desiredArmPositions.get(robotSide).get(armJointName).set(desiredPosition);
            desiredArmVelocities.get(robotSide).get(armJointName).set(desiredVelocity);
         }
      }
   }

   protected void setGains()
   {
      for (RobotSide robotSide : RobotSide.values())
      {
         armControllers.get(robotSide).get(ArmJointName.SHOULDER_PITCH).setProportionalGain(100.0);
         armControllers.get(robotSide).get(ArmJointName.SHOULDER_ROLL).setProportionalGain(100.0);
         armControllers.get(robotSide).get(ArmJointName.SHOULDER_YAW).setProportionalGain(100.0);
         armControllers.get(robotSide).get(ArmJointName.ELBOW).setProportionalGain(100.0);

         armControllers.get(robotSide).get(ArmJointName.SHOULDER_PITCH).setDerivativeGain(10.0);
         armControllers.get(robotSide).get(ArmJointName.SHOULDER_ROLL).setDerivativeGain(10.0);
         armControllers.get(robotSide).get(ArmJointName.SHOULDER_YAW).setDerivativeGain(10.0);
         armControllers.get(robotSide).get(ArmJointName.ELBOW).setDerivativeGain(10.0);
      }
   }

}
