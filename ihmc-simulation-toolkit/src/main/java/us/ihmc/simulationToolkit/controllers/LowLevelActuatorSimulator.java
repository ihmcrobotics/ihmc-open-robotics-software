package us.ihmc.simulationToolkit.controllers;

import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.controllers.pidGains.PIDGainsReadOnly;
import us.ihmc.robotics.controllers.pidGains.implementations.YoPIDGains;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.sensorProcessing.outputData.LowLevelActuatorMode;
import us.ihmc.sensorProcessing.outputData.LowLevelStateReadOnly;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class LowLevelActuatorSimulator implements RobotController
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry;
   private final PIDController jointController;

   private final OneDegreeOfFreedomJoint simulatedJoint;
   private final LowLevelStateReadOnly actuatorDesireds;

   private LowLevelActuatorMode actuatorMode;

   private final YoPIDGains gains;
   private final double controlDT;

   public LowLevelActuatorSimulator(OneDegreeOfFreedomJoint simulatedJoint, LowLevelStateReadOnly jointDesiredOutput, double controlDT)
   {
      this(simulatedJoint, jointDesiredOutput, controlDT, 0.0, 0.0, LowLevelActuatorMode.EFFORT);
   }

   public LowLevelActuatorSimulator(OneDegreeOfFreedomJoint simulatedJoint, LowLevelStateReadOnly jointDesiredOutput, double controlDT, LowLevelActuatorMode actuatorMode)
   {
      this(simulatedJoint, jointDesiredOutput, controlDT, 0.0, 0.0, actuatorMode);
   }

   public LowLevelActuatorSimulator(OneDegreeOfFreedomJoint simulatedJoint, LowLevelStateReadOnly jointDesiredOutput, double controlDT, double kp, double kd,
                                    LowLevelActuatorMode actuatorMode)
   {
      this.controlDT = controlDT;
      this.actuatorMode = actuatorMode;
      registry = new YoVariableRegistry(simulatedJoint.getName() + name);
      gains = new YoPIDGains(simulatedJoint.getName() + "Actuator", registry);
      jointController = new PIDController(gains, simulatedJoint.getName() + "LowLevelActuatorSimulator", registry);

      gains.setKp(kp);
      gains.setKd(kd);

      this.simulatedJoint = simulatedJoint;
      this.actuatorDesireds = jointDesiredOutput;
   }

   public void setKp(double kp)
   {
      gains.setKp(kp);
   }

   public void setKd(double kd)
   {
      gains.setKd(kd);
   }

   public void setGains(PIDGainsReadOnly gains)
   {
      this.gains.set(gains);
   }

   public void setActuatorMode(LowLevelActuatorMode actuatorMode)
   {
      this.actuatorMode = actuatorMode;
   }

   @Override
   public void doControl()
   {
      switch (actuatorMode)
      {
      case POSITION:
      {
         if (!actuatorDesireds.isPositionValid())
            throw new RuntimeException("Joint " + simulatedJoint.getName() + " cannot be in position control mode without a valid position.");

         double currentPosition = simulatedJoint.getQYoVariable().getDoubleValue();
         double desiredPosition = actuatorDesireds.getPosition();

         double desiredRate = 0.0;
         if (actuatorDesireds.isVelocityValid())
            desiredRate = actuatorDesireds.getVelocity();
         double currentRate = simulatedJoint.getQDYoVariable().getDoubleValue();

         double outputEffort = jointController.computeForAngles(currentPosition, desiredPosition, currentRate, desiredRate, controlDT);

         simulatedJoint.setTau(outputEffort);
      }

      break;
      case VELOCITY:
      {
         if (!actuatorDesireds.isVelocityValid())
            throw new RuntimeException("Joint " + simulatedJoint.getName() + " cannot be in velocity control mode without a valid velocity.");
         double currentRate = simulatedJoint.getQDYoVariable().getDoubleValue();
         double desiredRate = actuatorDesireds.getVelocity();

         double outputEffort = jointController.compute(currentRate, desiredRate, 0.0, 0.0, controlDT);

         simulatedJoint.setTau(outputEffort);
      }
      break;
      case EFFORT:
      {
         double currentPosition = simulatedJoint.getQ();
         double desiredPosition;
         if (actuatorDesireds.isPositionValid())
         {
            desiredPosition = actuatorDesireds.getPosition();
         }
         else
         {
            desiredPosition = currentPosition;
         }

         double currentRate = simulatedJoint.getQDYoVariable().getDoubleValue();
         double desiredRate = 0.0;
         if (actuatorDesireds.isVelocityValid())
            desiredRate = actuatorDesireds.getVelocity();

         double desiredEffort = jointController.compute(currentPosition, desiredPosition, currentRate, desiredRate, controlDT);

         if (actuatorDesireds.isEffortValid())
            desiredEffort += actuatorDesireds.getEffort();

         simulatedJoint.setTau(desiredEffort);
      }
      break;
      case DISABLED:
      {
         simulatedJoint.setTau(0.0);
      }
      }
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return name;
   }

   @Override
   public String getDescription()
   {
      return getName();
   }
}
