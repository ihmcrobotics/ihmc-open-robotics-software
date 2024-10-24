package us.ihmc.simulationToolkit.controllers;

import us.ihmc.robotics.controllers.PDController;
import us.ihmc.robotics.controllers.pidGains.implementations.YoPDGains;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.util.RobotController;

public class PassiveJointController implements RobotController
{
   private final String name = getClass().getSimpleName();
   private final YoRegistry registry;
   private final PDController jointController;

   private final OneDegreeOfFreedomJoint simulatedJoint;

   public PassiveJointController(OneDegreeOfFreedomJoint simulatedJoint, YoPDGains gains)
   {
      registry = new YoRegistry(simulatedJoint.getName() + name);
      jointController = new PDController(gains.getYoKp(), gains.getYoKd(), simulatedJoint.getName() + "PassiveController", registry);

      this.simulatedJoint = simulatedJoint;

      jointController.setProportionalGain(36000.0);
      jointController.setDerivativeGain(1000.0);
   }

   @Override
   public void doControl()
   {
      double currentPosition = simulatedJoint.getQYoVariable().getDoubleValue();
      double desiredPosition = 0.0;
      double currentRate = simulatedJoint.getQDYoVariable().getDoubleValue();
      double desiredRate = 0.0;
      double desiredTau = jointController.compute(currentPosition, desiredPosition, currentRate, desiredRate);
      simulatedJoint.setTau(desiredTau);
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public YoRegistry getYoRegistry()
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
