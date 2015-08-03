package us.ihmc.darpaRoboticsChallenge.controllers;

import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.robotController.RobotController;
import us.ihmc.yoUtilities.controllers.PDController;
import us.ihmc.yoUtilities.controllers.YoPDGains;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

public class PassiveJointController implements RobotController
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry;
   private final PDController jointController;

   private final OneDegreeOfFreedomJoint simulatedJoint;

   public PassiveJointController(OneDegreeOfFreedomJoint simulatedJoint, YoPDGains gains)
   {
      registry = new YoVariableRegistry(simulatedJoint.getName() + name);
      jointController = new PDController(gains.getYoKp(), gains.getYoKd(), simulatedJoint.getName() + "PassiveController", registry);

      this.simulatedJoint = simulatedJoint;

      jointController.setProportionalGain(36000.0);
      jointController.setDerivativeGain(1000.0);
   }

   public void doControl()
   {
      double currentPosition = simulatedJoint.getQ().getDoubleValue();
      double desiredPosition = 0.0;
      double currentRate = simulatedJoint.getQD().getDoubleValue();
      double desiredRate = 0.0;
      double desiredTau = jointController.compute(currentPosition, desiredPosition, currentRate, desiredRate);
      simulatedJoint.setTau(desiredTau);
   }

   public void initialize()
   {
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public String getName()
   {
      return name;
   }

   public String getDescription()
   {
      return getName();
   }
}
