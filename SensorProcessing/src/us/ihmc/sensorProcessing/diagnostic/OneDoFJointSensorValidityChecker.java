package us.ihmc.sensorProcessing.diagnostic;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;

public class OneDoFJointSensorValidityChecker implements DiagnosticUpdatable
{
   private final YoVariableRegistry registry;
   private final DoubleYoVariableValidityChecker positionChecker;
   private final DoubleYoVariableValidityChecker velocityChecker;
   private final DoubleYoVariableValidityChecker tauChecker;

   private final OneDoFJoint joint;

   private final boolean hasYoVariables;

   public OneDoFJointSensorValidityChecker(OneDoFJoint jointToCheck, DoubleYoVariable position, DoubleYoVariable velocity, DoubleYoVariable tau,
         YoVariableRegistry parentRegistry)
   {
      hasYoVariables = true;
      this.joint = jointToCheck;
      String jointName = jointToCheck.getName();
      registry = new YoVariableRegistry(jointName + "JointSensorValidityChecker");
      parentRegistry.addChild(registry);

      verifyYoVariableNames(jointName, position, velocity, tau);

      positionChecker = new DoubleYoVariableValidityChecker(position, registry);
      velocityChecker = new DoubleYoVariableValidityChecker(velocity, registry);
      tauChecker = new DoubleYoVariableValidityChecker(tau, registry);
   }

   public OneDoFJointSensorValidityChecker(OneDoFJoint jointToCheck, YoVariableRegistry parentRegistry)
   {
      hasYoVariables = false;
      this.joint = jointToCheck;
      String jointName = jointToCheck.getName();

      registry = new YoVariableRegistry(jointName + "SensorValidityChecker");
      parentRegistry.addChild(registry);

      positionChecker = new DoubleYoVariableValidityChecker(jointName + "Position", registry);
      velocityChecker = new DoubleYoVariableValidityChecker(jointName + "Velocity", registry);
      tauChecker = new DoubleYoVariableValidityChecker(jointName + "Tau", registry);
   }

   @Override
   public void update()
   {
      if (hasYoVariables)
      {
         positionChecker.update();
         velocityChecker.update();
         tauChecker.update();
      }
      else
      {
         positionChecker.update(joint.getQ());
         velocityChecker.update(joint.getQd());
         tauChecker.update(joint.getTau());
      }
   }

   public void setupForLogging()
   {
      String loggerName = registry.getName();
      positionChecker.setupForLogging(loggerName);
      velocityChecker.setupForLogging(loggerName);
      tauChecker.setupForLogging(loggerName);
   }

   public boolean areSensorValuesSane()
   {
      return positionChecker.isInputSane() && velocityChecker.isInputSane() && tauChecker.isInputSane();
   }

   public boolean areAllSensorsAlive()
   {
      return positionChecker.isInputAlive() && velocityChecker.isInputAlive() && tauChecker.isInputAlive();
   }

   public boolean sensorsCannotBeTrusted()
   {
      return positionChecker.variableCannotBeTrusted() || velocityChecker.variableCannotBeTrusted() || tauChecker.variableCannotBeTrusted();
   }

   private void verifyYoVariableNames(String jointName, DoubleYoVariable position, DoubleYoVariable velocity, DoubleYoVariable tau)
   {
      if (!position.getName().contains(jointName))
         throw new RuntimeException("The position variable: " + position.getName() + " may not belong to the joint: " + jointName);
      if (!velocity.getName().contains(jointName))
         throw new RuntimeException("The velocity variable: " + velocity.getName() + " may not belong to the joint: " + jointName);
      if (!tau.getName().contains(jointName))
         throw new RuntimeException("The tau variable: " + tau.getName() + " may not belong to the joint: " + jointName);
   }
}
