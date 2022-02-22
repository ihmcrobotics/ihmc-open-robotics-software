package us.ihmc.avatar.scs2;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.scs2.definition.controller.ControllerInput;
import us.ihmc.scs2.definition.controller.ControllerOutput;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.definition.state.interfaces.OneDoFJointStateBasics;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class SCS2PIDLidarTorqueController implements Controller
{
   private final String name = getClass().getSimpleName();
   private final YoRegistry registry = new YoRegistry(name);
   private final PIDController lidarJointController = new PIDController("lidar", registry);

   private final YoDouble desiredLidarAngle = new YoDouble("desiredLidarAngle", registry);
   private final YoDouble desiredLidarVelocity = new YoDouble("desiredLidarVelocity", registry);

   private final double controlDT;
   private final OneDoFJointReadOnly lidarJointState;
   private final OneDoFJointStateBasics lidarJointOutput;

   public SCS2PIDLidarTorqueController(ControllerInput controllerInput,
                                       ControllerOutput controllerOutput,
                                       String jointName,
                                       double desiredSpindleSpeed,
                                       double controlDT)
   {
      this.controlDT = controlDT;
      lidarJointState = (OneDoFJointReadOnly) controllerInput.getInput().findJoint(jointName);
      lidarJointOutput = controllerOutput.getOneDoFJointOutput(lidarJointState);

      desiredLidarVelocity.set(desiredSpindleSpeed);
      lidarJointController.setProportionalGain(10.0);
      lidarJointController.setDerivativeGain(1.0);
   }

   @Override
   public void doControl()
   {
      desiredLidarAngle.add(desiredLidarVelocity.getDoubleValue() * controlDT);

      double lidarJointTau = lidarJointController.compute(lidarJointState.getQ(),
                                                          desiredLidarAngle.getDoubleValue(),
                                                          lidarJointState.getQd(),
                                                          desiredLidarVelocity.getDoubleValue(),
                                                          controlDT);
      lidarJointOutput.setEffort(lidarJointTau);
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }
}
