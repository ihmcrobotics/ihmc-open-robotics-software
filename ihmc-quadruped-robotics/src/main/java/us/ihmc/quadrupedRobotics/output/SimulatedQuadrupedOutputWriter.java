package us.ihmc.quadrupedRobotics.output;

import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotModels.OutputWriter;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.sensorProcessing.outputData.*;
import us.ihmc.simulationToolkit.controllers.LowLevelActuatorSimulator;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class SimulatedQuadrupedOutputWriter implements OutputWriter
{
   private final OneDoFJoint[] controllerJoints;

   protected final JointDesiredOutputListReadOnly jointDesiredOutputList;
   protected final LowLevelStateList lowLevelStateList;
   private final HashMap<OneDoFJoint, LowLevelActuatorSimulator> quadrupedActuators = new HashMap<>();

//   private final List<QuadrupedJointController> quadrupedJointControllers = new ArrayList<>();


   public SimulatedQuadrupedOutputWriter(FloatingRootJointRobot robot, FullRobotModel fullRobotModel, JointDesiredOutputList jointDesiredOutputList,
                                         double controlDT)
   {
      this.jointDesiredOutputList = jointDesiredOutputList;
      controllerJoints = fullRobotModel.getOneDoFJoints();
      lowLevelStateList = new LowLevelStateList(controllerJoints);

      YoVariableRegistry registry = new YoVariableRegistry("quadrupedOutputWriter");
      for (OneDoFJoint controllerJoint : controllerJoints)
      {
         String name = controllerJoint.getName();
         OneDegreeOfFreedomJoint simulatedJoint = robot.getOneDegreeOfFreedomJoint(name);

         LowLevelActuatorSimulator actuator = new LowLevelActuatorSimulator(simulatedJoint, lowLevelStateList.getLowLevelState(controllerJoint), controlDT);
         quadrupedActuators.put(controllerJoint, actuator);
         robot.setController(actuator);

//         quadrupedJointControllers.add(new QuadrupedJointController(controllerJoint, jointDesiredOutputList.getJointDesiredOutput(controllerJoint), registry));
      }

      robot.getRobotsYoVariableRegistry().addChild(registry);
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public void setFullRobotModel(FullRobotModel fullRobotModel)
   {
      throw new RuntimeException("This should have been done earlier.");
   }

   @Override
   public void write()
   {
//      for (int i = 0; i < controllerJoints.length; i++)
//      {
//         quadrupedJointControllers.get(i).computeDesiredStateFromJointController();
//      }

      for (OneDoFJoint controllerJoint : controllerJoints)
      {
         LowLevelState desiredState = lowLevelStateList.getLowLevelState(controllerJoint);
         JointDesiredOutputReadOnly jointSetpoints = jointDesiredOutputList.getJointDesiredOutput(controllerJoint);
         desiredState.clear();

         // Pass through setpoints
         if (jointSetpoints.hasDesiredAcceleration())
            desiredState.setAcceleration(jointSetpoints.getDesiredAcceleration());
         if (jointSetpoints.hasDesiredVelocity())
            desiredState.setVelocity(jointSetpoints.getDesiredVelocity());
         if (jointSetpoints.hasDesiredPosition())
            desiredState.setPosition(jointSetpoints.getDesiredPosition());
         if (jointSetpoints.hasDesiredTorque())
            desiredState.setEffort(jointSetpoints.getDesiredTorque());

         // Apply velocity scaling
         if (desiredState.isVelocityValid() && jointSetpoints.hasVelocityScaling())
            desiredState.setVelocity(jointSetpoints.getVelocityScaling() * desiredState.getVelocity());

         LowLevelActuatorSimulator actuator = quadrupedActuators.get(controllerJoint);

         if (jointSetpoints.hasStiffness())
         {
            QuadrupedJointGainScheduler.QuadrupedJointGains jointGains = QuadrupedJointGainScheduler.getCorrespondingJointGains(jointSetpoints.getStiffness());
            actuator.setKp(jointGains.stiffness());
            actuator.setKd(jointGains.damping());
         }
         actuator.setActuatorMode(getDesiredActuatorMode(jointSetpoints));
      }
   }

   private LowLevelActuatorMode getDesiredActuatorMode(JointDesiredOutputReadOnly jointDesiredSetpoints)
   {
      if (jointDesiredSetpoints.hasControlMode())
      {
         switch (jointDesiredSetpoints.getControlMode())
         {
         case POSITION:
            return LowLevelActuatorMode.POSITION;
         case VELOCITY:
            return LowLevelActuatorMode.VELOCITY;
         case EFFORT:
            return LowLevelActuatorMode.EFFORT;
         case DISABLED:
            return LowLevelActuatorMode.DISABLED;
         default:
            throw new RuntimeException("Control mode " + jointDesiredSetpoints.getControlMode() + " not implemented.");
         }
      }
      return LowLevelActuatorMode.DISABLED;
   }
}
