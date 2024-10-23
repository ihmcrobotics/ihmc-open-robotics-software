package us.ihmc.avatar.drcRobot;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.commons.robotics.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.commons.robotics.outputData.JointDesiredOutputReadOnly;
import us.ihmc.simulationToolkit.controllers.JointLowLevelControlSimulator;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.yoVariables.registry.YoRegistry;

public class DefaultSimulationLowLevelControllerFactory implements SimulationLowLevelControllerFactory
{
   private final HumanoidJointNameMap jointMap;
   private final double simulateDT;

   public DefaultSimulationLowLevelControllerFactory(HumanoidJointNameMap jointMap, double simulateDT)
   {
      this.jointMap = jointMap;
      this.simulateDT = simulateDT;
   }

   @Override
   public RobotController createLowLevelController(FullRobotModel controllerRobot, Robot simulatedRobot,
                                                   JointDesiredOutputListReadOnly controllerDesiredOutputList)
   {
      String[] positionControlledJointNames = jointMap.getPositionControlledJointsForSimulation();

      if (positionControlledJointNames == null)
         return null;

      List<RobotController> jointControllers = new ArrayList<>();

      for (String positionControlledJointName : positionControlledJointNames)
      {
         Joint simulatedJoint = simulatedRobot.getJoint(positionControlledJointName);
         if (!(simulatedJoint instanceof OneDegreeOfFreedomJoint simulatedOneDoFJoint))
            continue;

         OneDoFJointBasics controllerOneDoFJoint = controllerRobot.getOneDoFJointByName(positionControlledJointName);
         if (controllerOneDoFJoint == null)
            continue;

         JointDesiredOutputReadOnly controllerDesiredOutput = controllerDesiredOutputList.getJointDesiredOutput(controllerOneDoFJoint);
         jointControllers.add(new JointLowLevelControlSimulator(simulatedOneDoFJoint, controllerDesiredOutput, simulateDT));
      }

      YoRegistry lowLevelRegistry = new YoRegistry("DefaultSimulationLowLevelController");
      jointControllers.forEach(controller -> lowLevelRegistry.addChild(controller.getYoRegistry()));

      return new RobotController()
      {
         @Override
         public void initialize()
         {
            for (int i = 0; i < jointControllers.size(); i++)
            {
               jointControllers.get(i).initialize();
            }
         }

         @Override
         public void doControl()
         {
            for (int i = 0; i < jointControllers.size(); i++)
            {
               jointControllers.get(i).doControl();
            }
         }

         @Override
         public YoRegistry getYoRegistry()
         {
            return lowLevelRegistry;
         }
      };
   }
}
