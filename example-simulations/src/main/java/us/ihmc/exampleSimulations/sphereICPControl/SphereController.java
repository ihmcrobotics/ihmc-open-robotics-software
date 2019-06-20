package us.ihmc.exampleSimulations.sphereICPControl;

import us.ihmc.exampleSimulations.sphereICPControl.controllers.BasicSphereController;
import us.ihmc.exampleSimulations.sphereICPControl.controllers.GenericSphereController;
import us.ihmc.exampleSimulations.sphereICPControl.controllers.SphereControlToolbox;
import us.ihmc.exampleSimulations.sphereICPControl.controllers.SphereICPOptimizationController;
import us.ihmc.exampleSimulations.sphereICPControl.controllers.SphereNewICPController;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.simulationConstructionSetTools.tools.RobotTools;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class SphereController implements RobotController
{
   private enum SphereControllerEnum {BASIC, NEW_ICP, ICP_OPTIMIZATION}

   private static final SphereControllerEnum controllerType = SphereControllerEnum.ICP_OPTIMIZATION;

   private final YoVariableRegistry registry = new YoVariableRegistry("SphereController");

   private final GenericSphereController sphereController;

   private final RobotTools.SCSRobotFromInverseDynamicsRobotModel scsRobot;
   private final FullRobotModel robot;
   private final SphereControlToolbox controlToolbox;
   private final ExternalForcePoint externalForcePoint;

   public SphereController(RobotTools.SCSRobotFromInverseDynamicsRobotModel scsRobot, SphereControlToolbox controlToolbox, ExternalForcePoint externalForcePoint)
   {
      this.scsRobot = scsRobot;
      this.controlToolbox = controlToolbox;
      this.robot = controlToolbox.getFullRobotModel();
      this.externalForcePoint = externalForcePoint;

      switch(controllerType)
      {
      case BASIC:
         sphereController = new BasicSphereController(controlToolbox, registry);
         break;
      case NEW_ICP:
         sphereController = new SphereNewICPController(controlToolbox, registry);
         break;
      case ICP_OPTIMIZATION:
         sphereController = new SphereICPOptimizationController(controlToolbox, registry);
         break;
      default:
         sphereController = new BasicSphereController(controlToolbox, registry);
         break;
      }
   }

   @Override
   public void doControl()
   {
      scsRobot.updateJointPositions_SCS_to_ID();
      scsRobot.updateJointVelocities_SCS_to_ID();

      robot.updateFrames();
      controlToolbox.update();

      sphereController.doControl();

      externalForcePoint.setForce(sphereController.getForces());

      scsRobot.updateJointPositions_ID_to_SCS();
      scsRobot.updateJointVelocities_ID_to_SCS();
      scsRobot.updateJointTorques_ID_to_SCS();
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
   public String getDescription()
   {
      return registry.getName();
   }

   @Override
   public String getName()
   {
      return registry.getName();
   }

}
