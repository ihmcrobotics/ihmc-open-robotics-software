package us.ihmc.exampleSimulations.fourBarLinkage;

import javax.vecmath.Vector3d;

import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class FourBarLinkageSimulation
{
   private Vector3d offsetWorld = new Vector3d(0.0, 0.0, 0.0);
   private static final double simDT = 1.0e-4;
   private static final int recordFrequency = 10;

   private static final MethodToCloseLoop METHOD_TO_CLOSE_LOOP = MethodToCloseLoop.STIFF_AXIAL_FUNCTION_TO_INTEGRATE;

   public FourBarLinkageSimulation(FourBarLinkageParameters fourBarLinkageParameters)
   {
      YoVariableRegistry registry = new YoVariableRegistry("fourBarLinkageRegistry");

      // Robot
      FourBarLinkageRobot robot = new FourBarLinkageRobot("basicFourBar", fourBarLinkageParameters, offsetWorld, registry);

      // Four bar constraint
      switch (METHOD_TO_CLOSE_LOOP)
      {
      case PD_CONTROLLER:
         robot.setController(new FourBarLinkageSimpleClosedLoopConstraintController(robot));
         break;
      case SIMPLE_FUNCTION_TO_INTEGRATE:
         FourBarLinkagePDConstraintToIntegrate fourBarLinkagePDConstraintToIntegrate = new FourBarLinkagePDConstraintToIntegrate("fourBarIntegratedConstraint",
               robot.getEfpJoint1to2(), robot.getEfpJoint1to4(), robot.getRobotsYoVariableRegistry());
         fourBarLinkagePDConstraintToIntegrate.setStiffness(1000.0);
         fourBarLinkagePDConstraintToIntegrate.setDamping(100.0);

         robot.addFunctionToIntegrate(fourBarLinkagePDConstraintToIntegrate);
         break;
      case STIFF_AXIAL_FUNCTION_TO_INTEGRATE:
         FourBarLinkageConstraintToIntegrate fourBarLinkageConstraintToIntegrate = new FourBarLinkageConstraintToIntegrate("fourBarIntegratedConstraint",
               robot.getEfpJoint1to2(), robot.getEfpJoint1to4(), robot.getRootJoints().get(0), robot.getRobotsYoVariableRegistry());
         fourBarLinkageConstraintToIntegrate.setAxialStiffness(1000.0);
         fourBarLinkageConstraintToIntegrate.setAxialDamping(100.0);
         fourBarLinkageConstraintToIntegrate.setRadialStiffness(100.0);
         fourBarLinkageConstraintToIntegrate.setRadialDamping(10.0);

         robot.addFunctionToIntegrate(fourBarLinkageConstraintToIntegrate);
         break;
      }
      
      // Controller
      FourBarLinkageController controller = new FourBarLinkageController(robot, "fourBarLinkageController", fourBarLinkageParameters);
      robot.setController(controller);

      // SCS
      SimulationConstructionSet scs = new SimulationConstructionSet(robot);
      scs.setSimulateDuration(3.0);
      scs.setGroundVisible(false);
      scs.setDT(simDT, recordFrequency);
      scs.addYoVariableRegistry(registry);
      scs.setCameraFix(0.335, 0.0, -0.5);
      scs.setCameraPosition(0.355, -11.2, -0.75);
      scs.startOnAThread();
   }

   public static void main(String[] args)
   {
      FourBarLinkageParameters fourBarLinkageParameters = new FourBarLinkageParameters();
      fourBarLinkageParameters.linkageLength_1 = fourBarLinkageParameters.linkageLength_2 = fourBarLinkageParameters.linkageLength_3 = fourBarLinkageParameters.linkageLength_4 = 1.0;
      fourBarLinkageParameters.damping_1 = fourBarLinkageParameters.damping_2 = fourBarLinkageParameters.damping_3 = 0.05;
      fourBarLinkageParameters.mass_1 = fourBarLinkageParameters.mass_2 = fourBarLinkageParameters.mass_3 = fourBarLinkageParameters.mass_4 = 0.3;
      fourBarLinkageParameters.radius_1 = fourBarLinkageParameters.radius_2 = fourBarLinkageParameters.radius_3 = fourBarLinkageParameters.radius_4 = 0.03;
      fourBarLinkageParameters.angle_1 = fourBarLinkageParameters.angle_2 = fourBarLinkageParameters.angle_3 = 0.5 * Math.PI;

      new FourBarLinkageSimulation(fourBarLinkageParameters);
   }

   private enum MethodToCloseLoop
   {
      PD_CONTROLLER, SIMPLE_FUNCTION_TO_INTEGRATE, STIFF_AXIAL_FUNCTION_TO_INTEGRATE
   }
}