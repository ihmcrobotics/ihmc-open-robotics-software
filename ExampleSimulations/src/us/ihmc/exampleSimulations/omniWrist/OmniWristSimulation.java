package us.ihmc.exampleSimulations.omniWrist;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.RobotFromDescription;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.physics.ExternalForcePointPDConstraintToIntegrate;

public class OmniWristSimulation
{

   public OmniWristSimulation()
   {
      OmniWristDescription description = new OmniWristDescription();
      RobotDescription robotDescription = description.createRobotDescription();

      RobotFromDescription robot = new RobotFromDescription(robotDescription);
//      Link.addEllipsoidFromMassPropertiesToAllLinks(robot, YoAppearance.Cornsilk());

      OmniWristController controller = new OmniWristController(robot);
      robot.setController(controller);

      DoubleYoVariable q_jointOneA = (DoubleYoVariable) robot.getRobotsYoVariableRegistry().getVariable("q_jointOneA");
      DoubleYoVariable q_jointOneB = (DoubleYoVariable) robot.getRobotsYoVariableRegistry().getVariable("q_jointOneB");
      DoubleYoVariable q_jointOneC = (DoubleYoVariable) robot.getRobotsYoVariableRegistry().getVariable("q_jointOneC");
      DoubleYoVariable q_jointOneD = (DoubleYoVariable) robot.getRobotsYoVariableRegistry().getVariable("q_jointOneD");
      DoubleYoVariable q_jointFourA = (DoubleYoVariable) robot.getRobotsYoVariableRegistry().getVariable("q_jointFourA");

      double initialAngle = -0.38;
      q_jointOneA.set(initialAngle);
      q_jointOneB.set(initialAngle);
      q_jointOneC.set(initialAngle);
      q_jointOneD.set(initialAngle);
      q_jointFourA.set(-initialAngle);     

      YoVariableRegistry registry = robot.getRobotsYoVariableRegistry();

      double weldStiffness = 2000000.0;
      double weldDamping = 500.0;
      
      createWeldConstraint("weldB1", robot, weldStiffness, weldDamping, "ef_B1", "ef_matchB1", registry);
      createWeldConstraint("weldB2", robot, weldStiffness, weldDamping, "ef_B2", "ef_matchB2", registry);
      createWeldConstraint("weldC1", robot, weldStiffness, weldDamping, "ef_C1", "ef_matchC1", registry);
      createWeldConstraint("weldC2", robot, weldStiffness, weldDamping, "ef_C2", "ef_matchC2", registry);
      createWeldConstraint("weldD1", robot, weldStiffness, weldDamping, "ef_D1", "ef_matchD1", registry);
      createWeldConstraint("weldD2", robot, weldStiffness, weldDamping, "ef_D2", "ef_matchD2", registry);

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setDataBufferSize(32000);

      SimulationConstructionSet scs = new SimulationConstructionSet(robot);
      scs.setDT(0.00002, 100);
      scs.setSimulateNoFasterThanRealTime(true);
      scs.setGroundVisible(false);

      Graphics3DObject staticLinkGraphics = new Graphics3DObject();
      staticLinkGraphics.addModelFile(OmniWristDescription.omniWristBaseModelFile, YoAppearance.Black());
      scs.addStaticLinkGraphics(staticLinkGraphics);

      scs.setCameraFix(0.0, 0.15, 0.0);
      scs.setCameraPosition(0.0, -0.4, 0.13);

      scs.startOnAThread();
      scs.simulate();
   }

   private void createWeldConstraint(String weldName, RobotFromDescription robot, double weldStiffness, double weldDamping,
                                     String externalForcePointName, String externalForcePointMatchName, YoVariableRegistry registry)
   {
      ExternalForcePoint ef_B1 = robot.getExternalForcePoint(externalForcePointName);
      ExternalForcePoint ef_matchB1 = robot.getExternalForcePoint(externalForcePointMatchName);
      ExternalForcePointPDConstraintToIntegrate weldJointB1 = new ExternalForcePointPDConstraintToIntegrate(weldName, ef_B1, ef_matchB1, registry);
      weldJointB1.setStiffness(weldStiffness);
      weldJointB1.setDamping(weldDamping);
      robot.addFunctionToIntegrate(weldJointB1);
   }

   public static void main(String[] args)
   {
      new OmniWristSimulation();
   }
}
