package us.ihmc.exampleSimulations.fourBarLinkage;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.simulationconstructionset.RobotFromDescription;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;

public class CrossFourBarLinkageSimulation
{
   public CrossFourBarLinkageSimulation()
   {
      CrossFourBarLinkageRobotDescription robotDescription = new CrossFourBarLinkageRobotDescription();
      RobotFromDescription robot = new RobotFromDescription(robotDescription);

      double dt = 1.0e-5;
      robot.setController(new CrossFourBarOneDoFJointWBCController(robotDescription, robot, dt));
      //      robot.setController(new CrossFourBarLinkageIDController(robotDescription, robot));
//            robot.setController(new CrossFourBarLinkageWBCController(robotDescription, robot, dt));

      SimulationConstructionSet scs = new SimulationConstructionSet(robot, new SimulationConstructionSetParameters(1 << 17));
      Graphics3DObject graphics3dObject = new Graphics3DObject();
      graphics3dObject.addCoordinateSystem(0.3, YoAppearance.Blue());
      scs.addStaticLinkGraphics(graphics3dObject);
      scs.setFastSimulate(true);
      scs.setGroundVisible(false);
      scs.setDT(dt, 10);
      scs.startOnAThread();
      scs.simulate(0.5);
   }

   public static void main(String[] args)
   {
      new CrossFourBarLinkageSimulation();
   }
}
