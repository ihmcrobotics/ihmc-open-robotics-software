package us.ihmc.exampleSimulations.fourBarLinkage;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.simulationconstructionset.RobotFromDescription;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class InvertedFourBarLinkageSimulation
{
   public InvertedFourBarLinkageSimulation()
   {
      InvertedFourBarLinkageRobotDescription robotDescription = new InvertedFourBarLinkageRobotDescription();
      RobotFromDescription robot = new RobotFromDescription(robotDescription);

      SimulationConstructionSet scs = new SimulationConstructionSet(robot);
      Graphics3DObject graphics3dObject = new Graphics3DObject();
      graphics3dObject.translate(0.0, 0.0, 2.0);
      graphics3dObject.addCoordinateSystem(0.3, YoAppearance.Blue());
      scs.addStaticLinkGraphics(graphics3dObject);
      scs.startOnAThread();
      scs.simulate(1.0);
   }

   public static void main(String[] args)
   {
      new InvertedFourBarLinkageSimulation();
   }
}
