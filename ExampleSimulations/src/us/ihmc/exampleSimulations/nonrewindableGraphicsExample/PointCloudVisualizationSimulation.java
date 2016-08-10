package us.ihmc.exampleSimulations.nonrewindableGraphicsExample;

import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.graphics3DAdapter.structure.Graphics3DNode;
import us.ihmc.graphics3DAdapter.structure.Graphics3DNodeType;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class PointCloudVisualizationSimulation
{
   public PointCloudVisualizationSimulation()
   {
      PointCloudRobot pointCloudRobot = new PointCloudRobot("PointCloudRobot");
      SimulationConstructionSet scs = new SimulationConstructionSet(pointCloudRobot);

      Graphics3DObject testGraphicObject = new Graphics3DObject();
      testGraphicObject.addCube(0.1, 0.1, 0.1, YoAppearance.Aqua());
      Graphics3DNode node = scs.addStaticLinkGraphics(testGraphicObject, Graphics3DNodeType.VISUALIZATION);
      node.translate(1.0, 0, 0);

      scs.startOnAThread();
   }

   public static void main(String[] args)
   {
      new PointCloudVisualizationSimulation();
   }
}
