package us.ihmc.avatar.reachabilityMap.example;

import us.ihmc.avatar.reachabilityMap.ReachabilityMapTools;
import us.ihmc.avatar.reachabilityMap.ReachabilitySphereMapCalculator;
import us.ihmc.avatar.reachabilityMap.Voxel3DGrid;
import us.ihmc.avatar.reachabilityMap.example.RobotParameters.RobotArmJointParameters;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.iterators.SubtreeStreams;
import us.ihmc.scs2.definition.visual.VisualDefinitionFactory;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizer;
import us.ihmc.scs2.sessionVisualizer.jfx.SessionVisualizerControls;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.SimulationSessionControls;
import us.ihmc.scs2.simulation.robot.Robot;

public class ReachabilitySphereMapExample
{
   public ReachabilitySphereMapExample()
   {
      RobotArmDefinition robotDefinition = new RobotArmDefinition();
      robotDefinition.ignoreAllJoints();

      RigidBodyBasics rootBody = robotDefinition.newInstance(ReferenceFrame.getWorldFrame());
      OneDoFJointBasics[] armJoints = SubtreeStreams.fromChildren(OneDoFJointBasics.class, rootBody).toArray(OneDoFJointBasics[]::new);

      SimulationSession session = new SimulationSession("Reachability Analysis - Example");
      session.initializeBufferSize(16000);
      Robot robot = session.addRobot(robotDefinition);
      ReachabilitySphereMapCalculator calculator = new ReachabilitySphereMapCalculator(armJoints, robot.getControllerOutput(), Voxel3DGrid.newVoxel3DGrid(25, 0.05, 50, 1));
      robot.addController(calculator);
      session.addYoGraphicDefinition(calculator.getYoGraphicVisuals());

      SessionVisualizerControls guiControls = SessionVisualizer.startSessionVisualizer(session);
      VisualDefinitionFactory visualDefinitionFactory = new VisualDefinitionFactory();
      visualDefinitionFactory.appendTranslation(RobotArmJointParameters.getRootJoint().getJointOffset());
      visualDefinitionFactory.addCoordinateSystem(1.0);
      guiControls.addStaticVisuals(visualDefinitionFactory.getVisualDefinitions());
      guiControls.addStaticVisuals(ReachabilityMapTools.createReachibilityColorScaleVisuals());
      guiControls.addStaticVisuals(ReachabilityMapTools.createBoundingBoxVisuals(calculator.getVoxel3DGrid()));
      calculator.setStaticVisualConsumer(guiControls::addStaticVisual);

      SimulationSessionControls simControls = session.getSimulationSessionControls();
      simControls.addExternalTerminalCondition(calculator::isDone);
      simControls.simulate(Integer.MAX_VALUE);
      guiControls.waitUntilDown();
   }

   public static void main(String[] args)
   {
      new ReachabilitySphereMapExample();
   }
}
