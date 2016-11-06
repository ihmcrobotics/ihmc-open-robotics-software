package us.ihmc.quadrupedRobotics.stateEstimator;

import java.util.List;

import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.quadrupedRobotics.estimator.stateEstimator.QuadrupedFootSwitchFactory;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.sensors.FootSwitchInterface;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.simulatedSensors.GroundContactPointBasedWrenchCalculator;
import us.ihmc.simulationconstructionset.simulatedSensors.WrenchCalculatorInterface;
import us.ihmc.tools.factories.OptionalFactoryField;
import us.ihmc.tools.io.printing.PrintTools;

public class SimulatedQuadrupedFootSwitchFactory extends QuadrupedFootSwitchFactory
{
   
   // Used to create the ground contact point based foot switches.
   private final OptionalFactoryField<FloatingRootJointRobot> simulatedRobot = new OptionalFactoryField<>("simulatedRobot");

   
   @Override
   protected void setupGroundContactPointFootSwitches(QuadrantDependentList<FootSwitchInterface> footSwitches, double totalRobotWeight)
   {
      if (!simulatedRobot.hasBeenSet())
      {
         PrintTools.warn(this, "simulatedRobot is not set, creating touchdown based foot switches.");
         setupTouchdownBasedFootSwitches(footSwitches, totalRobotWeight);
         return;
      }

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         ContactablePlaneBody contactablePlaneBody = footContactableBodies.get().get(robotQuadrant);
         InverseDynamicsJoint parentJoint = contactablePlaneBody.getRigidBody().getParentJoint();
         String jointName = parentJoint.getName();
         String forceSensorName = contactablePlaneBody.getName() + "ForceSensor";
         OneDegreeOfFreedomJoint forceTorqueSensorJoint = simulatedRobot.get().getOneDegreeOfFreedomJoint(jointName);
         List<GroundContactPoint> contactPoints = forceTorqueSensorJoint.getGroundContactPointGroup().getGroundContactPoints();
         RigidBodyTransform transformToParentJoint = contactablePlaneBody.getSoleFrame().getTransformToDesiredFrame(parentJoint.getFrameAfterJoint());
         WrenchCalculatorInterface wrenchCaluclator = new GroundContactPointBasedWrenchCalculator(forceSensorName, contactPoints, forceTorqueSensorJoint, transformToParentJoint);
         FootSwitchInterface footSwitch = new QuadrupedDebugFootSwitch(wrenchCaluclator, contactablePlaneBody, totalRobotWeight, registry);
         footSwitches.set(robotQuadrant, footSwitch);
      }
   }
   
   public void setSimulatedRobot(FloatingRootJointRobot simulatedRobot)
   {
      this.simulatedRobot.set(simulatedRobot);
   }

}
