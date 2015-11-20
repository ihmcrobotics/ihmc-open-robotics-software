package us.ihmc.quadrupedRobotics.stateEstimator;

import java.util.ArrayList;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedJointNameMap;
import us.ihmc.quadrupedRobotics.parameters.QuadrupedRobotParameters;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.sensors.ContactBasedFootSwitch;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.simulatedSensors.SimulatedContactBasedFootSwitch;

public class SimpleSimulationQuadrupedStateEstimator implements QuadrupedStateEstimator
{
   private final QuadrantDependentList<ContactBasedFootSwitch> footSwitches = new QuadrantDependentList<>();

   public SimpleSimulationQuadrupedStateEstimator(SDFRobot simulationRobot, QuadrupedRobotParameters robotParameters, YoVariableRegistry registry)
   {
      ArrayList<ExternalForcePoint> allExternalForcePoints = simulationRobot.getAllExternalForcePoints();
      QuadrupedJointNameMap jointMap = robotParameters.getJointMap();
      for(RobotQuadrant quadrant : RobotQuadrant.values)
      {
         String prefix = quadrant.getCamelCaseNameForStartOfExpression();
         String jointBeforeFootName = jointMap.getJointBeforeFootName(quadrant);
         Joint jointBeforeFoot = simulationRobot.getJoint(jointBeforeFootName);
         
         for(ExternalForcePoint forcePoint : allExternalForcePoints)
         {
            if(forcePoint.getParentJoint() == jointBeforeFoot)
            {
               footSwitches.put(quadrant, new SimulatedContactBasedFootSwitch(prefix, forcePoint, registry));
            }
         }
      }
   }

   @Override
   public boolean isFootInContact(RobotQuadrant quadrant)
   {
      return footSwitches.get(quadrant).isInContact();
   }

}
