package us.ihmc.quadrupedRobotics.mechanics.inverseKinematics;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.model.QuadrupedModelFactory;
import us.ihmc.quadrupedRobotics.model.QuadrupedPhysicalProperties;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;

public class SimulatedQuadrupedInverseKinematicsCalculators extends QuadrupedInverseKinematicsCalculators
{

   public SimulatedQuadrupedInverseKinematicsCalculators(QuadrupedModelFactory modelFactory, QuadrupedPhysicalProperties physicalProperties, FullQuadrupedRobotModel fullRobotModel, QuadrupedReferenceFrames referenceFrames,
         YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      super(modelFactory, physicalProperties, fullRobotModel, referenceFrames, parentRegistry, yoGraphicsListRegistry);
   }

   public void updateSimulationBasedOnFullRobotModel(FloatingRootJointRobot sdfRobot)
   {
      for (OneDoFJoint joint : oneDoFJoints)
      {
         OneDegreeOfFreedomJoint simulatedJoint = (OneDegreeOfFreedomJoint) sdfRobot.getJoint(joint.getName());
         simulatedJoint.setQ(joint.getqDesired());
      }
   }
}
