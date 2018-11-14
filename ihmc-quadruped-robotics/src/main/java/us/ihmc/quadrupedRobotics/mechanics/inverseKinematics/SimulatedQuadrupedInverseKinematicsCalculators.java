package us.ihmc.quadrupedRobotics.mechanics.inverseKinematics;

import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.model.QuadrupedModelFactory;
import us.ihmc.quadrupedRobotics.model.QuadrupedPhysicalProperties;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class SimulatedQuadrupedInverseKinematicsCalculators extends QuadrupedInverseKinematicsCalculators
{

   public SimulatedQuadrupedInverseKinematicsCalculators(QuadrupedModelFactory modelFactory, JointDesiredOutputList jointDesiredOutputList,
                                                         QuadrupedPhysicalProperties physicalProperties, FullQuadrupedRobotModel fullRobotModel, QuadrupedReferenceFrames referenceFrames,
                                                         YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      super(modelFactory, jointDesiredOutputList, physicalProperties, fullRobotModel, referenceFrames, parentRegistry, yoGraphicsListRegistry);
   }

   public void updateSimulationBasedOnFullRobotModel(FloatingRootJointRobot sdfRobot)
   {
      for (OneDoFJointBasics joint : oneDoFJoints)
      {
         OneDegreeOfFreedomJoint simulatedJoint = (OneDegreeOfFreedomJoint) sdfRobot.getJoint(joint.getName());
         simulatedJoint.setQ(jointDesiredOutputList.getJointDesiredOutput(joint).getDesiredPosition());
      }
   }
}
