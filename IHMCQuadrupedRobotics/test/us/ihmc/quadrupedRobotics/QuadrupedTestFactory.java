package us.ihmc.quadrupedRobotics;

import java.io.IOException;

import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedParameterSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.GoalOrientedTestConductor;

public interface QuadrupedTestFactory
{
   public GoalOrientedTestConductor createTestConductor() throws IOException;

   public void setControlMode(QuadrupedControlMode controlMode);

   public void setGroundContactModelType(QuadrupedGroundContactModelType groundContactModelType);
   
   public void setUseStateEstimator(boolean useStateEstimator);
   
   public void setGroundProfile3D(GroundProfile3D groundProfile3D);

   public void setUsePushRobotController(boolean usePushRobotController);
   
   public void setParameterSet(QuadrupedParameterSet parameterSet);
}
