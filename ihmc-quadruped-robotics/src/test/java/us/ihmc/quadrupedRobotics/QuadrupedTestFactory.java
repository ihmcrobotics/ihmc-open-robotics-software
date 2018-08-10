package us.ihmc.quadrupedRobotics;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.input.managers.QuadrupedTeleopManager;
import us.ihmc.quadrupedRobotics.model.QuadrupedInitialOffsetAndYaw;
import us.ihmc.quadrupedRobotics.model.QuadrupedInitialPositionParameters;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

import java.io.IOException;

public interface QuadrupedTestFactory
{
   public GoalOrientedTestConductor createTestConductor() throws IOException;

   public QuadrupedTeleopManager getStepTeleopManager();

   public void setControlMode(WholeBodyControllerCoreMode controlMode);

   public void setGroundContactModelType(QuadrupedGroundContactModelType groundContactModelType);
   
   public void setUseStateEstimator(boolean useStateEstimator);
   
   public void setGroundProfile3D(GroundProfile3D groundProfile3D);

   public void setTerrainObject3D(TerrainObject3D terrainObject3D);

   public void setUsePushRobotController(boolean usePushRobotController);

   public void setInitialPosition(QuadrupedInitialPositionParameters initialPosition);

   public void setUseNetworking(boolean useNetworking);

   void setScsParameters(SimulationConstructionSetParameters scsParameters);

   void setInitialOffset(QuadrupedInitialOffsetAndYaw initialOffset);

   String getRobotName();

   FullRobotModel getFullRobotModel();

   void close();
}
