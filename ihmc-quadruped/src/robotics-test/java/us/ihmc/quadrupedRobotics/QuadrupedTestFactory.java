package us.ihmc.quadrupedRobotics;

import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.quadrupedCommunication.teleop.RemoteQuadrupedTeleopManager;
import us.ihmc.quadrupedRobotics.model.QuadrupedInitialOffsetAndYaw;
import us.ihmc.quadrupedRobotics.model.QuadrupedInitialPositionParameters;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.robotModels.FullQuadrupedRobotModel;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

import java.io.IOException;

public interface QuadrupedTestFactory
{
   public GoalOrientedTestConductor createTestConductor() throws IOException;

   public RemoteQuadrupedTeleopManager getRemoteStepTeleopManager();

   public void setGroundContactModelType(QuadrupedGroundContactModelType groundContactModelType);

   public void setUseStateEstimator(boolean useStateEstimator);

   public void setGroundProfile3D(GroundProfile3D groundProfile3D);

   public void setTerrainObject3D(TerrainObject3D terrainObject3D);

   public void setUsePushRobotController(boolean usePushRobotController);

   public void setInitialPosition(QuadrupedInitialPositionParameters initialPosition);

   void setScsParameters(SimulationConstructionSetParameters scsParameters);

   void setInitialOffset(QuadrupedInitialOffsetAndYaw initialOffset);

   String getRobotName();

   FullQuadrupedRobotModel getFullRobotModel();

   void close();
}
