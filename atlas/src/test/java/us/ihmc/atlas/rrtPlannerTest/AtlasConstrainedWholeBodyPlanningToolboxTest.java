package us.ihmc.atlas.rrtPlannerTest;

import java.io.IOException;

import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.behaviorTests.rrtPlannerTests.ConstrainedWholeBodyPlanningToolboxTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasConstrainedWholeBodyPlanningToolboxTest extends ConstrainedWholeBodyPlanningToolboxTest
{
   private final DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return robotModel.getSimpleRobotName();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 33000)
   public void testForConfigurationSpace() throws SimulationExceededMaximumTimeException, IOException
   {
      super.testForConfigurationSpace();
   }
      
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 60.0)
   @Test(timeout = 80000)
   public void testForInverseKinematicsToolbox() throws SimulationExceededMaximumTimeException, IOException
   {
      super.testForInverseKinematicsToolbox();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 30.0)
   @Test(timeout = 35000)
   public void testForTrajSolver() throws SimulationExceededMaximumTimeException, IOException
   {
      super.testForTrajSolver();
   }
         
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 25000)
   public void testForConstrainedTrajectory() throws SimulationExceededMaximumTimeException, IOException
   {
      super.testForConstrainedTrajectory();
   }
   
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 20.0)
   @Test(timeout = 23000)
   public void testForSolver() throws SimulationExceededMaximumTimeException, IOException
   {
      super.testForSolver();
   }
}