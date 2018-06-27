package us.ihmc.exampleSimulations.genericQuadruped.controller.force;

import gnu.trove.list.array.TDoubleArrayList;
import gnu.trove.list.array.TIntArrayList;
import org.ejml.alg.dense.decomposition.chol.CholeskyDecompositionCommon_D64;
import org.ejml.alg.dense.decomposition.lu.LUDecompositionBase_D64;
import org.ejml.data.DenseMatrix64F;
import org.junit.After;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCoreMode;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.allocations.AllocationProfiler;
import us.ihmc.commons.allocations.AllocationRecord;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.exampleSimulations.genericQuadruped.GenericQuadrupedTestFactory;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.jMonkeyEngineToolkit.jme.JMEGraphicsObject;
import us.ihmc.quadrupedRobotics.QuadrupedTestBehaviors;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestGoals;
import us.ihmc.quadrupedRobotics.QuadrupedTestYoVariables;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerManager;
import us.ihmc.quadrupedRobotics.input.managers.QuadrupedTeleopManager;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.simulationconstructionset.dataBuffer.MirroredYoVariableRegistry;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.MemoryTools;

import java.io.IOException;
import java.util.List;

public class GenericQuadrupedAllocationTest
{
   private QuadrupedTestFactory quadrupedTestFactory;
   private GoalOrientedTestConductor conductor;
   private QuadrupedTestYoVariables variables;
   private QuadrupedTeleopManager stepTeleopManager;
   private AllocationProfiler allocationProfiler = new AllocationProfiler();

   @Before
   public void before() throws SimulationExceededMaximumTimeException
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
      
      AllocationProfiler.checkInstrumentation();

      allocationProfiler.includeAllocationsInsideClass(QuadrupedControllerManager.class.getName());
      // These are places specific to the simulation and will not show up on the real robot.
      allocationProfiler.excludeAllocationsInsideClass(MirroredYoVariableRegistry.class.getName());
      allocationProfiler.excludeAllocationsInsideClass(MeshDataGenerator.class.getName());
      allocationProfiler.excludeAllocationsInsideClass(JMEGraphicsObject.class.getName());

      // TODO: fix these!
      allocationProfiler.excludeAllocationsInsideClass(StatusMessageOutputManager.class.getName());
      // These methods are "safe" as they will only allocate to increase their capacity.
      allocationProfiler.excludeAllocationsInsideMethod(DenseMatrix64F.class.getName() + ".reshape");
      allocationProfiler.excludeAllocationsInsideMethod(TIntArrayList.class.getName() + ".ensureCapacity");
      allocationProfiler.excludeAllocationsInsideMethod(ConvexPolygon2D.class.getName() + ".setOrCreate");
      allocationProfiler.excludeAllocationsInsideMethod(FrameConvexPolygon2D.class.getName() + ".setOrCreate");
      allocationProfiler.excludeAllocationsInsideMethod(RecyclingArrayList.class.getName() + ".ensureCapacity");
      allocationProfiler.excludeAllocationsInsideMethod(LUDecompositionBase_D64.class.getName() + ".decomposeCommonInit");
      allocationProfiler.excludeAllocationsInsideMethod(CholeskyDecompositionCommon_D64.class.getName() + ".decompose");
      allocationProfiler.excludeAllocationsInsideMethod(TDoubleArrayList.class.getName() + ".ensureCapacity");

      // Ignore the following methods as they are related to printouts.
      allocationProfiler.excludeAllocationsInsideMethod(Throwable.class.getName() + ".printStackTrace");
      allocationProfiler.excludeAllocationsInsideMethod(PrintTools.class.getName() + ".print");
      
      setup();
   }

   @After
   public void tearDown()
   {
      quadrupedTestFactory.close();
      conductor.concludeTesting();
      conductor = null;
      variables = null;
      stepTeleopManager = null;

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   public void setup()
   {
      try
      {
         quadrupedTestFactory = new GenericQuadrupedTestFactory();
         quadrupedTestFactory.setControlMode(WholeBodyControllerCoreMode.VIRTUAL_MODEL);
         quadrupedTestFactory.setGroundContactModelType(QuadrupedGroundContactModelType.FLAT);
         quadrupedTestFactory.setUseNetworking(true);
         conductor = quadrupedTestFactory.createTestConductor();
         variables = new QuadrupedTestYoVariables(conductor.getScs());
         stepTeleopManager = quadrupedTestFactory.getStepTeleopManager();
      }
      catch (IOException e)
      {
         throw new RuntimeException("Error loading simulation: " + e.getMessage());
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 300.0, categoriesOverride = {IntegrationCategory.SLOW})
   @Test(timeout = 600000)
   public void testForAllocationStanding()
   {
      QuadrupedTestBehaviors.standUp(conductor, variables);
      QuadrupedTestBehaviors.startBalancing(conductor, variables, stepTeleopManager);

      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(QuadrupedTestGoals.timeInFuture(variables, 0.25));

      testInternal(() -> conductor.simulate());
   }


   @ContinuousIntegrationTest(estimatedDuration = 300.0, categoriesOverride = {IntegrationCategory.SLOW})
   @Test(timeout = 600000)
   public void testForAllocationStepping()
   {
      QuadrupedTestBehaviors.readyXGait(conductor, variables, stepTeleopManager);

      stepTeleopManager.getXGaitSettings().setEndPhaseShift(180);

      double walkTime = 5.0;
      double walkSpeed = 0.25;
      stepTeleopManager.requestXGait();
      stepTeleopManager.setDesiredVelocity(walkSpeed, 0.0, 0.0);
      conductor.addSustainGoal(QuadrupedTestGoals.notFallen(variables));
      conductor.addTerminalGoal(YoVariableTestGoal.timeInFuture(variables.getYoTime(), walkTime));
      testInternal(() -> conductor.simulate());
   }

   private void testInternal(Runnable whatToTestFor)
   {
      List<AllocationRecord> allocations = allocationProfiler.recordAllocations(whatToTestFor);

      if (!allocations.isEmpty())
      {
         allocations.forEach(allocation -> System.out.println(allocation));
         Assert.fail("Found allocations in the controller.");
      }
   }
}
