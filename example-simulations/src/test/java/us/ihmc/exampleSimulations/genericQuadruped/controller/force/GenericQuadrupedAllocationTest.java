package us.ihmc.exampleSimulations.genericQuadruped.controller.force;

import gnu.trove.list.array.TIntArrayList;
import org.ejml.alg.dense.decomposition.chol.CholeskyDecompositionCommon_D64;
import org.ejml.alg.dense.decomposition.lu.LUDecompositionBase_D64;
import org.ejml.data.DenseMatrix64F;
import org.junit.After;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.exampleSimulations.genericQuadruped.GenericQuadrupedTestFactory;
import us.ihmc.graphicsDescription.MeshDataGenerator;
import us.ihmc.jMonkeyEngineToolkit.jme.JMEGraphicsObject;
import us.ihmc.quadrupedRobotics.QuadrupedForceTestYoVariables;
import us.ihmc.quadrupedRobotics.QuadrupedTestBehaviors;
import us.ihmc.quadrupedRobotics.QuadrupedTestFactory;
import us.ihmc.quadrupedRobotics.QuadrupedTestGoals;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControlMode;
import us.ihmc.quadrupedRobotics.controller.QuadrupedControllerManager;
import us.ihmc.quadrupedRobotics.input.managers.QuadrupedTeleopManager;
import us.ihmc.quadrupedRobotics.simulation.QuadrupedGroundContactModelType;
import us.ihmc.robotics.allocations.AllocationTest;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.robotics.testing.YoVariableTestGoal;
import us.ihmc.simulationConstructionSetTools.util.simulationrunner.GoalOrientedTestConductor;
import us.ihmc.simulationconstructionset.dataBuffer.MirroredYoVariableRegistry;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.MemoryTools;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class GenericQuadrupedAllocationTest implements AllocationTest
{
   private GoalOrientedTestConductor conductor;
   private QuadrupedForceTestYoVariables variables;
   private QuadrupedTeleopManager stepTeleopManager;

   @Before
   public void before() throws SimulationExceededMaximumTimeException
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
      AllocationTest.checkInstrumentation();
      setup();
   }

   @After
   public void tearDown()
   {
      conductor.concludeTesting();
      conductor = null;
      variables = null;

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   public void setup()
   {
      try
      {
         QuadrupedTestFactory quadrupedTestFactory = new GenericQuadrupedTestFactory(false);
         quadrupedTestFactory.setControlMode(QuadrupedControlMode.FORCE);
         quadrupedTestFactory.setGroundContactModelType(QuadrupedGroundContactModelType.FLAT);
         quadrupedTestFactory.setUseNetworking(true);
         conductor = quadrupedTestFactory.createTestConductor();
         variables = new QuadrupedForceTestYoVariables(conductor.getScs());
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
      List<Throwable> allocations = runAndCollectAllocations(whatToTestFor);

      if (!allocations.isEmpty())
      {
         allocations.forEach(allocation -> allocation.printStackTrace());
         Assert.fail("Found allocations in the controller.");
      }
   }

   @Override
   public List<Class<?>> getClassesOfInterest()
   {
      List<Class<?>> classesOfInterest = new ArrayList<>();
      classesOfInterest.add(QuadrupedControllerManager.class);
      return classesOfInterest;
   }

   @Override
   public List<Class<?>> getClassesToIgnore()
   {
      List<Class<?>> classesToIgnore = new ArrayList<>();

      // These are places specific to the simulation and will not show up on the real robot.
      classesToIgnore.add(MirroredYoVariableRegistry.class);
      classesToIgnore.add(MeshDataGenerator.class);
      classesToIgnore.add(JMEGraphicsObject.class);

      // TODO: fix these!
      classesToIgnore.add(StatusMessageOutputManager.class);

      return classesToIgnore;
   }

   @Override
   public List<String> getMethodsToIgnore()
   {
      List<String> methodsToIgnore = new ArrayList<>();

      // These methods are "safe" as they will only allocate to increase their capacity.
      methodsToIgnore.add(DenseMatrix64F.class.getName() + ".reshape");
      methodsToIgnore.add(TIntArrayList.class.getName() + ".ensureCapacity");
      methodsToIgnore.add(ConvexPolygon2D.class.getName() + ".setOrCreate");
      methodsToIgnore.add(FrameConvexPolygon2D.class.getName() + ".setOrCreate");
      methodsToIgnore.add(RecyclingArrayList.class.getName() + ".ensureCapacity");
      methodsToIgnore.add(LUDecompositionBase_D64.class.getName() + ".decomposeCommonInit");
      methodsToIgnore.add(CholeskyDecompositionCommon_D64.class.getName() + ".decompose");

      // Ignore the following methods as they are related to printouts.
      methodsToIgnore.add(Throwable.class.getName() + ".printStackTrace");
      methodsToIgnore.add(PrintTools.class.getName() + ".print");

      return methodsToIgnore;
   }
}
