package us.ihmc.commonWalkingControlModules.trajectories;

import static org.junit.Assert.fail;

import java.util.Random;

import org.junit.Test;

import us.ihmc.utilities.RandomTools;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.Robot;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.robotController.RobotController;
import com.yobotics.simulationconstructionset.util.graphics.BagOfBalls;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;

public class StraightUpThenParabolicCartesianTrajectoryGeneratorTest
{
   @Test
   public void testMaxHeight()
   {
      Random random = new Random(176L);

      ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();

      FramePoint positionToPack = new FramePoint(referenceFrame);
      FrameVector velocityToPack = new FrameVector(referenceFrame);
      FrameVector accelerationToPack = new FrameVector(referenceFrame);

      int n = 1000;
      double epsilon = 1e-3;
      for (int i = 0; i < n; i++)
      {
         double straightUpVelocity = random.nextDouble();
         double parabolicTime = straightUpVelocity + random.nextDouble();
         double groundClearance = random.nextDouble();
         YoVariableRegistry registry = new YoVariableRegistry("test");
         StraightUpThenParabolicCartesianTrajectoryGenerator trajectoryGenerator = new StraightUpThenParabolicCartesianTrajectoryGenerator("test",
                                                                                      referenceFrame, straightUpVelocity, parabolicTime, groundClearance, registry);

         FramePoint initialPosition = new FramePoint(referenceFrame, RandomTools.getRandomVector(random));
         FrameVector initialVelocity = new FrameVector(referenceFrame, RandomTools.getRandomVector(random));
         FramePoint finalDesiredPosition = new FramePoint(initialPosition);
         finalDesiredPosition.add(new FrameVector(referenceFrame, random.nextDouble(), random.nextDouble(), random.nextDouble()));

         trajectoryGenerator.initialize(initialPosition, initialVelocity, finalDesiredPosition);

         double zMax = finalDesiredPosition.getZ() + groundClearance;
         double minZDifference = Double.POSITIVE_INFINITY;

         double dt = parabolicTime / 1000.0;
         while (!trajectoryGenerator.isDone())
         {
            trajectoryGenerator.computeNextTick(positionToPack, velocityToPack, accelerationToPack, dt);
            double z = positionToPack.getZ();
            if (z > zMax + epsilon)
               fail("z = " + z + ", zMax = " + zMax);

            double zDifference = Math.abs(z - zMax);
            if (zDifference < minZDifference)
               minZDifference = zDifference;
         }

         if (minZDifference > epsilon)
            fail("minZDifference = " + minZDifference);
      }
   }

   public static void main(String[] args)
   {
      final double dt = 1e-3;
      final DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry = new DynamicGraphicObjectsListRegistry();

      double straightUpAverageVelocity = 0.2;
      double parabolicTime = 1.5;
      double groundClearance = 0.2;
      TrajectoryEvaluatorController robotController = new TrajectoryEvaluatorController(straightUpAverageVelocity, parabolicTime, groundClearance, dt, dynamicGraphicObjectsListRegistry);

      robotController.initialize();

      Robot robot = new Robot("robot");
      robot.setController(robotController);

      SimulationConstructionSet scs = new SimulationConstructionSet(robot);
      scs.setDT(dt, 1);

      dynamicGraphicObjectsListRegistry.addDynamicGraphicsObjectListsToSimulationConstructionSet(scs);
      Thread thread = new Thread(scs);
      thread.start();
      scs.simulate(robotController.getTrajectoryTime());
   }

   private static final class TrajectoryEvaluatorController implements RobotController
   {
      private final double dt;
      private static final long serialVersionUID = 1L;
      private final String name = "test";
      private final YoVariableRegistry registry = new YoVariableRegistry(name);
      private final ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();
      private final StraightUpThenParabolicCartesianTrajectoryGenerator trajectoryGenerator;
      private final FramePoint initialPosition = new FramePoint(referenceFrame, 0.0, 0.0, 0.0);
      private final FrameVector initialVelocity = new FrameVector(referenceFrame);
      private final FramePoint finalDesiredPosition = new FramePoint(referenceFrame, 0.3, 0.1, 0.25);
      private final FramePoint positionToPack = new FramePoint(referenceFrame);
      private final FrameVector velocityToPack = new FrameVector(referenceFrame);
      private final FrameVector accelerationToPack = new FrameVector(referenceFrame);

      private final YoFramePoint position = new YoFramePoint("position", "", referenceFrame, registry);
      private final YoFrameVector velocity = new YoFrameVector("velocity", "", referenceFrame, registry);
      private final YoFrameVector acceleration = new YoFrameVector("acceleration", "", referenceFrame, registry);

      private final BagOfBalls bagOfBalls;

      private TrajectoryEvaluatorController(double straightUpTime, double stepTime, double groundClearance, double dt,
              DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry)
      {
         this.dt = dt;
         trajectoryGenerator = new StraightUpThenParabolicCartesianTrajectoryGenerator("test", referenceFrame, straightUpTime, stepTime, groundClearance,
                 registry);
         trajectoryGenerator.initialize(initialPosition, initialVelocity, finalDesiredPosition);
         bagOfBalls = new BagOfBalls((int) (getTrajectoryTime() / dt), registry, dynamicGraphicObjectsListRegistry);
      }

      public double getTrajectoryTime()
      {
         return trajectoryGenerator.getFinalTime();
      }

      public void initialize()
      {
      }

      public YoVariableRegistry getYoVariableRegistry()
      {
         return registry;
      }

      public String getName()
      {
         return name;
      }

      public String getDescription()
      {
         return getName();
      }

      public void doControl()
      {
         trajectoryGenerator.computeNextTick(positionToPack, velocityToPack, accelerationToPack, dt);
         position.set(positionToPack);
         velocity.set(velocityToPack);
         acceleration.set(accelerationToPack);
         bagOfBalls.setBallLoop(positionToPack);
      }
   }
}
