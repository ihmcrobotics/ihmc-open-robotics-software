package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.robotController.RobotController;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.BagOfBalls;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.trajectories.CartesianTrajectoryGenerator;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class CartesianTrajectoryGeneratorTesterNew
{
   private static final class TrajectoryEvaluatorController implements RobotController
   {
      private final double dt;
      private final String name = "test";
      private final YoVariableRegistry registry = new YoVariableRegistry(name);
      private final ReferenceFrame referenceFrame = ReferenceFrame.getWorldFrame();
      private final CartesianTrajectoryGenerator trajectoryGenerator;

      //      private final FramePoint initialPosition = new FramePoint(referenceFrame, 0.0, 0.0, 0.0);
      //      private final FrameVector initialVelocity = new FrameVector(referenceFrame, 0.1, 0.2, 0.3);
      //      private final FrameVector initialAcceleration = new FrameVector(referenceFrame, -0.1, -0.2, -0.3);
      //      private final FramePoint finalDesiredPosition = new FramePoint(referenceFrame, 0.3, 0.1, 0.25);
      //      private final FrameVector finalDesiredVelocity = new FrameVector(referenceFrame, -0.1, 0.1, -0.1);

//            private final FramePoint initialPosition = new FramePoint(referenceFrame, 0.0, 0.0, 0.0);
//            private final FrameVector initialVelocity = new FrameVector(referenceFrame, 0.1, 0.0, 0.0);
//            private final FrameVector initialAcceleration = new FrameVector(referenceFrame, -0.1, -0.2, -0.3);
//            private final FramePoint finalDesiredPosition = new FramePoint(referenceFrame, 0.3, 0.1, 0.25);
//            private final FrameVector finalDesiredVelocity = new FrameVector(referenceFrame, -0.1, 0.1, -0.1);

      //      private final FramePoint initialPosition = new FramePoint(referenceFrame, 0.0, 0.0, 0.0);
      //      private final FrameVector initialVelocity = new FrameVector(referenceFrame, 0.0, 0.0, 0.0);
      //      private final FrameVector initialAcceleration = new FrameVector(referenceFrame, -0.1, -0.2, -0.3);
      //      private final FramePoint finalDesiredPosition = new FramePoint(referenceFrame, 0.3, 0.1, 0.25);
      //      private final FrameVector finalDesiredVelocity = new FrameVector(referenceFrame, 0.0, 0.0, 0.0);

      private final FramePoint initialPosition = new FramePoint(referenceFrame, -0.05666757380023707, 0.10052877782151563, 0.3309106951292634);
      private final FrameVector initialVelocity = new FrameVector(referenceFrame, 0.002249342893913859, -0.0033555201145383877, -0.0029238257850914256);
      private final FrameVector initialAcceleration = new FrameVector(referenceFrame, 0.0, 0.0, 0.0);
      private final FramePoint finalDesiredPosition = new FramePoint(referenceFrame, 0.2090459436488653, 0.09971527634692637, 1.086355304703153);
      private final FrameVector finalDesiredVelocity = new FrameVector(referenceFrame, 0.0, 0.0, 0.0);

      private final FramePoint positionToPack = new FramePoint(referenceFrame);
      private final FrameVector velocityToPack = new FrameVector(referenceFrame);
      private final FrameVector accelerationToPack = new FrameVector(referenceFrame);

      private final YoFramePoint position = new YoFramePoint("position", "", referenceFrame, registry);
      private final YoFrameVector velocity = new YoFrameVector("velocity", "", referenceFrame, registry);
      private final YoFrameVector acceleration = new YoFrameVector("acceleration", "", referenceFrame, registry);

      private final BagOfBalls bagOfBalls;

      private TrajectoryEvaluatorController(double straightUpTime, double stepTime, double groundClearance, double dt,
            YoGraphicsListRegistry yoGraphicsListRegistry)
      {
         this.dt = dt;
         trajectoryGenerator = new StraightUpThenParabolicCartesianTrajectoryGenerator("test", referenceFrame, straightUpTime, stepTime, groundClearance,
               registry);
         trajectoryGenerator.initialize(initialPosition, initialVelocity, null, finalDesiredPosition, null);

//         trajectoryGenerator = new FifthOrderWaypointCartesianTrajectoryGenerator("test", referenceFrame, new ConstantDoubleProvider(stepTime), groundClearance, registry);
         trajectoryGenerator.initialize(initialPosition, initialVelocity, initialAcceleration, finalDesiredPosition, finalDesiredVelocity);

         bagOfBalls = new BagOfBalls((int) (getTrajectoryTime() / dt), registry, yoGraphicsListRegistry);
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

   public static void main(String[] args)
   {
      final double dt = 1e-3;
      final YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

      double straightUpAverageVelocity = 0.2;
      double parabolicTime = 1.5;
      double groundClearance = 0.1;
      TrajectoryEvaluatorController robotController = new TrajectoryEvaluatorController(straightUpAverageVelocity, parabolicTime, groundClearance, dt,
            yoGraphicsListRegistry);

      robotController.initialize();

      Robot robot = new Robot("robot");
      robot.setController(robotController);

      SimulationConstructionSet scs = new SimulationConstructionSet(robot);
      scs.setDT(dt, 1);

      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      Thread thread = new Thread(scs);
      thread.start();
      scs.simulate(robotController.getTrajectoryTime());
   }
}
