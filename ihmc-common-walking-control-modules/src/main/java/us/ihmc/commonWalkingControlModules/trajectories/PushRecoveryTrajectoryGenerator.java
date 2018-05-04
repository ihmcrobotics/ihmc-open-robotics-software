package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.math.trajectories.PositionTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.robotics.trajectories.providers.PositionProvider;
import us.ihmc.robotics.trajectories.providers.VectorProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

/**
 *
 *
 * This trajectory unless the robot is pushed, this class will behave exactly like the TwoWaypointTrajectoyrGenerator except this class has
 * the soft TouchdownTrajectoryGenerator included rather than the two being combined in an ArrayList of position trajectory generators. When
 * the robot is pushed, the XY portion of the trajectory are replanned so the robot can recover from the push.
 */
public class PushRecoveryTrajectoryGenerator implements PositionTrajectoryGenerator
{
   private static final boolean VISUALIZE = true;

   private final String namePostFix = getClass().getSimpleName();
   private final YoVariableRegistry registry;
   private final YoBoolean visualize;

   private final int numberOfBallsInBag = 30;
   private final BagOfBalls bagOfBalls;
   private double t0ForViz;
   private double tfForViz;
   private double tForViz;

   private final DoubleProvider swingTimeRemainingProvider;
   private final PositionProvider[] positionSources = new PositionProvider[2];
   private final VectorProvider[] velocitySources = new VectorProvider[2];

   private final YoDouble swingTime;
   private final YoDouble timeIntoStep;

   private final YoFramePoint3D desiredPosition;
   private final YoFrameVector3D desiredVelocity;
   private final YoFrameVector3D desiredAcceleration;

   private final YoPolynomial xPolynomial, yPolynomial;
   private final PositionTrajectoryGenerator nominalTrajectoryGenerator;
   private final DoubleProvider swingTimeProvider;

   private FramePoint3D nominalTrajectoryPosition = new FramePoint3D();
   private FrameVector3D nominalTrajectoryVelocity = new FrameVector3D();
   private FrameVector3D nominalTrajectoryAcceleration = new FrameVector3D();

   public PushRecoveryTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, DoubleProvider swingTimeProvider,
         DoubleProvider swingTimeRemainingProvider, PositionProvider initialPositionProvider, VectorProvider initialVelocityProvider,
         PositionProvider finalPositionProvider, YoVariableRegistry parentRegistry, YoGraphicsListRegistry yoGraphicsListRegistry,
         PositionTrajectoryGenerator nominalTrajectoryGenerator)
   {
      registry = new YoVariableRegistry(namePrefix + namePostFix);
      parentRegistry.addChild(registry);

      this.swingTimeRemainingProvider = swingTimeRemainingProvider;
      this.swingTimeProvider = swingTimeProvider;

      positionSources[0] = initialPositionProvider;
      positionSources[1] = finalPositionProvider;

      velocitySources[0] = initialVelocityProvider;

      xPolynomial = new YoPolynomial(namePrefix + "PolynomialX", 6, registry);
      yPolynomial = new YoPolynomial(namePrefix + "PolynomialY", 6, registry);

      swingTime = new YoDouble(namePrefix + "SwingTime", registry);
      swingTime.set(swingTimeProvider.getValue());

      timeIntoStep = new YoDouble(namePrefix + "TimeIntoStep", registry);

      desiredPosition = new YoFramePoint3D(namePrefix + "DesiredPosition", referenceFrame, registry);
      desiredVelocity = new YoFrameVector3D(namePrefix + "DesiredVelocity", referenceFrame, registry);
      desiredAcceleration = new YoFrameVector3D(namePrefix + "DesiredAcceleration", referenceFrame, registry);

      this.visualize = new YoBoolean(namePrefix + "Visualize", registry);
      this.visualize.set(VISUALIZE);

      this.nominalTrajectoryGenerator = nominalTrajectoryGenerator;

      this.bagOfBalls = new BagOfBalls(numberOfBallsInBag, 0.01, namePrefix + "SwingTrajectoryBagOfBalls", registry, yoGraphicsListRegistry);
   }

   private final FrameVector3D tempVector = new FrameVector3D();
   private final FramePoint3D tempPosition = new FramePoint3D();

   public void initialize()
   {
      swingTime.set(swingTimeProvider.getValue());
      timeIntoStep.set(swingTime.getDoubleValue() - swingTimeRemainingProvider.getValue());

      positionSources[0].getPosition(tempPosition);
      tempPosition.changeFrame(desiredPosition.getReferenceFrame());
      double x0 = tempPosition.getX();
      double y0 = tempPosition.getY();

      velocitySources[0].get(tempVector);
      tempVector.changeFrame(desiredPosition.getReferenceFrame());
      double xd0 = tempVector.getX();
      double yd0 = tempVector.getY();

      positionSources[1].getPosition(tempPosition);
      tempPosition.changeFrame(desiredPosition.getReferenceFrame());
      double xFinal = tempPosition.getX();
      double yFinal = tempPosition.getY();

      nominalTrajectoryGenerator.compute(timeIntoStep.getDoubleValue());
      nominalTrajectoryGenerator.getAcceleration(tempVector);
      tempVector.changeFrame(desiredPosition.getReferenceFrame());
      double xdd0 = tempVector.getX();
      double ydd0 = tempVector.getY();

      xPolynomial.setQuintic(timeIntoStep.getDoubleValue(), swingTime.getDoubleValue(), x0, xd0, xdd0, xFinal, 0.0, 0.0);
      yPolynomial.setQuintic(timeIntoStep.getDoubleValue(), swingTime.getDoubleValue(), y0, yd0, ydd0, yFinal, 0.0, 0.0);

      if (VISUALIZE)
      {
         visualizeTrajectory();
      }
   }

   public void compute(double time)
   {
      timeIntoStep.set(time);

      nominalTrajectoryGenerator.compute(time);

      nominalTrajectoryGenerator.getLinearData(nominalTrajectoryPosition, nominalTrajectoryVelocity, nominalTrajectoryAcceleration);

      xPolynomial.compute(time);
      yPolynomial.compute(time);

      desiredPosition.setX(xPolynomial.getPosition());
      desiredPosition.setY(yPolynomial.getPosition());
      desiredPosition.setZ(nominalTrajectoryPosition.getZ());

      desiredVelocity.setX(xPolynomial.getVelocity());
      desiredVelocity.setY(yPolynomial.getVelocity());
      desiredVelocity.setZ(nominalTrajectoryVelocity.getZ());

      desiredAcceleration.setX(xPolynomial.getAcceleration());
      desiredAcceleration.setY(yPolynomial.getAcceleration());
      desiredAcceleration.setZ(nominalTrajectoryAcceleration.getZ());
   }

   private void visualizeTrajectory()
   {
      t0ForViz = timeIntoStep.getDoubleValue();
      tfForViz = swingTime.getDoubleValue();

      for (int i = 0; i < numberOfBallsInBag; i++)
      {
         tForViz = t0ForViz + (double) i / (double) (numberOfBallsInBag) * (tfForViz - t0ForViz);
         computePositionsForVis(tForViz);
         bagOfBalls.setBall(desiredPosition, i);
      }
   }

   public void computePositionsForVis(double time)
   {
      nominalTrajectoryGenerator.compute(time);

      xPolynomial.compute(time);
      yPolynomial.compute(time);

      nominalTrajectoryGenerator.getPosition(nominalTrajectoryPosition);
      nominalTrajectoryGenerator.getVelocity(nominalTrajectoryVelocity);
      nominalTrajectoryGenerator.getAcceleration(nominalTrajectoryAcceleration);

      desiredPosition.setX(xPolynomial.getPosition());
      desiredPosition.setY(yPolynomial.getPosition());
      desiredPosition.setZ(nominalTrajectoryPosition.getZ());
   }

   public void getPosition(FramePoint3D positionToPack)
   {
      positionToPack.setIncludingFrame(desiredPosition);
   }

   public void getVelocity(FrameVector3D velocityToPack)
   {
      velocityToPack.setIncludingFrame(desiredVelocity);
   }

   public void getAcceleration(FrameVector3D accelerationToPack)
   {
      accelerationToPack.setIncludingFrame(desiredAcceleration);
   }

   @Override
   public boolean isDone()
   {
      return timeIntoStep.getDoubleValue() >= swingTime.getDoubleValue();
   }

   public void getLinearData(FramePoint3D positionToPack, FrameVector3D velocityToPack, FrameVector3D accelerationToPack)
   {
      getPosition(positionToPack);
      getVelocity(velocityToPack);
      getAcceleration(accelerationToPack);
   }

   @Override
   public void showVisualization()
   {
   }

   @Override
   public void hideVisualization()
   {
   }
}
