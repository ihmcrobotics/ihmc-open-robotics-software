package us.ihmc.robotics.math.trajectories;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.CylindricalCoordinatesCalculator;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;
import us.ihmc.robotics.trajectories.providers.PositionProvider;


public class CirclePositionTrajectoryGenerator implements PositionTrajectoryGenerator
{
   private final YoVariableRegistry registry;
   private final DoubleYoVariable currentTime;
   private final DoubleYoVariable trajectoryTime;
   private final YoPolynomial anglePolynomial;

   private final DoubleYoVariable radius;
   private final DoubleYoVariable z;

   private final PositionProvider initialPositionProvider;
   private final DoubleProvider trajectoryTimeProvider;

   private final FramePoint tempFramePoint;
   private final DoubleProvider desiredRotationAngleProvider;


   private final ReferenceFrame referenceFrame;
   private final FramePoint position;
   private final FrameVector velocity;
   private final FrameVector acceleration;

   public CirclePositionTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, DoubleProvider trajectoryTimeProvider,
           PositionProvider initialPositionProvider, YoVariableRegistry parentRegistry, DoubleProvider desiredRotationAngleProvider)
   {
      // calculate the initial angle:

      this.registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      this.trajectoryTime = new DoubleYoVariable(namePrefix + "TrajectoryTime", registry);
      this.currentTime = new DoubleYoVariable(namePrefix + "Time", registry);
      this.anglePolynomial = new YoPolynomial(namePrefix + "ParameterPolynomial", 6, registry);

      this.referenceFrame = referenceFrame;

      this.desiredRotationAngleProvider = desiredRotationAngleProvider;
      this.trajectoryTimeProvider = trajectoryTimeProvider;

      this.initialPositionProvider = initialPositionProvider;
      this.radius = new DoubleYoVariable(namePrefix + "Radius", registry);
      this.z = new DoubleYoVariable(namePrefix + "ZPosition", registry);
      position = new FramePoint(referenceFrame);
      velocity = new FrameVector(referenceFrame);
      acceleration = new FrameVector(referenceFrame);

      tempFramePoint = new FramePoint(referenceFrame);

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      currentTime.set(0.0);
      this.trajectoryTime.set(trajectoryTimeProvider.getValue());

      initialPositionProvider.getPosition(tempFramePoint);

      tempFramePoint.changeFrame(referenceFrame);
      double y = tempFramePoint.getY();
      double x = tempFramePoint.getX();
      z.set(tempFramePoint.getZ());
      radius.set(Math.sqrt(x * x + y * y));
      double initialAngle = Math.atan2(y, x);
      double finalAngle = initialAngle + desiredRotationAngleProvider.getValue();

      anglePolynomial.setQuintic(0.0, trajectoryTime.getDoubleValue(), initialAngle, 0.0, 0.0, finalAngle, 0.0, 0.0);
   }

   public void compute(double time)
   {
      this.currentTime.set(time);
      anglePolynomial.compute(MathTools.clamp(time, 0.0, trajectoryTime.getDoubleValue()));

      double angle = anglePolynomial.getPosition();
      double angleDot = anglePolynomial.getVelocity();
      double angleDDot = anglePolynomial.getAcceleration();

      if ((time < 0.0) || (time > trajectoryTime.getDoubleValue()))
      {
         angleDot = 0.0;
         angleDDot = 0.0;
      }

//      double r = radius.getDoubleValue();
//
//    double cos = Math.cos(angle);
//    double sin = Math.sin(angle);
//
//    position.setX(r * cos);
//    position.setY(r * sin);
//    position.setZ(z.getValueAsDouble());
//
//    velocity.setX(-r * sin * angleDot);
//    velocity.setY(r * cos * angleDot);
//    velocity.setZ(0.0);
//
//    acceleration.setX(-r * cos * angleDot * angleDot - r * sin * angleDDot);
//    acceleration.setY(-r * sin * angleDot * angleDot + r * cos * angleDDot);
//    acceleration.setZ(0.0);

      CylindricalCoordinatesCalculator.getPosition(position, referenceFrame, angle, radius.getDoubleValue(), z.getDoubleValue());
      CylindricalCoordinatesCalculator.getVelocity(velocity, referenceFrame, angle, angleDot, radius.getDoubleValue(), 0.0, 0.0);
      CylindricalCoordinatesCalculator.getAcceleration(acceleration, referenceFrame, angle, angleDot, angleDDot, radius.getDoubleValue(), 0.0, 0.0, 0.0);
   }

   public FramePoint getPosition()
   {
      return position;
   }

   public boolean isDone()
   {
      return currentTime.getDoubleValue() >= trajectoryTime.getDoubleValue();
   }

   public void getPosition(FramePoint positionToPack)
   {
      positionToPack.setIncludingFrame(position);
   }

   public void getVelocity(FrameVector velocityToPack)
   {
      velocityToPack.setIncludingFrame(velocity);
   }

   public void getAcceleration(FrameVector accelerationToPack)
   {
      accelerationToPack.setIncludingFrame(acceleration);
   }

   public double getAngleFromXAxis()
   {
      return anglePolynomial.getPosition();
   }

   public void getLinearData(FramePoint positionToPack, FrameVector velocityToPack, FrameVector accelerationToPack)
   {
      getPosition(positionToPack);
      getVelocity(velocityToPack);
      getAcceleration(accelerationToPack);
   }

   public void showVisualization()
   {
      // TODO Auto-generated method stub
   }

   public void hideVisualization()
   {
      // TODO Auto-generated method stub
   }
}
