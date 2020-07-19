package us.ihmc.robotics.math.trajectories;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.dataStructures.parameters.ParameterVector3D;
import us.ihmc.robotics.math.filters.RateLimitedYoFramePoint;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

/**
 * Simple trajectory generator that can be used for debugging. Will smoothly move between two parameterized points while holding a zero orientation.
 */
public class LineTrajectory implements PoseTrajectoryGenerator
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final FramePoint3D position = new FramePoint3D();
   private final FrameVector3D linearVelocity = new FrameVector3D();
   private final FrameVector3D linearAcceleration = new FrameVector3D();

   private final FrameQuaternion orientation = new FrameQuaternion();
   private final FrameVector3D angularVelocity = new FrameVector3D();
   private final FrameVector3D angularAcceleration = new FrameVector3D();

   private final double controlDT;
   private final YoDouble integratedPhaseAngle = new YoDouble("PhaseAngle", registry);

   private final DoubleProvider frequency;
   private final Tuple3DReadOnly pointA;
   private final Tuple3DReadOnly pointB;

   private final DoubleProvider maxVelocity;
   private final RateLimitedYoFramePoint limitedPointA;
   private final RateLimitedYoFramePoint limitedPointB;

   private final YoGraphicPosition pointAViz;
   private final YoGraphicPosition pointBViz;

   public LineTrajectory(double controlDT, Tuple3DReadOnly initialPosition, YoRegistry parentRegistry, YoGraphicsListRegistry graphicsListRegistry)
   {
      this.controlDT = controlDT;

      Vector3D initialPositionA = new Vector3D(initialPosition);
      Vector3D initialPositionB = new Vector3D(initialPosition);
      initialPositionA.addX(0.025);
      initialPositionB.addX(-0.025);

      pointA = new ParameterVector3D("PointA", initialPositionA, registry);
      pointB = new ParameterVector3D("PointB", initialPositionB, registry);
      frequency = new DoubleParameter("Frequency", registry, 0.25);
      maxVelocity = new DoubleParameter("MaxVelocity", registry, 0.1);

      limitedPointA = new RateLimitedYoFramePoint("PointALim", "", registry, maxVelocity, controlDT, createFrameTuple(ReferenceFrame.getWorldFrame(), pointA));
      limitedPointB = new RateLimitedYoFramePoint("PointBLim", "", registry, maxVelocity, controlDT, createFrameTuple(ReferenceFrame.getWorldFrame(), pointB));
      pointAViz = new YoGraphicPosition("PointAViz", limitedPointA, 0.025, YoAppearance.Blue());
      pointBViz = new YoGraphicPosition("PointBViz", limitedPointB, 0.025, YoAppearance.Blue());
      graphicsListRegistry.registerYoGraphic(getClass().getSimpleName(), pointAViz);
      graphicsListRegistry.registerYoGraphic(getClass().getSimpleName(), pointBViz);

      parentRegistry.addChild(registry);
   }

   @Override
   public void initialize()
   {
      integratedPhaseAngle.set(0.0);

      limitedPointA.reset();
      limitedPointB.reset();
   }

   @Override
   public void compute(double time)
   {
      integratedPhaseAngle.add(2.0 * Math.PI * frequency.getValue() * controlDT);
      double angle = integratedPhaseAngle.getValue();
      double mult = 2.0 * Math.PI * frequency.getValue();

      double alpha = 0.5 * Math.sin(angle) + 0.5;
      double alphaDot = 0.5 * 2.0 * mult * Math.cos(angle);
      double alphaDDot = -0.5 * 2.0 * 2.0 * mult * mult * Math.sin(angle);

      limitedPointA.update();
      limitedPointB.update();

      position.interpolate(limitedPointA, limitedPointB, alpha);
      linearVelocity.sub(limitedPointB, limitedPointA);
      linearVelocity.scale(alphaDot);
      linearAcceleration.sub(limitedPointB, limitedPointA);
      linearAcceleration.scale(alphaDDot);
   }

   @Override
   public void getVelocity(FrameVector3D velocityToPack)
   {
      velocityToPack.set(linearVelocity);
   }

   @Override
   public void getAcceleration(FrameVector3D accelerationToPack)
   {
      accelerationToPack.set(linearAcceleration);
   }

   @Override
   public void getPosition(FramePoint3D positionToPack)
   {
      positionToPack.set(position);
   }

   @Override
   public void getAngularVelocity(FrameVector3D angularVelocityToPack)
   {
      angularVelocityToPack.setIncludingFrame(angularVelocity);
   }

   @Override
   public void getAngularAcceleration(FrameVector3D angularAccelerationToPack)
   {
      angularAccelerationToPack.setIncludingFrame(angularAcceleration);
   }

   @Override
   public void getOrientation(FrameQuaternion orientationToPack)
   {
      orientationToPack.setIncludingFrame(orientation);
   }

   @Override
   public void getPose(FramePose3D framePoseToPack)
   {
      framePoseToPack.setIncludingFrame(position, orientation);
   }

   @Override
   public void showVisualization()
   {
   }

   @Override
   public void hideVisualization()
   {
   }

   @Override
   public boolean isDone()
   {
      return false;
   }

   private static FrameTuple3DReadOnly createFrameTuple(ReferenceFrame frame, Tuple3DReadOnly tuple)
   {
      return new FrameTuple3DReadOnly()
      {
         @Override
         public ReferenceFrame getReferenceFrame()
         {
            return frame;
         }

         @Override
         public double getZ()
         {
            return tuple.getZ();
         }

         @Override
         public double getY()
         {
            return tuple.getY();
         }

         @Override
         public double getX()
         {
            return tuple.getX();
         }
      };
   }
}
