package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.commons.MathTools;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;
import us.ihmc.robotics.math.frames.YoFramePointInMultipleFrames;
import us.ihmc.robotics.math.frames.YoFrameVectorInMultipleFrames;
import us.ihmc.robotics.math.trajectories.PositionTrajectoryGeneratorInMultipleFrames;
import us.ihmc.robotics.math.trajectories.YoPolynomial;

public class FinalApproachPositionTrajectoryGenerator extends PositionTrajectoryGeneratorInMultipleFrames
{
   public static final double defaultApproachTimeInPercentOfTrajectoryTime = 0.25;

   private final YoVariableRegistry registry;

   private final YoDouble currentTime;
   private final YoDouble trajectoryTime;
   private final YoDouble approachTime;
   private final YoPolynomial xyPolynomial, zPolynomial;

   private final YoFramePointInMultipleFrames initialPosition;
   private final YoFramePointInMultipleFrames finalPosition;
   private final YoFrameVectorInMultipleFrames finalDirection;

   private final YoFramePointInMultipleFrames currentPosition;
   private final YoFrameVectorInMultipleFrames currentVelocity;
   private final YoFrameVectorInMultipleFrames currentAcceleration;

   private final YoDouble approachDistance;

   /** The current trajectory frame chosen by the user. */
   private ReferenceFrame currentTrajectoryFrame;

   /** The tangential plane is the frame in which the trajectory can be expressed in 2D. It is tangential to the final direction vector. */
   private final ReferenceFrame tangentialPlane;
   private final FrameQuaternion rotationPlane = new FrameQuaternion();
   private final AxisAngle axisAngleToWorld = new AxisAngle();

   // For viz
   private final boolean visualize;
   private final YoGraphicsList yoGraphicsList;
   private final BagOfBalls bagOfBalls;
   private final FramePoint3D ballPosition = new FramePoint3D();
   private final int numberOfBalls = 50;

   /** Use a YoBoolean to hide and show visualization with a VariableChangedListener, so it is still working in playback mode. */
   private final YoBoolean showViz;

   public FinalApproachPositionTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, YoVariableRegistry parentRegistry)
   {
      this(namePrefix, false, referenceFrame, parentRegistry, false, null);
   }

   public FinalApproachPositionTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, YoVariableRegistry parentRegistry, boolean visualize,
         YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(namePrefix, false, referenceFrame, parentRegistry, visualize, yoGraphicsListRegistry);
   }

   public FinalApproachPositionTrajectoryGenerator(String namePrefix, boolean allowMultipleFrames, ReferenceFrame referenceFrame,
         YoVariableRegistry parentRegistry)
   {
      this(namePrefix, allowMultipleFrames, referenceFrame, parentRegistry, false, null);
   }

   public FinalApproachPositionTrajectoryGenerator(String namePrefix, boolean allowMultipleFrames, ReferenceFrame referenceFrame,
         YoVariableRegistry parentRegistry, boolean visualize, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      super(allowMultipleFrames, referenceFrame);
      registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      approachTime = new YoDouble(namePrefix + "ApproachTime", registry);
      trajectoryTime = new YoDouble(namePrefix + "TrajectoryTime", registry);
      currentTime = new YoDouble(namePrefix + "Time", registry);
      xyPolynomial = new YoPolynomial(namePrefix + "PositionPolynomial", 4, registry);
      zPolynomial = new YoPolynomial(namePrefix + "VelocityPolynomial", 7, registry);

      tangentialPlane = new ReferenceFrame("tangentialPlane", ReferenceFrame.getWorldFrame())
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.setIdentity();
            rotationPlane.changeFrame(parentFrame);
            transformToParent.setRotation(rotationPlane);
         }
      };

      initialPosition = new YoFramePointInMultipleFrames(namePrefix + "InitialPosition", registry, referenceFrame, tangentialPlane);
      finalPosition = new YoFramePointInMultipleFrames(namePrefix + "FinalPosition", registry, referenceFrame, tangentialPlane);
      finalDirection = new YoFrameVectorInMultipleFrames(namePrefix + "FinalDirection", registry, referenceFrame, tangentialPlane);
      currentPosition = new YoFramePointInMultipleFrames(namePrefix + "CurrentPosition", registry, referenceFrame, tangentialPlane);
      currentVelocity = new YoFrameVectorInMultipleFrames(namePrefix + "CurrentVelocity", registry, referenceFrame, tangentialPlane);
      currentAcceleration = new YoFrameVectorInMultipleFrames(namePrefix + "CurrentAcceleration", registry, referenceFrame, tangentialPlane);

      approachDistance = new YoDouble(namePrefix + "ApproachDistance", registry);

      registerMultipleFramesHolders(initialPosition, finalPosition, currentPosition, currentVelocity, currentAcceleration);

      parentRegistry.addChild(registry);

      this.visualize = visualize && yoGraphicsListRegistry != null;

      if (this.visualize)
      {
         final YoGraphicPosition currentPositionViz = new YoGraphicPosition(namePrefix + "CurrentPosition", currentPosition, 0.025, YoAppearance.Blue());
         final YoGraphicPosition initialPositionViz = new YoGraphicPosition(namePrefix + "InitialPosition", initialPosition, 0.02, YoAppearance.BlueViolet());
         final YoGraphicPosition finalPositionViz = new YoGraphicPosition(namePrefix + "FinalPosition", finalPosition, 0.02, YoAppearance.Red());
         final YoGraphicVector finalDirectionViz = new YoGraphicVector(namePrefix + "FinalDirection",
               finalPosition.buildUpdatedYoFramePointForVisualizationOnly(), finalDirection.buildUpdatedYoFrameVectorForVisualizationOnly(), 0.2,
               YoAppearance.Red());
         yoGraphicsList = new YoGraphicsList(namePrefix + "FinalApproachTraj");
         yoGraphicsList.add(currentPositionViz);
         yoGraphicsList.add(initialPositionViz);
         yoGraphicsList.add(finalPositionViz);
         yoGraphicsList.add(finalDirectionViz);
         yoGraphicsListRegistry.registerYoGraphicsList(yoGraphicsList);

         bagOfBalls = new BagOfBalls(numberOfBalls, 0.01, yoGraphicsList.getLabel(), registry, yoGraphicsListRegistry);

         showViz = new YoBoolean(namePrefix + "ShowViz", registry);
         showViz.addVariableChangedListener(new VariableChangedListener()
         {
            @Override
            public void notifyOfVariableChange(YoVariable<?> v)
            {
               boolean visible = showViz.getBooleanValue();
               currentPositionViz.setVisible(visible);
               initialPositionViz.setVisible(visible);
               finalPositionViz.setVisible(visible);
               finalDirectionViz.setVisible(visible);
               bagOfBalls.setVisible(visible);
            }
         });
         showViz.notifyVariableChangedListeners();
      }
      else
      {
         yoGraphicsList = null;
         bagOfBalls = null;
         showViz = null;
      }
   }

   public void setInitialPosition(double x, double y, double z)
   {
      initialPosition.set(x, y, z);
   }

   public void setInitialPosition(FramePoint3D initialPosition)
   {
      this.initialPosition.set(initialPosition);
   }

   public void setFinalPosition(double x, double y, double z)
   {
      finalPosition.set(x, y, z);
   }

   public void setFinalPosition(FramePoint3D finalPosition)
   {
      this.finalPosition.set(finalPosition);
   }

   private final Vector3D tempVector = new Vector3D();

   public void setFinalApproach(FrameVector3D finalDirection, double approachDistance)
   {
      this.finalDirection.set(finalDirection);
      this.finalDirection.normalize();
      tempVector.set(this.finalDirection);
      tempVector.negate();
      EuclidGeometryTools.axisAngleFromZUpToVector3D(tempVector, axisAngleToWorld);
      rotationPlane.setIncludingFrame(this.finalDirection.getReferenceFrame(), axisAngleToWorld);

      this.approachDistance.set(approachDistance);
   }

   public void setTrajectoryTime(double newTrajectoryTime)
   {
      trajectoryTime.set(newTrajectoryTime);
      approachTime.set(defaultApproachTimeInPercentOfTrajectoryTime * newTrajectoryTime);
   }

   public void setTrajectoryTime(double newTrajectoryTime, double approachTime)
   {
      trajectoryTime.set(newTrajectoryTime);
      MathTools.checkIntervalContains(approachTime, 0.0, newTrajectoryTime);
      this.approachTime.set(approachTime);
   }

   @Override
   public void initialize()
   {
      tangentialPlane.update();
      currentTrajectoryFrame = initialPosition.getReferenceFrame();

      changeFrame(tangentialPlane, false);

      currentTime.set(0.0);
      MathTools.checkIntervalContains(trajectoryTime.getDoubleValue(), 0.0, Double.POSITIVE_INFINITY);
      double tIntermediate = trajectoryTime.getDoubleValue() - approachTime.getDoubleValue();
      xyPolynomial.setCubic(0.0, tIntermediate, 0.0, 0.0, 1.0, 0.0);
      double z0 = initialPosition.getZ();
      double zIntermediate = finalPosition.getZ() + approachDistance.getDoubleValue();
      double zf = finalPosition.getZ();
      zPolynomial.setSexticUsingWaypoint(0.0, tIntermediate, trajectoryTime.getDoubleValue(), z0, 0.0, 0.0, zIntermediate, zf, 0.0, 0.0);

      currentPosition.set(initialPosition);
      currentVelocity.setToZero();
      currentAcceleration.setToZero();

      changeFrame(currentTrajectoryFrame, false);

      if (visualize)
         visualizeTrajectory();
   }

   @Override
   public void compute(double time)
   {
      tangentialPlane.update();
      changeFrame(tangentialPlane, false);

      currentTime.set(time);

      double tIntermediate = trajectoryTime.getDoubleValue() - approachTime.getDoubleValue();
      time = MathTools.clamp(time, 0.0, tIntermediate);
      xyPolynomial.compute(MathTools.clamp(time, 0.0, tIntermediate));
      boolean shouldBeZero = currentTime.getDoubleValue() >= tIntermediate || currentTime.getDoubleValue() < 0.0;
      double alphaDot = shouldBeZero ? 0.0 : xyPolynomial.getVelocity();
      double alphaDDot = shouldBeZero ? 0.0 : xyPolynomial.getAcceleration();

      currentPosition.interpolate(initialPosition, finalPosition, xyPolynomial.getPosition());
      currentVelocity.sub(finalPosition, initialPosition);
      currentVelocity.scale(alphaDot);
      currentAcceleration.sub(finalPosition, initialPosition);
      currentAcceleration.scale(alphaDDot);

      time = MathTools.clamp(currentTime.getDoubleValue(), 0.0, trajectoryTime.getDoubleValue());
      zPolynomial.compute(time);
      shouldBeZero = isDone() || currentTime.getDoubleValue() < 0.0;
      alphaDot = shouldBeZero ? 0.0 : zPolynomial.getVelocity();
      alphaDDot = shouldBeZero ? 0.0 : zPolynomial.getAcceleration();

      currentPosition.setZ(zPolynomial.getPosition());
      currentVelocity.setZ(zPolynomial.getVelocity());
      currentAcceleration.setZ(zPolynomial.getAcceleration());

      changeFrame(currentTrajectoryFrame, false);
   }

   private void visualizeTrajectory()
   {
      for (int i = 0; i < numberOfBalls; i++)
      {
         double t = i / ((double) numberOfBalls - 1) * trajectoryTime.getDoubleValue();
         compute(t);
         currentPosition.getFrameTupleIncludingFrame(ballPosition);
         ballPosition.changeFrame(ReferenceFrame.getWorldFrame());
         bagOfBalls.setBallLoop(ballPosition);
      }
   }

   @Override
   public void showVisualization()
   {
      if (!visualize)
         return;

      showViz.set(true);
   }

   @Override
   public void hideVisualization()
   {
      if (!visualize)
         return;

      showViz.set(false);
   }

   @Override
   public void getPosition(FramePoint3D positionToPack)
   {
      currentPosition.getFrameTupleIncludingFrame(positionToPack);
   }

   @Override
   public void getVelocity(FrameVector3D velocityToPack)
   {
      currentVelocity.getFrameTupleIncludingFrame(velocityToPack);
   }

   @Override
   public void getAcceleration(FrameVector3D accelerationToPack)
   {
      currentAcceleration.getFrameTupleIncludingFrame(accelerationToPack);
   }

   @Override
   public void getLinearData(FramePoint3D positionToPack, FrameVector3D velocityToPack, FrameVector3D accelerationToPack)
   {
      getPosition(positionToPack);
      getVelocity(velocityToPack);
      getAcceleration(accelerationToPack);
   }

   @Override
   public boolean isDone()
   {
      return currentTime.getDoubleValue() >= trajectoryTime.getDoubleValue();
   }

   @Override
   public String toString()
   {
      String ret = "";

      ReferenceFrame currentFrame = initialPosition.getReferenceFrame();

      ret += "Current time: " + currentTime.getDoubleValue() + ", trajectory time: " + trajectoryTime.getDoubleValue();
      ret += "\nCurrent position: " + currentPosition.toStringForASingleReferenceFrame(currentFrame);
      ret += "\nCurrent velocity: " + currentVelocity.toStringForASingleReferenceFrame(currentFrame);
      ret += "\nCurrent acceleration: " + currentAcceleration.toStringForASingleReferenceFrame(currentFrame);
      return ret;
   }
}
