package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
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
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.math.frames.YoFrameVectorInMultipleFrames;
import us.ihmc.robotics.math.trajectories.PositionTrajectoryGeneratorInMultipleFrames;
import us.ihmc.robotics.math.trajectories.YoPolynomial;

public class LeadInOutPositionTrajectoryGenerator extends PositionTrajectoryGeneratorInMultipleFrames
{
   public static final double defaultClearanceTimeInPercentOfTrajectoryTime = 0.20;

   private final YoVariableRegistry registry;

   private final YoDouble currentTime;
   private final YoDouble trajectoryTime;
   private final YoDouble leaveTime;
   private final YoDouble approachTime;
   private final YoPolynomial xyPolynomial, zPolynomial;

   private final YoFramePointInMultipleFrames initialPosition;
   private final YoFramePointInMultipleFrames finalPosition;
   private final YoFrameVectorInMultipleFrames initialDirection;
   private final YoFrameVectorInMultipleFrames finalDirection;

   private final YoFramePointInMultipleFrames currentPosition;
   private final YoFrameVectorInMultipleFrames currentVelocity;
   private final YoFrameVectorInMultipleFrames currentAcceleration;

   private final YoDouble leaveDistance;
   private final YoDouble approachDistance;

   /** The current trajectory frame chosen by the user. */
   private ReferenceFrame currentTrajectoryFrame;

   /**
    * The trajectory is first created in 2D in the distortedPlane. The plane is then distorted to obtain the desired 3D trajectory.
    */
   private final ReferenceFrame distortedPlane;
   private final FramePose3D currentDistortionPose = new FramePose3D();
   private final FramePose3D initialDistortionPose = new FramePose3D();
   private final FramePose3D finalDistortionPose = new FramePose3D();

   // For viz
   private final boolean visualize;
   private final BagOfBalls bagOfBalls;
   private final FramePoint3D ballPosition = new FramePoint3D();
   private final int numberOfBalls = 50;

   private final YoFramePose distortedPlanePose;

   /** Use a YoBoolean to hide and show visualization with a VariableChangedListener, so it is still working in playback mode. */
   private final YoBoolean showViz;

   public LeadInOutPositionTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, YoVariableRegistry parentRegistry)
   {
      this(namePrefix, false, referenceFrame, parentRegistry, false, null);
   }

   public LeadInOutPositionTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, YoVariableRegistry parentRegistry, boolean visualize,
         YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      this(namePrefix, false, referenceFrame, parentRegistry, visualize, yoGraphicsListRegistry);
   }

   public LeadInOutPositionTrajectoryGenerator(String namePrefix, boolean allowMultipleFrames, ReferenceFrame referenceFrame, YoVariableRegistry parentRegistry)
   {
      this(namePrefix, allowMultipleFrames, referenceFrame, parentRegistry, false, null);
   }

   public LeadInOutPositionTrajectoryGenerator(String namePrefix, boolean allowMultipleFrames, ReferenceFrame referenceFrame, YoVariableRegistry parentRegistry,
         boolean visualize, YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      super(allowMultipleFrames, referenceFrame);
      registry = new YoVariableRegistry(namePrefix + getClass().getSimpleName());
      leaveTime = new YoDouble(namePrefix + "LeaveTime", registry);
      approachTime = new YoDouble(namePrefix + "ApproachTime", registry);
      trajectoryTime = new YoDouble(namePrefix + "TrajectoryTime", registry);
      currentTime = new YoDouble(namePrefix + "Time", registry);
      xyPolynomial = new YoPolynomial(namePrefix + "PositionPolynomial", 6, registry);
      zPolynomial = new YoPolynomial(namePrefix + "VelocityPolynomial", 8, registry);

      distortedPlane = new ReferenceFrame("tangentialPlane", ReferenceFrame.getWorldFrame())
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            currentDistortionPose.changeFrame(parentFrame);
            currentDistortionPose.get(transformToParent);
         }
      };

      initialPosition = new YoFramePointInMultipleFrames(namePrefix + "InitialPosition", registry, referenceFrame, distortedPlane);
      finalPosition = new YoFramePointInMultipleFrames(namePrefix + "FinalPosition", registry, referenceFrame, distortedPlane);
      initialDirection = new YoFrameVectorInMultipleFrames(namePrefix + "InitialDirection", registry, referenceFrame, distortedPlane);
      finalDirection = new YoFrameVectorInMultipleFrames(namePrefix + "FinalDirection", registry, referenceFrame, distortedPlane);
      currentPosition = new YoFramePointInMultipleFrames(namePrefix + "CurrentPosition", registry, referenceFrame, distortedPlane);
      currentVelocity = new YoFrameVectorInMultipleFrames(namePrefix + "CurrentVelocity", registry, referenceFrame, distortedPlane);
      currentAcceleration = new YoFrameVectorInMultipleFrames(namePrefix + "CurrentAcceleration", registry, referenceFrame, distortedPlane);

      leaveDistance = new YoDouble(namePrefix + "LeaveDistance", registry);
      approachDistance = new YoDouble(namePrefix + "ApproachDistance", registry);

      registerMultipleFramesHolders(initialPosition, finalPosition, currentPosition, currentVelocity, currentAcceleration);

      parentRegistry.addChild(registry);

      this.visualize = visualize && yoGraphicsListRegistry != null;

      if (this.visualize)
      {
         final YoGraphicPosition currentPositionViz = new YoGraphicPosition(namePrefix + "CurrentPosition", currentPosition, 0.025, YoAppearance.Blue());
         final YoGraphicPosition initialPositionViz = new YoGraphicPosition(namePrefix + "InitialPosition", initialPosition, 0.02, YoAppearance.BlueViolet());
         final YoGraphicPosition finalPositionViz = new YoGraphicPosition(namePrefix + "FinalPosition", finalPosition, 0.02, YoAppearance.Red());
         final YoGraphicVector initialDirectionViz = new YoGraphicVector(namePrefix + "InitialDirection",
               initialPosition.buildUpdatedYoFramePointForVisualizationOnly(), initialDirection.buildUpdatedYoFrameVectorForVisualizationOnly(), 0.2,
               YoAppearance.BlueViolet());
         final YoGraphicVector finalDirectionViz = new YoGraphicVector(namePrefix + "FinalDirection",
               finalPosition.buildUpdatedYoFramePointForVisualizationOnly(), finalDirection.buildUpdatedYoFrameVectorForVisualizationOnly(), 0.2,
               YoAppearance.Red());
         distortedPlanePose = new YoFramePose(namePrefix + "DistortedPlane", ReferenceFrame.getWorldFrame(), registry);
         final YoGraphicCoordinateSystem distortedPlaneViz = new YoGraphicCoordinateSystem(namePrefix + "DistortedPlan", distortedPlanePose, 0.1);
         YoGraphicsList yoGraphicsList = new YoGraphicsList(namePrefix + "FinalApproachTraj");
         yoGraphicsList.add(currentPositionViz);
         yoGraphicsList.add(initialPositionViz);
         yoGraphicsList.add(finalPositionViz);
         yoGraphicsList.add(initialDirectionViz);
         yoGraphicsList.add(finalDirectionViz);
         yoGraphicsList.add(distortedPlaneViz);
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
               initialDirectionViz.setVisible(visible);
               finalDirectionViz.setVisible(visible);
               distortedPlaneViz.setVisible(visible);
               bagOfBalls.setVisible(visible);
               if (!visible)
                  bagOfBalls.hideAll();
            }
         });
         showViz.notifyVariableChangedListeners();
      }
      else
      {
         distortedPlanePose = null;
         bagOfBalls = null;
         showViz = null;
      }
   }

   private final Vector3D tempVector = new Vector3D();
   private final AxisAngle tempAxisAngle = new AxisAngle();

   public void setInitialLeadOut(FramePoint3DReadOnly initialPosition, FrameVector3DReadOnly initialDirection, double leaveDistance)
   {
      this.initialPosition.set(initialPosition);
      this.initialDirection.set(initialDirection);
      this.initialDirection.normalize();
      tempVector.set(this.initialDirection);
      EuclidGeometryTools.axisAngleFromZUpToVector3D(tempVector, tempAxisAngle);

      initialDistortionPose.setToZero(this.initialPosition.getReferenceFrame());
      initialDistortionPose.setPosition(initialPosition);
      initialDistortionPose.setOrientation(tempAxisAngle);

      this.leaveDistance.set(leaveDistance);
   }

   public void setFinalLeadIn(FramePoint3DReadOnly finalPosition, FrameVector3DReadOnly finalDirection, double approachDistance)
   {
      this.finalPosition.set(finalPosition);
      this.finalDirection.set(finalDirection);
      this.finalDirection.normalize();
      tempVector.set(this.finalDirection);
      tempVector.negate();
      EuclidGeometryTools.axisAngleFromZUpToVector3D(tempVector, tempAxisAngle);

      finalDistortionPose.setToZero(this.finalPosition.getReferenceFrame());
      finalDistortionPose.setPosition(finalPosition);
      finalDistortionPose.setOrientation(tempAxisAngle);

      this.approachDistance.set(approachDistance);
   }

   public void setTrajectoryTime(double newTrajectoryTime)
   {
      trajectoryTime.set(newTrajectoryTime);
      approachTime.set(defaultClearanceTimeInPercentOfTrajectoryTime * newTrajectoryTime);
      leaveTime.set(defaultClearanceTimeInPercentOfTrajectoryTime * newTrajectoryTime);
   }

   public void setTrajectoryTime(double newTrajectoryTime, double leaveTime, double approachTime)
   {
      trajectoryTime.set(newTrajectoryTime);
      MathTools.checkIntervalContains(approachTime, 0.0, newTrajectoryTime - leaveTime);
      this.approachTime.set(approachTime);
      this.leaveTime.set(leaveTime);
   }

   protected YoDouble getYoLeaveTime()
   {
      return leaveTime;
   }

   @Override
   public void initialize()
   {
      currentTrajectoryFrame = initialPosition.getReferenceFrame();

      MathTools.checkIntervalContains(trajectoryTime.getDoubleValue(), 0.0, Double.POSITIVE_INFINITY);
      double t1 = leaveTime.getDoubleValue();
      double t2 = trajectoryTime.getDoubleValue() - approachTime.getDoubleValue();
      double tf = trajectoryTime.getDoubleValue();
      //      xyPolynomial.setCubic(t1, t2, 0.0, 0.0, 1.0, 0.0);
      xyPolynomial.setQuintic(t1, t2, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0);

      currentDistortionPose.setIncludingFrame(initialDistortionPose);
      distortedPlane.update();
      changeFrame(distortedPlane, false);

      double z0 = initialPosition.getZ();
      double z1 = initialPosition.getZ() + leaveDistance.getDoubleValue();

      changeFrame(currentTrajectoryFrame, false);
      currentDistortionPose.setIncludingFrame(finalDistortionPose);
      distortedPlane.update();
      changeFrame(distortedPlane, false);

      double z2 = finalPosition.getZ() + approachDistance.getDoubleValue();
      double zf = finalPosition.getZ();
      //      zPolynomial.setQuinticTwoWaypoints(0.0, t1, t2, tf, z0, 0.0, z1, z2, zf, 0.0);
      zPolynomial.setSepticInitialAndFinalAcceleration(0.0, t1, t2, tf, z0, 0.0, 0.0, z1, z2, zf, 0.3, 0.0);

      changeFrame(currentTrajectoryFrame, false);

      if (visualize)
         visualizeTrajectory();

      currentTime.set(0.0);
      currentPosition.set(initialPosition);
      currentVelocity.setToZero();
      currentAcceleration.setToZero();

   }

   @Override
   public void compute(double time)
   {
      currentTime.set(time);

      double t1 = leaveTime.getDoubleValue();
      double t2 = trajectoryTime.getDoubleValue() - approachTime.getDoubleValue();
      double tf = trajectoryTime.getDoubleValue();

      xyPolynomial.compute(MathTools.clamp(time, t1, t2));

      currentDistortionPose.interpolate(initialDistortionPose, finalDistortionPose, xyPolynomial.getPosition());
      distortedPlanePose.setAndMatchFrame(currentDistortionPose);
      distortedPlane.update();
      changeFrame(distortedPlane, false);

      boolean shouldBeZero = currentTime.getDoubleValue() >= t2 || currentTime.getDoubleValue() < t1;
      double alphaDot = shouldBeZero ? 0.0 : xyPolynomial.getVelocity();
      double alphaDDot = shouldBeZero ? 0.0 : xyPolynomial.getAcceleration();

      currentPosition.interpolate(initialPosition, finalPosition, xyPolynomial.getPosition());
      currentVelocity.sub(finalPosition, initialPosition);
      currentVelocity.scale(alphaDot);
      currentAcceleration.sub(finalPosition, initialPosition);
      currentAcceleration.scale(alphaDDot);

      zPolynomial.compute(MathTools.clamp(time, 0.0, tf));

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
         ballPosition.setIncludingFrame(currentPosition);
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
      positionToPack.setIncludingFrame(currentPosition);
   }

   @Override
   public void getVelocity(FrameVector3D velocityToPack)
   {
      velocityToPack.setIncludingFrame(currentVelocity);
   }

   @Override
   public void getAcceleration(FrameVector3D accelerationToPack)
   {
      accelerationToPack.setIncludingFrame(currentAcceleration);
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
