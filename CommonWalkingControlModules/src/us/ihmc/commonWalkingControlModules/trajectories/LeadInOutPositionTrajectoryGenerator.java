package us.ihmc.commonWalkingControlModules.trajectories;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicCoordinateSystem;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.GeometryTools;
import us.ihmc.robotics.math.frames.YoFramePointInMultipleFrames;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.math.frames.YoFrameVectorInMultipleFrames;
import us.ihmc.robotics.math.trajectories.PositionTrajectoryGeneratorInMultipleFrames;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class LeadInOutPositionTrajectoryGenerator extends PositionTrajectoryGeneratorInMultipleFrames
{
   public static final double defaultClearanceTimeInPercentOfTrajectoryTime = 0.20;

   private final YoVariableRegistry registry;

   private final DoubleYoVariable currentTime;
   private final DoubleYoVariable trajectoryTime;
   private final DoubleYoVariable leaveTime;
   private final DoubleYoVariable approachTime;
   private final YoPolynomial xyPolynomial, zPolynomial;

   private final YoFramePointInMultipleFrames initialPosition;
   private final YoFramePointInMultipleFrames finalPosition;
   private final YoFrameVectorInMultipleFrames initialDirection;
   private final YoFrameVectorInMultipleFrames finalDirection;

   private final YoFramePointInMultipleFrames currentPosition;
   private final YoFrameVectorInMultipleFrames currentVelocity;
   private final YoFrameVectorInMultipleFrames currentAcceleration;

   private final DoubleYoVariable leaveDistance;
   private final DoubleYoVariable approachDistance;

   /** The current trajectory frame chosen by the user. */
   private ReferenceFrame currentTrajectoryFrame;

   /**
    * The trajectory is first created in 2D in the distortedPlane. The plane is then distorted to obtain the desired 3D trajectory.
    */
   private final ReferenceFrame distortedPlane;
   private final FramePose currentDistortionPose = new FramePose();
   private final FramePose initialDistortionPose = new FramePose();
   private final FramePose finalDistortionPose = new FramePose();

   // For viz
   private final boolean visualize;
   private final BagOfBalls bagOfBalls;
   private final FramePoint ballPosition = new FramePoint();
   private final int numberOfBalls = 50;

   private final YoFramePose distortedPlanePose;

   /** Use a BooleanYoVariable to hide and show visualization with a VariableChangedListener, so it is still working in playback mode. */
   private final BooleanYoVariable showViz;

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
      leaveTime = new DoubleYoVariable(namePrefix + "LeaveTime", registry);
      approachTime = new DoubleYoVariable(namePrefix + "ApproachTime", registry);
      trajectoryTime = new DoubleYoVariable(namePrefix + "TrajectoryTime", registry);
      currentTime = new DoubleYoVariable(namePrefix + "Time", registry);
      xyPolynomial = new YoPolynomial(namePrefix + "PositionPolynomial", 6, registry);
      zPolynomial = new YoPolynomial(namePrefix + "VelocityPolynomial", 8, registry);

      distortedPlane = new ReferenceFrame("tangentialPlane", ReferenceFrame.getWorldFrame())
      {
         private static final long serialVersionUID = -6071552109268422430L;

         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            currentDistortionPose.changeFrame(parentFrame);
            currentDistortionPose.getPose(transformToParent);
         }
      };

      initialPosition = new YoFramePointInMultipleFrames(namePrefix + "InitialPosition", registry, referenceFrame, distortedPlane);
      finalPosition = new YoFramePointInMultipleFrames(namePrefix + "FinalPosition", registry, referenceFrame, distortedPlane);
      initialDirection = new YoFrameVectorInMultipleFrames(namePrefix + "InitialDirection", registry, referenceFrame, distortedPlane);
      finalDirection = new YoFrameVectorInMultipleFrames(namePrefix + "FinalDirection", registry, referenceFrame, distortedPlane);
      currentPosition = new YoFramePointInMultipleFrames(namePrefix + "CurrentPosition", registry, referenceFrame, distortedPlane);
      currentVelocity = new YoFrameVectorInMultipleFrames(namePrefix + "CurrentVelocity", registry, referenceFrame, distortedPlane);
      currentAcceleration = new YoFrameVectorInMultipleFrames(namePrefix + "CurrentAcceleration", registry, referenceFrame, distortedPlane);

      leaveDistance = new DoubleYoVariable(namePrefix + "LeaveDistance", registry);
      approachDistance = new DoubleYoVariable(namePrefix + "ApproachDistance", registry);

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

         showViz = new BooleanYoVariable(namePrefix + "ShowViz", registry);
         showViz.addVariableChangedListener(new VariableChangedListener()
         {
            @Override
            public void variableChanged(YoVariable<?> v)
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

   public void setInitialLeadOut(FramePoint initialPosition, FrameVector initialDirection, double leaveDistance)
   {
      this.initialPosition.set(initialPosition);
      this.initialDirection.set(initialDirection);
      this.initialDirection.normalize();
      this.initialDirection.get(tempVector);
      GeometryTools.getAxisAngleFromZUpToVector(tempVector, tempAxisAngle);

      initialDistortionPose.setToZero(this.initialPosition.getReferenceFrame());
      initialDistortionPose.setPosition(initialPosition);
      initialDistortionPose.setOrientation(tempAxisAngle);

      this.leaveDistance.set(leaveDistance);
   }

   public void setFinalLeadIn(FramePoint finalPosition, FrameVector finalDirection, double approachDistance)
   {
      this.finalPosition.set(finalPosition);
      this.finalDirection.set(finalDirection);
      this.finalDirection.normalize();
      this.finalDirection.get(tempVector);
      tempVector.negate();
      GeometryTools.getAxisAngleFromZUpToVector(tempVector, tempAxisAngle);

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
      MathTools.checkIfInRange(approachTime, 0.0, newTrajectoryTime - leaveTime);
      this.approachTime.set(approachTime);
      this.leaveTime.set(leaveTime);
   }

   protected DoubleYoVariable getYoLeaveTime()
   {
      return leaveTime;
   }

   @Override
   public void initialize()
   {
      currentTrajectoryFrame = initialPosition.getReferenceFrame();

      MathTools.checkIfInRange(trajectoryTime.getDoubleValue(), 0.0, Double.POSITIVE_INFINITY);
      double t1 = leaveTime.getDoubleValue();
      double t2 = trajectoryTime.getDoubleValue() - approachTime.getDoubleValue();
      double tf = trajectoryTime.getDoubleValue();
      //      xyPolynomial.setCubic(t1, t2, 0.0, 0.0, 1.0, 0.0);
      xyPolynomial.setQuintic(t1, t2, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0);

      currentDistortionPose.setPoseIncludingFrame(initialDistortionPose);
      distortedPlane.update();
      changeFrame(distortedPlane, false);

      double z0 = initialPosition.getZ();
      double z1 = initialPosition.getZ() + leaveDistance.getDoubleValue();

      changeFrame(currentTrajectoryFrame, false);
      currentDistortionPose.setPoseIncludingFrame(finalDistortionPose);
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
      currentVelocity.subAndScale(alphaDot, finalPosition, initialPosition);
      currentAcceleration.subAndScale(alphaDDot, finalPosition, initialPosition);

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
   public void getPosition(FramePoint positionToPack)
   {
      currentPosition.getFrameTupleIncludingFrame(positionToPack);
   }

   @Override
   public void getVelocity(FrameVector velocityToPack)
   {
      currentVelocity.getFrameTupleIncludingFrame(velocityToPack);
   }

   @Override
   public void getAcceleration(FrameVector accelerationToPack)
   {
      currentAcceleration.getFrameTupleIncludingFrame(accelerationToPack);
   }

   @Override
   public void getLinearData(FramePoint positionToPack, FrameVector velocityToPack, FrameVector accelerationToPack)
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
