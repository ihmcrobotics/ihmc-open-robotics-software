package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController;

import java.util.List;

import us.ihmc.commonWalkingControlModules.messageHandlers.WalkingMessageHandler;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.contactable.ContactablePlaneBody;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

public class TouchdownErrorCompensator
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final SideDependentList<FramePose3DReadOnly> desiredFootstepPoses = new SideDependentList<>();
   private final SideDependentList<RecyclingArrayList<Point2D>> feetContactPoints = new SideDependentList<>(side -> new RecyclingArrayList<>(Point2D::new));
   private final YoFramePoint3D lowestDesiredContactPoint = new YoFramePoint3D("lowestDesiredContactPoint", ReferenceFrame.getWorldFrame(), registry);
   private final YoFramePoint3D lowestActualContactPoint = new YoFramePoint3D("lowestActualContactPoint", ReferenceFrame.getWorldFrame(), registry);

   private final SideDependentList<YoBoolean> planShouldBeOffsetFromStep = new SideDependentList<>();

   private final SideDependentList<? extends ContactablePlaneBody> contactableFeet;

   private final WalkingMessageHandler walkingMessageHandler;

   private final YoFrameVector3D touchdownErrorVector = new YoFrameVector3D("touchdownErrorVector", ReferenceFrame.getWorldFrame(), registry);

   private final DoubleParameter spatialVelocityThreshold = new DoubleParameter("spatialVelocityThresholdForSupportConfidence",
                                                                                registry,
                                                                                Double.POSITIVE_INFINITY);
   private final DoubleParameter touchdownErrorCorrectionScale = new DoubleParameter("touchdownErrorCorrectionScale", registry, 1.0);
   private final FrameVector3D linearVelocity = new FrameVector3D();

   public TouchdownErrorCompensator(WalkingMessageHandler walkingMessageHandler,
                                    SideDependentList<? extends ContactablePlaneBody> contactableFeet,
                                    YoRegistry parentRegistry)
   {
      this.walkingMessageHandler = walkingMessageHandler;
      this.contactableFeet = contactableFeet;

      for (RobotSide robotSide : RobotSide.values)
      {
         YoBoolean planShouldBeOffsetFromStep = new YoBoolean("planShouldBeOffsetFromStep" + robotSide.getPascalCaseName(), registry);
         planShouldBeOffsetFromStep.set(false);
         this.planShouldBeOffsetFromStep.put(robotSide, planShouldBeOffsetFromStep);
      }

      parentRegistry.addChild(registry);
   }

   public void clear()
   {
      desiredFootstepPoses.clear();
      touchdownErrorVector.setToNaN();

      for (RobotSide robotSide : RobotSide.values)
         planShouldBeOffsetFromStep.get(robotSide).set(false);
   }

   public boolean isFootPositionTrusted(RobotSide robotSide)
   {
      MovingReferenceFrame soleFrame = (MovingReferenceFrame) contactableFeet.get(robotSide).getSoleFrame();
      linearVelocity.setIncludingFrame(soleFrame.getTwistOfFrame().getLinearPart());
      linearVelocity.changeFrame(soleFrame);

      return Math.abs(linearVelocity.getZ()) < spatialVelocityThreshold.getValue();
   }

   public boolean planShouldBeOffsetFromStep(RobotSide robotSide)
   {
      return planShouldBeOffsetFromStep.get(robotSide).getBooleanValue();
   }

   public void registerCompletedFootstep(RobotSide robotSide, FramePose3DReadOnly desiredFootstepPose, Footstep footstep)
   {
      desiredFootstepPoses.put(robotSide, desiredFootstepPose);
      RecyclingArrayList<Point2D> footContactPoints = feetContactPoints.get(robotSide);
      footContactPoints.clear();
      List<Point2D> predictedContactPoints = footstep.getPredictedContactPoints();
      if (predictedContactPoints != null && !predictedContactPoints.isEmpty())
      {
         for (int i = 0; i < predictedContactPoints.size(); i++)
         {
            footContactPoints.add().set(predictedContactPoints.get(i));
         }
      }
      else
      {
         List<FramePoint2D> defaultContactPoints = contactableFeet.get(robotSide).getContactPoints2d();
         for (int i = 0; i < defaultContactPoints.size(); i++)
         {
            footContactPoints.add().set(defaultContactPoints.get(i));
         }
      }

      findLowestContactPoint(desiredFootstepPose, footContactPoints, lowestDesiredContactPoint);
      planShouldBeOffsetFromStep.get(robotSide).set(true);
   }

   public void commitToFootTouchdownError(RobotSide robotSide)
   {
      walkingMessageHandler.commitToFootTouchdownError();
   }

   public void updateFootTouchdownError(RobotSide robotSide, FramePose3DReadOnly actualFootPose)
   {
      if (!planShouldBeOffsetFromStep.get(robotSide).getBooleanValue())
         return;

      findLowestContactPoint(actualFootPose, feetContactPoints.get(robotSide), lowestActualContactPoint);
      double zError = lowestActualContactPoint.getZ() - lowestDesiredContactPoint.getZ();

      if (touchdownErrorVector.containsNaN() || zError <= touchdownErrorVector.getZ())
      {
         touchdownErrorVector.sub(actualFootPose.getPosition(), desiredFootstepPoses.get(robotSide).getPosition());
         touchdownErrorVector.setZ(zError);
         touchdownErrorVector.scale(touchdownErrorCorrectionScale.getValue());
      }

      walkingMessageHandler.updateFootTouchdownError(touchdownErrorVector);
   }

   private void findLowestContactPoint(FramePose3DReadOnly pose, List<? extends Point2DReadOnly> contactPoints, FixedFramePoint3DBasics lowestContactPoint)
   {
      if (contactPoints.isEmpty())
      {
         lowestContactPoint.setToZero();
         return;
      }

      double x = 0;
      double y = 0;
      double z = Double.POSITIVE_INFINITY;

      for (int i = 0; i < contactPoints.size(); i++)
      {
         lowestContactPoint.set(contactPoints.get(i), 0);
         pose.transform(lowestContactPoint);
         if (lowestContactPoint.getZ() < z)
         {
            x = lowestContactPoint.getX();
            y = lowestContactPoint.getY();
            z = lowestContactPoint.getZ();
         }
      }

      lowestContactPoint.set(pose.getReferenceFrame(), x, y, z);
   }
}
