package us.ihmc.quadrupedRobotics.planning.bodyPath;

import controller_msgs.msg.dds.QuadrupedFootstepStatusMessage;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.QuaternionBasedTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.quadrupedRobotics.estimator.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedRobotics.planning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint2D;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

public class QuadrupedConstantVelocityBodyPathProvider implements QuadrupedPlanarBodyPathProvider
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final QuadrupedXGaitSettingsReadOnly xGaitSettings;
   private final YoDouble timestamp;
   private final ReferenceFrame supportFrame;

   private final YoFramePoint2D startPoint = new YoFramePoint2D("startPoint", worldFrame, registry);
   private final YoDouble startYaw = new YoDouble("startYaw", registry);
   private final YoDouble startTime = new YoDouble("startTime", registry);

   private final Vector3D planarVelocity = new Vector3D();
   private final FramePose2D initialPose = new FramePose2D();

   private final QuadrantDependentList<AtomicReference<QuadrupedFootstepStatusMessage>> footstepStatuses = new QuadrantDependentList<>();
   private final AtomicBoolean recomputeInitialPose = new AtomicBoolean();

   private final Vector3D tempPoint = new Vector3D();
   private final QuaternionBasedTransform tempTransform = new QuaternionBasedTransform();

   public QuadrupedConstantVelocityBodyPathProvider(QuadrupedReferenceFrames referenceFrames, QuadrupedXGaitSettingsReadOnly xGaitSettings,
                                                    YoDouble timestamp, PacketCommunicator packetCommunicator, YoVariableRegistry parentRegistry)
   {
      this.supportFrame = referenceFrames.getCenterOfFeetZUpFrameAveragingLowestZHeightsAcrossEnds();

      for(RobotQuadrant quadrant : RobotQuadrant.values)
      {
         footstepStatuses.set(quadrant, new AtomicReference<>());
      }

      packetCommunicator.attachListener(QuadrupedFootstepStatusMessage.class, (packet) ->
      {
         RobotQuadrant quadrant = RobotQuadrant.fromByte((byte) packet.getFootstepQuadrant());
         footstepStatuses.get(quadrant).set(packet);
         recomputeInitialPose.set(true);
      });

      this.xGaitSettings = xGaitSettings;
      this.timestamp = timestamp;

      parentRegistry.addChild(registry);
   }

   @Override
   public void initialize()
   {
      for(RobotQuadrant quadrant : RobotQuadrant.values)
      {
         footstepStatuses.get(quadrant).set(null);
      }

      RigidBodyTransform supportTransform = supportFrame.getTransformToWorldFrame();
      startTime.set(timestamp.getDoubleValue());
      startYaw.set(supportTransform.getRotationMatrix().getYaw());
      startPoint.set(supportTransform.getTranslationVector());
   }

   public void setPlanarVelocity(double desiredVelocityX, double desiredVelocityY, double desiredVelocityYaw)
   {
      this.planarVelocity.set(desiredVelocityX, desiredVelocityY, desiredVelocityYaw);
   }

   @Override
   public void getPlanarPose(double time, FramePose2D poseToPack)
   {
      update();

      initialPose.set(startPoint.getX(), startPoint.getY(), startYaw.getDoubleValue());
      extrapolatePose(time - startTime.getDoubleValue(), poseToPack, initialPose, planarVelocity);
   }

   private static void extrapolatePose(double time, FramePose2D poseToPack, FramePose2D initialPose, Vector3D planarVelocity)
   {
      double a0 = initialPose.getYaw();
      double x0 = initialPose.getX();
      double y0 = initialPose.getY();

      // initialize forward, lateral, and rotational velocity in pose frame
      double u = planarVelocity.getX();
      double v = planarVelocity.getY();
      double phi = planarVelocity.getZ();

      // compute extrapolated pose assuming a constant planar velocity
      double a, x, y;
      double epsilon = 0.001;
      if (Math.abs(phi) > epsilon)
      {
         a = a0 + phi * time;
         x = x0 + u / phi * (Math.sin(a) - Math.sin(a0)) + v / phi * (Math.cos(a) - Math.cos(a0));
         y = y0 - u / phi * (Math.cos(a) - Math.cos(a0)) + v / phi * (Math.sin(a) - Math.sin(a0));
      }
      else
      {
         a = a0;
         x = x0 + (u * Math.cos(a) - v * Math.sin(a)) * time;
         y = y0 + (u * Math.sin(a) + v * Math.cos(a)) * time;
      }

      poseToPack.setX(x);
      poseToPack.setY(y);
      poseToPack.setYaw(a);
   }

   public void update()
   {
      if(recomputeInitialPose.getAndSet(false))
      {
         recomputeInitialPose();
      }
   }

   private void recomputeInitialPose()
   {
      QuadrupedFootstepStatusMessage latestStatusMessage = getLatestStatusMessage();
      if (latestStatusMessage == null) return;

      double previousStartTime = startTime.getDoubleValue();
      double newStartTime = latestStatusMessage.getDesiredStepInterval().getEndTime();

      startYaw.add(planarVelocity.getZ() * (newStartTime - previousStartTime));
      startTime.set(newStartTime);

      RobotQuadrant quadrant = RobotQuadrant.fromByte((byte) latestStatusMessage.getFootstepQuadrant());
      Point3DReadOnly latestMessageSolePosition = latestStatusMessage.desired_touchdown_position_in_world_;

      double halfStanceLength = quadrant.getEnd().negateIfFrontEnd(0.5 * xGaitSettings.getStanceLength());
      double halfStanceWidth = quadrant.getSide().negateIfLeftSide(0.5 * xGaitSettings.getStanceWidth());

      tempPoint.set(halfStanceLength, halfStanceWidth, 0.0);
      tempTransform.setRotationYaw(startYaw.getDoubleValue());
      tempPoint.applyTransform(tempTransform);
      tempPoint.add(latestMessageSolePosition);
      startPoint.set(tempPoint);
   }

   private QuadrupedFootstepStatusMessage getLatestStatusMessage()
   {
      double latestEndTime = Double.NEGATIVE_INFINITY;
      QuadrupedFootstepStatusMessage latestMessage = null;
      for(RobotQuadrant quadrant : RobotQuadrant.values)
      {
         QuadrupedFootstepStatusMessage message = footstepStatuses.get(quadrant).get();
         if(message != null && message.getDesiredStepInterval().getEndTime() > latestEndTime)
         {
            latestEndTime = message.getDesiredStepInterval().getEndTime();
            latestMessage = message;
         }
      }
      return latestMessage;
   }
}
