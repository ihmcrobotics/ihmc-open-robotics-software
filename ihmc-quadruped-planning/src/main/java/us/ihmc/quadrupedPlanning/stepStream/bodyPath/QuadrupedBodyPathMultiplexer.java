package us.ihmc.quadrupedPlanning.stepStream.bodyPath;

import controller_msgs.msg.dds.QuadrupedBodyPathPlanMessage;
import controller_msgs.msg.dds.QuadrupedFootstepStatusMessage;
import us.ihmc.euclid.referenceFrame.FramePose2D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.quadrupedBasics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.quadrupedPlanning.QuadrupedXGaitSettingsReadOnly;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class QuadrupedBodyPathMultiplexer implements QuadrupedPlanarBodyPathProvider
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final QuadrupedWaypointBasedBodyPathProvider waypointBasedPath;
   private final QuadrupedConstantVelocityBodyPathProvider joystickBasedPath;

   private YoBoolean usingJoystickBasedPath = new YoBoolean("usingJoystickBasedPath", registry);

   public QuadrupedBodyPathMultiplexer(QuadrupedReferenceFrames referenceFrames, YoDouble timestamp,
                                       QuadrupedXGaitSettingsReadOnly xGaitSettings, DoubleProvider firstStepDelay,
                                       YoGraphicsListRegistry graphicsListRegistry, YoVariableRegistry parentRegistry)
   {
      waypointBasedPath = new QuadrupedWaypointBasedBodyPathProvider(referenceFrames, timestamp, graphicsListRegistry, registry);
      joystickBasedPath = new QuadrupedConstantVelocityBodyPathProvider(referenceFrames, xGaitSettings, firstStepDelay, timestamp, registry, graphicsListRegistry);
      joystickBasedPath.setShiftPlanBasedOnStepAdjustment(true);

      parentRegistry.addChild(registry);
   }

   @Override
   public void initialize()
   {
      usingJoystickBasedPath.set(true);
      joystickBasedPath.initialize();
   }

   @Override
   public void getPlanarPose(double time, FramePose2D poseToPack)
   {
      if (usingJoystickBasedPath.getBooleanValue())
      {
         if (waypointBasedPath.bodyPathIsAvailable())
         {
            waypointBasedPath.initialize();
            waypointBasedPath.getPlanarPose(time, poseToPack);
            usingJoystickBasedPath.set(false);
         }
         else
         {
            joystickBasedPath.getPlanarPose(time, poseToPack);
         }
      }
      else
      {
         if (waypointBasedPath.isDone())
         {
            joystickBasedPath.initialize();
            joystickBasedPath.getPlanarPose(time, poseToPack);
            usingJoystickBasedPath.set(true);
         }
         else
         {
            waypointBasedPath.getPlanarPose(time, poseToPack);
         }
      }
   }

   public void startedFootstep(RobotQuadrant robotQuadrant, QuadrupedFootstepStatusMessage message)
   {
      joystickBasedPath.startedFootstep(robotQuadrant, message);
   }

   public void completedFootstep(RobotQuadrant robotQuadrant, QuadrupedFootstepStatusMessage message)
   {
      joystickBasedPath.completedFootstep(robotQuadrant, message);
   }


   public void setPlanarVelocityForJoystickPath(double desiredVelocityX, double desiredVelocityY, double desiredVelocityYaw)
   {
      joystickBasedPath.setPlanarVelocity(desiredVelocityX, desiredVelocityY, desiredVelocityYaw);
   }

   public void setShiftPlanBasedOnStepAdjustment(boolean shiftPlanBasedOnStepAdjustment)
   {
      joystickBasedPath.setShiftPlanBasedOnStepAdjustment(shiftPlanBasedOnStepAdjustment);
   }

   public void handleBodyPathPlanMessage(QuadrupedBodyPathPlanMessage bodyPathPlanMessage)
   {
      waypointBasedPath.setBodyPathPlanMessage(bodyPathPlanMessage);
   }
}
