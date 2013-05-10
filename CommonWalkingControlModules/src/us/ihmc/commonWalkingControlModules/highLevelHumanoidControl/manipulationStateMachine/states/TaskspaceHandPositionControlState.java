package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulationStateMachine.states;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsList;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicReferenceFrame;
import com.yobotics.simulationconstructionset.util.trajectory.PositionTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.trajectories.OrientationTrajectoryGenerator;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.*;
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;

import java.util.ArrayList;
import java.util.Collection;

/**
 * @author twan
 *         Date: 5/9/13
 */
public class TaskspaceHandPositionControlState extends TaskspaceHandControlState
{
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final RigidBodySpatialAccelerationControlModule handSpatialAccelerationControlModule;
   private final PositionTrajectoryGenerator positionTrajectoryGenerator;

   private final OrientationTrajectoryGenerator orientationTrajectoryGenerator;

   private final SpatialAccelerationVector handAcceleration = new SpatialAccelerationVector();

   // viz stuff:
   private final Collection<DynamicGraphicReferenceFrame> dynamicGraphicReferenceFrames = new ArrayList<DynamicGraphicReferenceFrame>();
   private final PoseReferenceFrame desiredPositionFrame;

   // temp stuff:
   private final FramePoint desiredPosition = new FramePoint(worldFrame);
   private final FrameVector desiredVelocity = new FrameVector(worldFrame);
   private final FrameVector desiredAcceleration = new FrameVector(worldFrame);

   private final FrameOrientation desiredOrientation = new FrameOrientation(worldFrame);
   private final FrameVector desiredAngularVelocity = new FrameVector(worldFrame);
   private final FrameVector desiredAngularAcceleration = new FrameVector(worldFrame);

   public TaskspaceHandPositionControlState(RobotSide robotSide, PositionTrajectoryGenerator positionTrajectoryGenerator,
           OrientationTrajectoryGenerator orientationTrajectoryGenerator, RigidBodySpatialAccelerationControlModule handSpatialAccelerationControlModule,
           MomentumBasedController momentumBasedController, GeometricJacobian jacobian, DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry,
           YoVariableRegistry parentRegistry)
   {
      super(momentumBasedController, jacobian, parentRegistry);

      this.handSpatialAccelerationControlModule = handSpatialAccelerationControlModule;
      this.positionTrajectoryGenerator = positionTrajectoryGenerator;
      this.orientationTrajectoryGenerator = orientationTrajectoryGenerator;
      desiredPositionFrame = new PoseReferenceFrame(robotSide.getCamelCaseNameForStartOfExpression() + name + "DesiredFrame", worldFrame);

      if (dynamicGraphicObjectsListRegistry != null)
      {
         DynamicGraphicObjectsList list = new DynamicGraphicObjectsList(name);

         DynamicGraphicReferenceFrame dynamicGraphicReferenceFrame = new DynamicGraphicReferenceFrame(desiredPositionFrame, registry, 0.3);
         dynamicGraphicReferenceFrames.add(dynamicGraphicReferenceFrame);
         list.add(dynamicGraphicReferenceFrame);

         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObjectsList(list);
         list.hideDynamicGraphicObjects();
      }
   }

   protected SpatialAccelerationVector computeDesiredSpatialAcceleration()
   {
      positionTrajectoryGenerator.compute(getTimeInCurrentState());
      orientationTrajectoryGenerator.compute(getTimeInCurrentState());

      positionTrajectoryGenerator.get(desiredPosition);
      positionTrajectoryGenerator.packVelocity(desiredVelocity);
      positionTrajectoryGenerator.packAcceleration(desiredAcceleration);

      orientationTrajectoryGenerator.get(desiredOrientation);
      orientationTrajectoryGenerator.packAngularVelocity(desiredAngularVelocity);
      orientationTrajectoryGenerator.packAngularAcceleration(desiredAngularAcceleration);

      handSpatialAccelerationControlModule.doPositionControl(desiredPosition, desiredOrientation, desiredVelocity, desiredAngularVelocity, desiredAcceleration,
              desiredAngularAcceleration, jacobian.getBase());

      handSpatialAccelerationControlModule.packAcceleration(handAcceleration);

      ReferenceFrame handFrame = handSpatialAccelerationControlModule.getEndEffector().getBodyFixedFrame();
      handAcceleration.changeBodyFrameNoRelativeAcceleration(handFrame);
      handAcceleration.changeFrameNoRelativeMotion(handFrame);

      updateVisualizers();

      return handAcceleration;
   }

   @Override
   public void doTransitionIntoAction()
   {
      positionTrajectoryGenerator.initialize();
      orientationTrajectoryGenerator.initialize();
   }

   @Override
   public void doTransitionOutOfAction()
   {
   }

   private void updateVisualizers()
   {
      desiredPositionFrame.updatePose(desiredPosition, desiredOrientation);
      desiredPositionFrame.update();

      for (DynamicGraphicReferenceFrame dynamicGraphicReferenceFrame : dynamicGraphicReferenceFrames)
      {
         dynamicGraphicReferenceFrame.update();
      }
   }
}
