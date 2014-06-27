package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.states;

import java.util.ArrayList;
import java.util.Collection;

import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.IndividualHandControlState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.commonWalkingControlModules.trajectories.OrientationTrajectoryGenerator;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.PoseReferenceFrame;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsList;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicReferenceFrame;
import com.yobotics.simulationconstructionset.util.trajectory.PositionTrajectoryGenerator;

/**
 * @author twan
 *         Date: 5/9/13
 */
public class TaskspaceHandPositionControlState extends TaskspaceHandControlState
{
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   protected final SpatialAccelerationVector handAcceleration = new SpatialAccelerationVector();

   // viz stuff:
   private final Collection<DynamicGraphicReferenceFrame> dynamicGraphicReferenceFrames = new ArrayList<DynamicGraphicReferenceFrame>();
   protected final PoseReferenceFrame desiredPositionFrame;

   // temp stuff:
   protected final FramePoint desiredPosition = new FramePoint(worldFrame);
   protected final FrameVector desiredVelocity = new FrameVector(worldFrame);
   protected final FrameVector desiredAcceleration = new FrameVector(worldFrame);

   protected final FrameOrientation desiredOrientation = new FrameOrientation(worldFrame);
   protected final FrameVector desiredAngularVelocity = new FrameVector(worldFrame);
   protected final FrameVector desiredAngularAcceleration = new FrameVector(worldFrame);

   protected PositionTrajectoryGenerator positionTrajectoryGenerator;
   protected OrientationTrajectoryGenerator orientationTrajectoryGenerator;
   protected RigidBodySpatialAccelerationControlModule handSpatialAccelerationControlModule;

   public TaskspaceHandPositionControlState(String namePrefix, IndividualHandControlState stateEnum, RobotSide robotSide, MomentumBasedController momentumBasedController,
                                            int jacobianId, RigidBody base, RigidBody endEffector,
                                            DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, YoVariableRegistry parentRegistry)
   {
      super(namePrefix, stateEnum, momentumBasedController, jacobianId, base, endEffector, parentRegistry);
      
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

      positionTrajectoryGenerator.packLinearData(desiredPosition, desiredVelocity, desiredAcceleration);
      orientationTrajectoryGenerator.packAngularData(desiredOrientation, desiredAngularVelocity, desiredAngularAcceleration);

      handSpatialAccelerationControlModule.doPositionControl(desiredPosition, desiredOrientation, desiredVelocity, desiredAngularVelocity, desiredAcceleration,
              desiredAngularAcceleration, getBase());

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

   @Override
   public boolean isDone()
   {
      return positionTrajectoryGenerator.isDone() && orientationTrajectoryGenerator.isDone();
   }

   private void updateVisualizers()
   {
      desiredPosition.changeFrame(worldFrame);
      desiredOrientation.changeFrame(worldFrame);
      desiredPositionFrame.setPoseAndUpdate(desiredPosition, desiredOrientation);
      desiredPositionFrame.update();

      for (DynamicGraphicReferenceFrame dynamicGraphicReferenceFrame : dynamicGraphicReferenceFrames)
      {
         dynamicGraphicReferenceFrame.update();
      }
   }

   public void setTrajectory(PositionTrajectoryGenerator positionTrajectoryGenerator, OrientationTrajectoryGenerator orientationTrajectoryGenerator,
         RigidBody base, RigidBodySpatialAccelerationControlModule rigidBodySpatialAccelerationControlModule, ReferenceFrame frameToControlPoseOf)
   {
      this.positionTrajectoryGenerator = positionTrajectoryGenerator;
      this.orientationTrajectoryGenerator = orientationTrajectoryGenerator;
      setBase(base);
      this.taskspaceConstraintData.set(getBase(), getEndEffector());
      this.handSpatialAccelerationControlModule = rigidBodySpatialAccelerationControlModule;
   }

   public ReferenceFrame getReferenceFrame()
   {
      // FIXME: hack

      FramePoint point = new FramePoint();
      positionTrajectoryGenerator.get(point);
      return point.getReferenceFrame();
   }

   public FramePose getDesiredPose()
   {
      positionTrajectoryGenerator.get(desiredPosition);
      desiredPosition.changeFrame(getFrameToControlPoseOf());

      orientationTrajectoryGenerator.get(desiredOrientation);
      desiredOrientation.changeFrame(getFrameToControlPoseOf());

      return new FramePose(desiredPosition, desiredOrientation);
   }

   public ReferenceFrame getFrameToControlPoseOf()
   {
      return handSpatialAccelerationControlModule.getTrackingFrame();
   }
}
