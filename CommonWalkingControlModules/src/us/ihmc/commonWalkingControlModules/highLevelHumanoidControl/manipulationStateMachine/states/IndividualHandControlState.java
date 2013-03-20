package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulationStateMachine.states;

import java.util.ArrayList;
import java.util.Collection;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.trajectories.OrientationTrajectoryGenerator;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.FormattingTools;
import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.PoseReferenceFrame;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.utilities.screwTheory.Wrench;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsList;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicReferenceFrame;
import com.yobotics.simulationconstructionset.util.trajectory.Finishable;
import com.yobotics.simulationconstructionset.util.trajectory.PositionTrajectoryGenerator;

public class IndividualHandControlState<T extends Enum<T>> extends IndividualManipulationState<T>
{
   private final YoVariableRegistry registry;
   private final RigidBodySpatialAccelerationControlModule handSpatialAccelerationControlModule;
   private final PositionTrajectoryGenerator positionTrajectoryGenerator;
   private final OrientationTrajectoryGenerator orientationTrajectoryGenerator;
   private final RigidBody base;

   private final SpatialAccelerationVector handAcceleration = new SpatialAccelerationVector();

   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

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
   private final Collection<Finishable> finishables = new ArrayList<Finishable>();

   public IndividualHandControlState(T stateEnum, RobotSide robotSide, RigidBody base, PositionTrajectoryGenerator positionTrajectoryGenerator,
         OrientationTrajectoryGenerator orientationTrajectoryGenerator, RigidBodySpatialAccelerationControlModule handSpatialAccelerationControlModule,
         DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, YoVariableRegistry parentRegistry)
   {
      super(stateEnum);

      String stateName = FormattingTools.underscoredToCamelCase(stateEnum.toString(), false) + "State";
      registry = new YoVariableRegistry(stateName);

      this.handSpatialAccelerationControlModule = handSpatialAccelerationControlModule;
      this.positionTrajectoryGenerator = positionTrajectoryGenerator;
      this.orientationTrajectoryGenerator = orientationTrajectoryGenerator;
      this.base = base;

      finishables.add(positionTrajectoryGenerator);
      finishables.add(orientationTrajectoryGenerator);

      desiredPositionFrame = new PoseReferenceFrame(robotSide.getCamelCaseNameForStartOfExpression() + stateName + "DesiredFrame", worldFrame);

      if (dynamicGraphicObjectsListRegistry != null)
      {
         DynamicGraphicObjectsList list = new DynamicGraphicObjectsList(stateName);

         DynamicGraphicReferenceFrame dynamicGraphicReferenceFrame = new DynamicGraphicReferenceFrame(desiredPositionFrame, registry, 0.3);
         dynamicGraphicReferenceFrames.add(dynamicGraphicReferenceFrame);
         list.add(dynamicGraphicReferenceFrame);

         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObjectsList(list);
         list.hideDynamicGraphicObjects();
      }

      parentRegistry.addChild(registry);
   }

   @Override
   public SpatialAccelerationVector getDesiredHandAcceleration()
   {
      return handAcceleration;
   }

   @Override
   public Wrench getHandExternalWrench()
   {
      ReferenceFrame handFrame = handSpatialAccelerationControlModule.getEndEffector().getBodyFixedFrame();
      Wrench ret = new Wrench(handFrame, handFrame); // TODO: garbage generation

      return ret;
   }

   @Override
   public DenseMatrix64F getNullspaceMultipliers()
   {
      return new DenseMatrix64F(0, 1); // TODO: pass on what the spatial acceleration control module does
   }

   @Override
   public void doAction()
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
            desiredAngularAcceleration, base);

      handSpatialAccelerationControlModule.packAcceleration(handAcceleration);

      ReferenceFrame handFrame = handSpatialAccelerationControlModule.getEndEffector().getBodyFixedFrame();
      handAcceleration.changeBodyFrameNoRelativeAcceleration(handFrame);
      handAcceleration.changeFrameNoRelativeMotion(handFrame);

      desiredPositionFrame.updatePose(desiredPosition, desiredOrientation);
      desiredPositionFrame.update();

      updateVisualizers();
   }

   @Override
   public void doTransitionIntoAction()
   {
      positionTrajectoryGenerator.initialize(null);
      orientationTrajectoryGenerator.initialize(null);
   }

   @Override
   public void doTransitionOutOfAction()
   {
   }

   private void updateVisualizers()
   {
      for (DynamicGraphicReferenceFrame dynamicGraphicReferenceFrame : dynamicGraphicReferenceFrames)
      {
         dynamicGraphicReferenceFrame.update();
      }
   }

   public void addFinishable(Finishable finishable)
   {
      finishables.add(finishable);
   }

   @Override
   public boolean isDone()
   {
      for (Finishable finishable : finishables)
      {
         if (!finishable.isDone())
            return false;
      }

      return true;
   }
}
