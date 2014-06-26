package us.ihmc.graveYard.commonWalkingControlModules.vrc.highLevelHumanoidControl.manipulation.states.toroidManipulation.states;

import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsList;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicObjectsListRegistry;
import com.yobotics.simulationconstructionset.util.graphics.DynamicGraphicReferenceFrame;
import com.yobotics.simulationconstructionset.util.trajectory.Finishable;
import com.yobotics.simulationconstructionset.util.trajectory.PositionTrajectoryGenerator;
import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.trajectories.OrientationTrajectoryGenerator;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.FormattingTools;
import us.ihmc.utilities.math.geometry.*;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.utilities.screwTheory.Wrench;

import java.util.ArrayList;
import java.util.Collection;

public class HandControlState<T extends Enum<T>> extends ToroidManipulationStateInterface<T>
{
   private final YoVariableRegistry registry;
   private final SideDependentList<RigidBodySpatialAccelerationControlModule> handSpatialAccelerationControlModules;
   private final SideDependentList<PositionTrajectoryGenerator> positionTrajectoryGenerators;
   private final SideDependentList<OrientationTrajectoryGenerator> orientationTrajectoryGenerators;
   private final RigidBody base;

   private final SideDependentList<SpatialAccelerationVector> handAccelerations = new SideDependentList<SpatialAccelerationVector>();

   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   // viz stuff:
   private final Collection<DynamicGraphicReferenceFrame> dynamicGraphicReferenceFrames = new ArrayList<DynamicGraphicReferenceFrame>();
   private final SideDependentList<PoseReferenceFrame> desiredPositionFrames = new SideDependentList<PoseReferenceFrame>();

   // temp stuff:
   private final FramePoint desiredPosition = new FramePoint(worldFrame);
   private final FrameVector desiredVelocity = new FrameVector(worldFrame);
   private final FrameVector desiredAcceleration = new FrameVector(worldFrame);

   private final FrameOrientation desiredOrientation = new FrameOrientation(worldFrame);
   private final FrameVector desiredAngularVelocity = new FrameVector(worldFrame);
   private final FrameVector desiredAngularAcceleration = new FrameVector(worldFrame);
   private final Collection<Finishable> finishables = new ArrayList<Finishable>();


   public HandControlState(T stateEnum, RigidBody base, SideDependentList<PositionTrajectoryGenerator> positionTrajectoryGenerators,
                                     SideDependentList<OrientationTrajectoryGenerator> orientationTrajectoryGenerators,
                                     SideDependentList<RigidBodySpatialAccelerationControlModule> handSpatialAccelerationControlModules,
                                     DynamicGraphicObjectsListRegistry dynamicGraphicObjectsListRegistry, YoVariableRegistry parentRegistry)
   {
      super(stateEnum);

      String stateName = FormattingTools.underscoredToCamelCase(stateEnum.toString(), false) + "State";
      registry = new YoVariableRegistry(stateName);

      for (RobotSide robotSide : RobotSide.values)
      {
         handAccelerations.put(robotSide, new SpatialAccelerationVector());
      }

      this.handSpatialAccelerationControlModules = handSpatialAccelerationControlModules;
      this.positionTrajectoryGenerators = positionTrajectoryGenerators;
      this.orientationTrajectoryGenerators = orientationTrajectoryGenerators;
      this.base = base;

      for (RobotSide robotSide : RobotSide.values)
      {
         finishables.add(positionTrajectoryGenerators.get(robotSide));
         finishables.add(orientationTrajectoryGenerators.get(robotSide));

         PoseReferenceFrame referenceFrame = new PoseReferenceFrame(robotSide.getCamelCaseNameForStartOfExpression() + stateName + "DesiredFrame", worldFrame);
         desiredPositionFrames.put(robotSide, referenceFrame);
      }

      if (dynamicGraphicObjectsListRegistry != null)
      {
         DynamicGraphicObjectsList list = new DynamicGraphicObjectsList(stateName);
         for (RobotSide robotSide : RobotSide.values)
         {
            DynamicGraphicReferenceFrame dynamicGraphicReferenceFrame = new DynamicGraphicReferenceFrame(desiredPositionFrames.get(robotSide), registry, 0.3);
            dynamicGraphicReferenceFrames.add(dynamicGraphicReferenceFrame);
            list.add(dynamicGraphicReferenceFrame);
         }
         dynamicGraphicObjectsListRegistry.registerDynamicGraphicObjectsList(list);
         list.hideDynamicGraphicObjects();
      }

      parentRegistry.addChild(registry);
   }

   @Override
   public SpatialAccelerationVector getDesiredHandAcceleration(RobotSide robotSide)
   {
      return handAccelerations.get(robotSide);
   }

   @Override
   public Wrench getHandExternalWrench(RobotSide robotSide)
   {
      ReferenceFrame handFrame = handSpatialAccelerationControlModules.get(robotSide).getEndEffector().getBodyFixedFrame();
      Wrench ret = new Wrench(handFrame, handFrame);    // TODO: garbage generation

      return ret;
   }

   @Override
   public void doAction()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         positionTrajectoryGenerators.get(robotSide).compute(getTimeInCurrentState());
         orientationTrajectoryGenerators.get(robotSide).compute(getTimeInCurrentState());

         positionTrajectoryGenerators.get(robotSide).packLinearData(desiredPosition, desiredVelocity, desiredAcceleration);
         orientationTrajectoryGenerators.get(robotSide).packAngularData(desiredOrientation, desiredAngularVelocity, desiredAngularAcceleration);


         RigidBodySpatialAccelerationControlModule handSpatialAccelerationControlModule = handSpatialAccelerationControlModules.get(robotSide);
         handSpatialAccelerationControlModule.doPositionControl(desiredPosition, desiredOrientation, desiredVelocity, desiredAngularVelocity,
                 desiredAcceleration, desiredAngularAcceleration, base);

         SpatialAccelerationVector handAcceleration = handAccelerations.get(robotSide);
         handSpatialAccelerationControlModule.packAcceleration(handAcceleration);

         ReferenceFrame handFrame = handSpatialAccelerationControlModule.getEndEffector().getBodyFixedFrame();
         handAcceleration.changeBodyFrameNoRelativeAcceleration(handFrame);
         handAcceleration.changeFrameNoRelativeMotion(handFrame);

         desiredPositionFrames.get(robotSide).setPoseAndUpdate(desiredPosition, desiredOrientation);
         desiredPositionFrames.get(robotSide).update();
      }

      updateVisualizers();
   }

   @Override
   public void doTransitionIntoAction()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         positionTrajectoryGenerators.get(robotSide).initialize();
         orientationTrajectoryGenerators.get(robotSide).initialize();
      }
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
