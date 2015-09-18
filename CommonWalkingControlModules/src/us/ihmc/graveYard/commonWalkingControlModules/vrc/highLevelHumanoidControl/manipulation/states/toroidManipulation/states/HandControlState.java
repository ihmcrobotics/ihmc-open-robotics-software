package us.ihmc.graveYard.commonWalkingControlModules.vrc.highLevelHumanoidControl.manipulation.states.toroidManipulation.states;

import java.util.ArrayList;
import java.util.Collection;

import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.tools.FormattingTools;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.trajectories.Finishable;
import us.ihmc.robotics.math.trajectories.OrientationTrajectoryGenerator;
import us.ihmc.robotics.math.trajectories.PositionTrajectoryGenerator;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicReferenceFrame;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsList;
import us.ihmc.simulationconstructionset.yoUtilities.graphics.YoGraphicsListRegistry;


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
   private final Collection<YoGraphicReferenceFrame> dynamicGraphicReferenceFrames = new ArrayList<YoGraphicReferenceFrame>();
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
                                     YoGraphicsListRegistry yoGraphicsListRegistry, YoVariableRegistry parentRegistry)
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

      if (yoGraphicsListRegistry != null)
      {
         YoGraphicsList list = new YoGraphicsList(stateName);
         for (RobotSide robotSide : RobotSide.values)
         {
            YoGraphicReferenceFrame dynamicGraphicReferenceFrame = new YoGraphicReferenceFrame(desiredPositionFrames.get(robotSide), registry, 0.3);
            dynamicGraphicReferenceFrames.add(dynamicGraphicReferenceFrame);
            list.add(dynamicGraphicReferenceFrame);
         }
         yoGraphicsListRegistry.registerYoGraphicsList(list);
         list.hideYoGraphics();
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
      for (YoGraphicReferenceFrame dynamicGraphicReferenceFrame : dynamicGraphicReferenceFrames)
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
