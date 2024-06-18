package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.ExternalWrenchCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.BimanualManipulationCommand;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.SpatialInertia;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.robotics.math.filters.RateLimitedYoVariable;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePose3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class BimanualManipulationManager implements SCS2YoGraphicHolder
{
   private static final double DEFAULT_INITIALIZE_DURATION = 2.0;
   private static final double DEFAULT_TRACKING_ERROR_THRESHOLD = 0.04;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final FullHumanoidRobotModel fullRobotModel;
   private final YoBoolean isEnabled = new YoBoolean("isEnabled", registry);
   private final YoDouble maxEnableRate = new YoDouble("maxEnableRate", registry);
   private final RateLimitedYoVariable squeezeForceScalingFactor;

   private final YoDouble objectMass = new YoDouble("objectMass", registry);
   private final YoDouble requestedSqueezeForce = new YoDouble("requestedSqueezeForce", registry);
   private final YoDouble trackingErrorThreshold = new YoDouble("trackingErrorThreshold", registry);
   private final YoDouble nominalHandDistance = new YoDouble("nominalHandDistance", registry);
   private final YoDouble currentHandDistance = new YoDouble("currentHandDistance", registry);

   private final InverseDynamicsCommandList commandList = new InverseDynamicsCommandList();
   private final SideDependentList<ExternalWrenchCommand> externalWrenchCommands = new SideDependentList<>();
   private final SideDependentList<SpatialInertiaBasics> baselineHandInertias = new SideDependentList<>();
   private final SideDependentList<RigidBodyBasics> hands = new SideDependentList<>();
   private final SideDependentList<MovingReferenceFrame> handControlFrames = new SideDependentList<>();
   private final SpatialInertiaBasics objectInertia = new SpatialInertia();
   private final SpatialInertiaBasics combinedInertia = new SpatialInertia();

   private final SideDependentList<FramePoint3D> handPoints = new SideDependentList<>(new FramePoint3D(), new FramePoint3D());
   private final SideDependentList<YoFramePoint3D> yoHandPoints = new SideDependentList<>();
   private final PoseReferenceFrame midHandFrame;
   private final FramePose3D midHandFramePose = new FramePose3D();
   private final YoFramePose3D yoMidHandFramePose;
   private final RotationMatrix midHandFrameOrientation = new RotationMatrix();
   private final FrameVector3D midHandFrameX = new FrameVector3D();
   private final FrameVector3D midHandFrameY = new FrameVector3D();
   private final FrameVector3D midHandFrameZ = new FrameVector3D();
   private final FrameVector3D worldFrameX = new FrameVector3D(ReferenceFrame.getWorldFrame(), Axis3D.X);

   private final SideDependentList<FrameVector3D> desiredSqueezingForces = new SideDependentList<>(new FrameVector3D(), new FrameVector3D());
   private final SideDependentList<YoFrameVector3D> yoDesiredSqueezingForces = new SideDependentList<>();

   public BimanualManipulationManager(FullHumanoidRobotModel fullRobotModel, double controlDT, YoRegistry parentRegistry)
   {
      this.fullRobotModel = fullRobotModel;

      maxEnableRate.set(1.0 / DEFAULT_INITIALIZE_DURATION);
      squeezeForceScalingFactor = new RateLimitedYoVariable("squeezeForceScalingFactor", registry, maxEnableRate, controlDT);
      squeezeForceScalingFactor.update(0.0);

      midHandFrame = new PoseReferenceFrame("midHandFrame", ReferenceFrame.getWorldFrame());
      yoMidHandFramePose = new YoFramePose3D("midHandFramePose", ReferenceFrame.getWorldFrame(), registry);

      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBodyBasics hand = fullRobotModel.getHand(robotSide);

         ExternalWrenchCommand externalWrenchCommand = new ExternalWrenchCommand();
         externalWrenchCommand.setRigidBody(hand);
         externalWrenchCommands.put(robotSide, externalWrenchCommand);
         commandList.addCommand(externalWrenchCommand);

         hands.put(robotSide, hand);
         handControlFrames.put(robotSide, fullRobotModel.getHandControlFrame(robotSide));

         SpatialInertia baselineHandInertia = new SpatialInertia(hand.getInertia());
         baselineHandInertia.changeFrame(handControlFrames.get(robotSide));
         baselineHandInertias.put(robotSide, baselineHandInertia);

         yoDesiredSqueezingForces.put(robotSide, new YoFrameVector3D(robotSide.getCamelCaseNameForStartOfExpression() + "_DesiredSqueezingForce", ReferenceFrame.getWorldFrame(), registry));
         yoHandPoints.put(robotSide, new YoFramePoint3D(robotSide.getCamelCaseNameForStartOfExpression() + "_HandPoint", ReferenceFrame.getWorldFrame(), registry));
      }

      parentRegistry.addChild(registry);
   }

   public void handleBimanualManipulationCommand(BimanualManipulationCommand command)
   {
      if (command.isDisableRequested())
      {
         isEnabled.set(false);
         squeezeForceScalingFactor.set(0.0);
      }
      else
      {
         isEnabled.set(true);
         objectMass.set(command.getObjectMass());
         requestedSqueezeForce.set(command.getSqueezeForce());

         trackingErrorThreshold.set(command.getTrackingErrorThreshold() > 0.0 ? command.getTrackingErrorThreshold() : DEFAULT_TRACKING_ERROR_THRESHOLD);
         double initializeDuration = command.getInitializeDuration() > 0.2 ? command.getInitializeDuration() : DEFAULT_INITIALIZE_DURATION;
         maxEnableRate.set(1.0 / initializeDuration);

         updateFrames();
         nominalHandDistance.set(currentHandDistance.getValue());
      }
   }

   public void update()
   {
      commandList.clear();

      // Safety check to switch off the applied force when there's too much tracking error.
      isEnabled.set(isEnabled.getValue() && isTracking());

      if (isEnabled.getValue())
      {
         updateFrames();

         squeezeForceScalingFactor.update(1.0);

         // Update inertia properties
         double effectiveObjectMass = squeezeForceScalingFactor.getValue() * objectMass.getValue();
         for (RobotSide robotSide : RobotSide.values)
         {
            RigidBodyBasics hand = hands.get(robotSide);

            double momentOfInertia = 0.05 * effectiveObjectMass * MathTools.square(nominalHandDistance.getValue()); // highly approximate - MOI should stay low since the hand can still rotate easily
            objectInertia.setIncludingFrame(hand.getBodyFixedFrame(),
                                            handControlFrames.get(robotSide),
                                            momentOfInertia,
                                            momentOfInertia,
                                            momentOfInertia,
                                            0.5 * effectiveObjectMass);

            combinedInertia.setIncludingFrame(baselineHandInertias.get(robotSide));
            combinedInertia.add(objectInertia);
            combinedInertia.changeFrame(hand.getBodyFixedFrame());

            hand.getInertia().setIncludingFrame(combinedInertia);
         }

         // Update squeeze force
         double effectiveSqueezeForce = squeezeForceScalingFactor.getValue() * requestedSqueezeForce.getValue();
         for (RobotSide robotSide : RobotSide.values)
         {
            ExternalWrenchCommand externalWrenchCommand = externalWrenchCommands.get(robotSide);
            RigidBodyBasics hand = hands.get(robotSide);
            MovingReferenceFrame handControlFrame = handControlFrames.get(robotSide);
            externalWrenchCommand.getExternalWrench().setBodyFrame(hand.getBodyFixedFrame());
            externalWrenchCommand.getExternalWrench().setReferenceFrame(handControlFrame);

            FrameVector3D desiredSqueezingForce = desiredSqueezingForces.get(robotSide);
            desiredSqueezingForce.setIncludingFrame(midHandFrame, 0.0, robotSide.negateIfLeftSide(effectiveSqueezeForce), 0.0);
            desiredSqueezingForce.changeFrame(handControlFrame);
            externalWrenchCommand.getExternalWrench().getLinearPart().set(desiredSqueezingForce);
            externalWrenchCommand.getExternalWrench().getLinearPart().negate(); // this is an external wrench to compensate for
            externalWrenchCommand.getExternalWrench().getAngularPart().setToZero();
            externalWrenchCommand.getExternalWrench().changeFrame(hand.getBodyFixedFrame());
            commandList.addCommand(externalWrenchCommand);

            yoDesiredSqueezingForces.get(robotSide).setMatchingFrame(desiredSqueezingForce);
         }
      }
      else
      {
         squeezeForceScalingFactor.set(0.0);

         // Set to nominal inertial parameters
         for (RobotSide robotSide : RobotSide.values)
         {
            RigidBodyBasics hand = hands.get(robotSide);
            combinedInertia.setIncludingFrame(baselineHandInertias.get(robotSide));
            combinedInertia.changeFrame(hand.getBodyFixedFrame());
            hand.getInertia().setIncludingFrame(combinedInertia);

            yoDesiredSqueezingForces.get(robotSide).setToZero();
         }

         for (RobotSide robotSide : RobotSide.values)
         {
            ExternalWrenchCommand externalWrenchCommand = externalWrenchCommands.get(robotSide);
            RigidBodyBasics hand = hands.get(robotSide);
            externalWrenchCommand.getExternalWrench().setBodyFrame(hand.getBodyFixedFrame());
            externalWrenchCommand.getExternalWrench().setReferenceFrame(hand.getBodyFixedFrame());

            externalWrenchCommand.getExternalWrench().setToZero();
            commandList.addCommand(externalWrenchCommand);
         }
      }
   }

   private boolean isTracking()
   {
      updateFrames();
      return Math.abs(currentHandDistance.getValue() - nominalHandDistance.getValue()) < trackingErrorThreshold.getValue();
   }

   private void updateFrames()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         handPoints.get(robotSide).setMatchingFrame(handControlFrames.get(robotSide), 0.0, 0.0, 0.0);
         yoHandPoints.get(robotSide).setMatchingFrame(handControlFrames.get(robotSide), 0.0, 0.0, 0.0);
      }

      currentHandDistance.set(handPoints.get(RobotSide.LEFT).distance(handPoints.get(RobotSide.RIGHT)));

      midHandFramePose.setToZero(ReferenceFrame.getWorldFrame());
      midHandFramePose.getPosition().interpolate(handPoints.get(RobotSide.LEFT), handPoints.get(RobotSide.RIGHT), 0.5);

      midHandFrameY.sub(handPoints.get(RobotSide.LEFT), handPoints.get(RobotSide.RIGHT));
      midHandFrameY.normalize();

      midHandFrameZ.cross(worldFrameX, midHandFrameY);
      midHandFrameZ.normalize();

      midHandFrameX.cross(midHandFrameY, midHandFrameZ);
      midHandFrameOrientation.setColumns(midHandFrameX, midHandFrameY, midHandFrameZ);
      midHandFramePose.getOrientation().set(midHandFrameOrientation);

      midHandFrame.setPoseAndUpdate(midHandFramePose);
      yoMidHandFramePose.set(midHandFramePose);
   }

   public boolean isEnabled()
   {
      return isEnabled.getValue();
   }

   public InverseDynamicsCommand<?> getCommand()
   {
      return commandList;
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      group.addChild(YoGraphicDefinitionFactory.newYoGraphicCoordinateSystem3D("MidHandFrameVis", yoMidHandFramePose, 0.01, ColorDefinitions.Red()));
      for (RobotSide robotSide : RobotSide.values)
         group.addChild(YoGraphicDefinitionFactory.newYoGraphicArrow3D(robotSide.getCamelCaseNameForStartOfExpression() + "SqueezingForceViz", yoHandPoints.get(robotSide), yoDesiredSqueezingForces.get(robotSide), 0.01, ColorDefinitions.Red()));
      return group;
   }
}