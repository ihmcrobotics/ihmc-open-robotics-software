package us.ihmc.exampleSimulations.beetle.controller;

import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommandList;
import us.ihmc.commonWalkingControlModules.trajectories.TwoWaypointSwingGenerator;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.exampleSimulations.beetle.footContact.SimulatedPlaneContactStateUpdater;
import us.ihmc.exampleSimulations.beetle.parameters.HexapodControllerParameters;
import us.ihmc.exampleSimulations.beetle.parameters.RhinoBeetleJointNameMapAndContactDefinition;
import us.ihmc.exampleSimulations.beetle.planning.FootStepPlanner;
import us.ihmc.exampleSimulations.beetle.referenceFrames.HexapodReferenceFrames;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.robotSide.RobotSextant;
import us.ihmc.robotics.robotSide.SegmentDependentList;
import us.ihmc.robotics.trajectories.TrajectoryType;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;

public class HexapodStepController
{
   private final double controllerDt;
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   private final YoGraphicsListRegistry yoGraphicsListRegistry;

   private final RobotSextant[] leftTriple = {RobotSextant.FRONT_LEFT, RobotSextant.MIDDLE_RIGHT, RobotSextant.HIND_LEFT};
   private final RobotSextant[] rightTriple = {RobotSextant.FRONT_RIGHT, RobotSextant.MIDDLE_LEFT, RobotSextant.HIND_RIGHT};

   private RobotSextant[] legsSwinging = leftTriple;
   private RobotSextant[] legsSupporting = rightTriple;

   private final FramePoint3D desiredPosition = new FramePoint3D();

   private RigidBodyBasics[] rigidBodiesToControl = new RigidBodyBasics[6];

   private final FeedbackControlCommandList feedbackControlCommandList = new FeedbackControlCommandList();
   private final VirtualModelControlCommandList contactStateVMCCommands = new VirtualModelControlCommandList();
   private final InverseDynamicsCommandList contactStateIDCommands = new InverseDynamicsCommandList();
   private final SegmentDependentList<RobotSextant, SimulatedPlaneContactStateUpdater> contactStateUpdaters;

   private final SegmentDependentList<RobotSextant, RigidBodyBasics> shinRigidBodies = new SegmentDependentList<>(RobotSextant.class);
   private final SegmentDependentList<RobotSextant, TwoWaypointSwingGenerator> swingTrajectoryGenerators = new SegmentDependentList<>(RobotSextant.class);
   private final SegmentDependentList<RobotSextant, SpatialFeedbackControlCommand> spatialFeedbackControlCommands = new SegmentDependentList<>(
         RobotSextant.class);
   private final SegmentDependentList<RobotSextant, YoFramePoint3D> desiredPositions = new SegmentDependentList<>(RobotSextant.class);
   private final SegmentDependentList<RobotSextant, YoFramePoint3D> currentPositions = new SegmentDependentList<>(RobotSextant.class);

   private int legIndex = 0;
   private YoBoolean replanTrajectories;

   private final FootStepPlanner footStepPlanner;

   private final YoDouble swingTime;
   private final YoBoolean inStance;
   private final YoDouble timeInStance;
   private final YoDouble transferTime;
   private final YoDouble groundClearance;
   private final YoDouble timeInSwing;
   private final HexapodReferenceFrames referenceFrames;
   private final FullRobotModel fullRobotModel;

   public HexapodStepController(String prefix, FullRobotModel fullRobotModel,
         SegmentDependentList<RobotSextant, SimulatedPlaneContactStateUpdater> contactStateUpdaters, YoGraphicsListRegistry yoGraphicsListRegistry,
         double controllerDt, YoVariableRegistry parentRegistry, HexapodReferenceFrames referenceFrames)
   {
      this.fullRobotModel = fullRobotModel;
      this.contactStateUpdaters = contactStateUpdaters;
      this.controllerDt = controllerDt;
      this.yoGraphicsListRegistry = yoGraphicsListRegistry;
      this.referenceFrames = referenceFrames;

      replanTrajectories = new YoBoolean(prefix + "replanTrajectories", registry);
      swingTime = new YoDouble(prefix + "SwingTime", registry);
      inStance = new YoBoolean(prefix + "InStance", registry);
      timeInStance = new YoDouble(prefix + "TimeInStance", registry);
      transferTime = new YoDouble(prefix + "TransferTime", registry);
      groundClearance = new YoDouble(prefix + "GroundClearance", registry);
      timeInSwing = new YoDouble(prefix + "TimeInSwing", registry);

      footStepPlanner = new FootStepPlanner(prefix, fullRobotModel, referenceFrames, yoGraphicsListRegistry, registry);
      transferTime.set(0.01);
      swingTime.set(0.5);
      groundClearance.set(0.03);
      inStance.set(true);

      RhinoBeetleJointNameMapAndContactDefinition jointMap = new RhinoBeetleJointNameMapAndContactDefinition();

      int i = 0;
      for (RobotSextant robotSextant : RobotSextant.values)
      {
         String name = prefix + robotSextant.toString();
         String jointName = jointMap.getJointNameBeforeFoot(robotSextant);
         OneDoFJointBasics oneDoFJoint = fullRobotModel.getOneDoFJointByName(jointName);
         RigidBodyBasics shinRigidBody = oneDoFJoint.getSuccessor();
         ReferenceFrame footFrame = referenceFrames.getFootFrame(robotSextant);
         FramePose3D footInShinFrame = new FramePose3D(footFrame);
         footInShinFrame.changeFrame(shinRigidBody.getBodyFixedFrame());
         shinRigidBodies.set(robotSextant, shinRigidBody);
         rigidBodiesToControl[i] = shinRigidBody;

         double minSwingHeight = 0.02;
         double maxSwingHeight = groundClearance.getDoubleValue();
         double defaultSwingHeight = 0.02;

         swingTrajectoryGenerators.set(robotSextant, new TwoWaypointSwingGenerator(name, minSwingHeight, maxSwingHeight, defaultSwingHeight, registry, yoGraphicsListRegistry));

         YoFramePoint3D desiredPosition = new YoFramePoint3D(name + "desiredPosition", ReferenceFrame.getWorldFrame(), registry);
         desiredPositions.set(robotSextant, desiredPosition);

         YoFramePoint3D currentPosition = new YoFramePoint3D(name + "currentPosition", ReferenceFrame.getWorldFrame(), registry);
         currentPositions.set(robotSextant, currentPosition);

         SpatialFeedbackControlCommand spatialFeedbackControlCommand = new SpatialFeedbackControlCommand();
         spatialFeedbackControlCommand.set(fullRobotModel.getRootBody(), shinRigidBody);
         spatialFeedbackControlCommand.setWeightsForSolver(new Vector3D(0.0, 0.0, 0.0), new Vector3D(100.0, 100.0, 100.0));
         spatialFeedbackControlCommand.setControlFrameFixedInEndEffector(footInShinFrame);
         spatialFeedbackControlCommands.set(robotSextant, spatialFeedbackControlCommand);

         i++;
      }

      parentRegistry.addChild(registry);
   }

   public void doControl(HexapodControllerParameters parameters, FrameVector3D desiredBodyLinearVelocity, FrameVector3D desiredAngularVelocity)
   {
      contactStateIDCommands.clear();
      contactStateVMCCommands.clear();
      feedbackControlCommandList.clear();
      for (RobotSextant robotSextant : RobotSextant.values)
      {
         SpatialFeedbackControlCommand spatialFeedbackControlCommand = spatialFeedbackControlCommands.get(robotSextant);
         spatialFeedbackControlCommand.setGains(parameters.getFootGains());
      }

      if (inStance.getBooleanValue())
      {
         timeInStance.add(controllerDt);

         for (RobotSextant robotSextant : RobotSextant.values)
         {
            setFootInContact(robotSextant);
         }

         if (timeInStance.getDoubleValue() >= transferTime.getDoubleValue())
         {
            inStance.set(false);
            timeInStance.set(0.0);
            initializeTrajectories(desiredBodyLinearVelocity, desiredAngularVelocity);
         }
      }
      else
      {
         footStepPlanner.drawSupportPolygon(legsSupporting);
         footStepPlanner.drawSwingPolygon(legsSwinging);

         for (RobotSextant robotSextant : legsSwinging)
         {
            replanTrajectories(desiredBodyLinearVelocity, desiredAngularVelocity);
            swingFoot(robotSextant);
         }

         for (RobotSextant robotSextant : legsSupporting)
         {
            setFootInContact(robotSextant);
         }

         if (areFeetDoneSwinging())
         {
            swapSwingingAndSupportLegs();
            inStance.set(true);
         }
      }
   }

   private void replanTrajectories(FrameVector3D desiredBodyLinearVelocity, FrameVector3D desiredAngularVelocity)
   {
      if (swingTime.getDoubleValue() - timeInSwing.getDoubleValue() > 0.1 && replanTrajectories.getBooleanValue())
      {
         legIndex++;
         if(legIndex >= legsSwinging.length)
         {
            legIndex = 0;
         }
         RobotSextant robotSextant = legsSwinging[legIndex];
         //get desired footstep position
         footStepPlanner.getDesiredFootPosition(robotSextant, swingTime.getDoubleValue(), desiredPosition);
         desiredPosition.changeFrame(ReferenceFrame.getWorldFrame());

         //ask for zero velocity in world when done
         finalDesiredVelocity.setToZero(ReferenceFrame.getWorldFrame());

         TwoWaypointSwingGenerator trajectoryGenerator = swingTrajectoryGenerators.get(robotSextant);
         trajectoryGenerator.setFinalConditions(desiredPosition, finalDesiredVelocity);
         trajectoryGenerator.setStepTime(swingTime.getDoubleValue());
         trajectoryGenerator.setTrajectoryType(TrajectoryType.DEFAULT);
         trajectoryGenerator.initialize();
      }
   }

   private final FramePoint3D currentPosition = new FramePoint3D();
   private final Twist currentTwist = new Twist();
   private final FrameVector3D currentVelocity = new FrameVector3D();
   private final FrameVector3D finalDesiredVelocity = new FrameVector3D();
   private final FramePoint3D pointFixedInBodyFrame = new FramePoint3D();

   private void initializeTrajectories(FrameVector3D desiredBodyLinearVelocity, FrameVector3D desiredAngularVelocity)
   {
      for (RobotSextant robotSextant : legsSwinging)
      {
         timeInSwing.set(0.0);

         //update current foot position
         ReferenceFrame footFrame = referenceFrames.getFootFrame(robotSextant);
         currentPosition.setToZero(footFrame);
         currentPosition.changeFrame(ReferenceFrame.getWorldFrame());

         //get current velocity of foot
         shinRigidBodies.get(robotSextant).getBodyFixedFrame().getTwistOfFrame(currentTwist);
         currentTwist.changeFrame(currentTwist.getBaseFrame());

         pointFixedInBodyFrame.setToZero(footFrame);
         pointFixedInBodyFrame.changeFrame(currentTwist.getBaseFrame());

         currentTwist.getLinearVelocityAt(pointFixedInBodyFrame, currentVelocity);
         currentVelocity.changeFrame(ReferenceFrame.getWorldFrame());

         //get desired footstep position
         //         footStepPlanner.getDesiredFootPosition(robotSextant, desiredBodyLinearVelocity, desiredAngularVelocity, swingTime.getDoubleValue(), desiredPosition);
         footStepPlanner.getDesiredFootPosition(robotSextant, swingTime.getDoubleValue(), desiredPosition);
         desiredPosition.changeFrame(ReferenceFrame.getWorldFrame());

         //ask for zero velocity in world when done
         finalDesiredVelocity.setToZero(ReferenceFrame.getWorldFrame());

         TwoWaypointSwingGenerator trajectoryGenerator = swingTrajectoryGenerators.get(robotSextant);
         trajectoryGenerator.setInitialConditions(currentPosition, currentVelocity);
         trajectoryGenerator.setFinalConditions(desiredPosition, finalDesiredVelocity);
         trajectoryGenerator.setSwingHeight(groundClearance.getDoubleValue());
         trajectoryGenerator.setStepTime(swingTime.getDoubleValue());
         trajectoryGenerator.setTrajectoryType(TrajectoryType.DEFAULT);
         trajectoryGenerator.initialize();
      }
   }

   private final FrameVector3D desiredLinearVelocity = new FrameVector3D();
   private final FrameVector3D feedForwardLinearAcceleration = new FrameVector3D();

   private void swingFoot(RobotSextant robotSextant)
   {
      TwoWaypointSwingGenerator footSwingController = swingTrajectoryGenerators.get(robotSextant);
      timeInSwing.add(controllerDt);
      footSwingController.compute(timeInSwing.getDoubleValue());
      footSwingController.getLinearData(desiredPosition, desiredLinearVelocity, feedForwardLinearAcceleration);

      //update current foot position
      ReferenceFrame footFrame = referenceFrames.getFootFrame(robotSextant);
      currentPosition.setToZero(footFrame);
      currentPosition.changeFrame(ReferenceFrame.getWorldFrame());

      desiredPositions.get(robotSextant).set(desiredPosition);
      currentPositions.get(robotSextant).set(currentPosition);

      SpatialFeedbackControlCommand spatialFeedbackControlCommand = spatialFeedbackControlCommands.get(robotSextant);
      spatialFeedbackControlCommand.setInverseDynamics(desiredPosition, desiredLinearVelocity, feedForwardLinearAcceleration);
      feedbackControlCommandList.addCommand(spatialFeedbackControlCommand);

      PlaneContactStateCommand contactState = contactStateUpdaters.get(robotSextant).getNotInContactState();
      contactStateIDCommands.addCommand(contactState);
      contactStateVMCCommands.addCommand(contactState);
   }

   private boolean areFeetDoneSwinging()
   {
      for (RobotSextant robotSextant : legsSwinging)
      {
         TwoWaypointSwingGenerator footSwingController = swingTrajectoryGenerators.get(robotSextant);
         if (!footSwingController.isDone())
         {
            return false;
         }
      }
      return true;
   }

   private void swapSwingingAndSupportLegs()
   {
      if (legsSwinging == leftTriple)
      {
         legsSwinging = rightTriple;
         legsSupporting = leftTriple;
      }
      else
      {
         legsSwinging = leftTriple;
         legsSupporting = rightTriple;
      }
   }

   private void setFootInContact(RobotSextant robotSextant)
   {
      PlaneContactStateCommand inContactState = contactStateUpdaters.get(robotSextant).getInContactState();
      contactStateVMCCommands.addCommand(inContactState);
      contactStateIDCommands.addCommand(inContactState);
   }

   public VirtualModelControlCommandList getVirtualModelControlCommand()
   {
      return contactStateVMCCommands;
   }

   public InverseDynamicsCommandList getInverseDynamicsCommand()
   {
      return contactStateIDCommands;
   }

   public RigidBodyBasics[] getRigidBodiesToControl()
   {
      return rigidBodiesToControl;
   }

   public FeedbackControlCommandList getFeedbackCommandList()
   {
      return feedbackControlCommandList;
   }

   public FeedbackControlCommandList getFeedbackControlTemplate()
   {
      feedbackControlCommandList.clear();
      for (RobotSextant robotSextant : RobotSextant.values)
      {
         feedbackControlCommandList.addCommand(spatialFeedbackControlCommands.get(robotSextant));
      }
      return feedbackControlCommandList;
   }

   public void initialize()
   {

   }
}
