package us.ihmc.exampleSimulations.beetle.controller;

import javax.vecmath.Vector3d;

import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.commonWalkingControlModules.controlModules.foot.YoFootOrientationGains;
import us.ihmc.commonWalkingControlModules.controlModules.foot.YoFootPositionGains;
import us.ihmc.commonWalkingControlModules.controlModules.foot.YoFootSE3Gains;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.SpatialFeedbackControlCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.PlaneContactStateCommand;
import us.ihmc.commonWalkingControlModules.trajectories.TwoWaypointSwingGenerator;
import us.ihmc.exampleSimulations.beetle.footContact.SimulatedPlaneContactStateUpdater;
import us.ihmc.exampleSimulations.beetle.parameters.RhinoBeetleJointNameMap;
import us.ihmc.exampleSimulations.beetle.parameters.HexapodControllerParameters;
import us.ihmc.exampleSimulations.beetle.planning.FootStepPlanner;
import us.ihmc.exampleSimulations.beetle.referenceFrames.HexapodReferenceFrames;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.robotSide.RobotSextant;
import us.ihmc.robotics.robotSide.SegmentDependentList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.robotics.trajectories.TrajectoryType;

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

   private final FramePoint desiredPosition = new FramePoint();

   private RigidBody[] rigidBodiesToControl = new RigidBody[6];

   private final FeedbackControlCommandList feedbackControlCommandList = new FeedbackControlCommandList();
   private final InverseDynamicsCommandList contactStateCommands = new InverseDynamicsCommandList();
   private final SegmentDependentList<RobotSextant, SimulatedPlaneContactStateUpdater> contactStateUpdaters;

   private final SegmentDependentList<RobotSextant, RigidBody> shinRigidBodies = new SegmentDependentList<>(RobotSextant.class);
   private final SegmentDependentList<RobotSextant, TwoWaypointSwingGenerator> swingTrajectoryGenerators = new SegmentDependentList<>(RobotSextant.class);
   private final SegmentDependentList<RobotSextant, SpatialFeedbackControlCommand> spatialFeedbackControlCommands = new SegmentDependentList<>(
         RobotSextant.class);
   private final SegmentDependentList<RobotSextant, YoFramePoint> desiredPositions = new SegmentDependentList<>(RobotSextant.class);
   private final SegmentDependentList<RobotSextant, YoFramePoint> currentPositions = new SegmentDependentList<>(RobotSextant.class);
   
   private int legIndex = 0;
   private BooleanYoVariable replanTrajectories;

   private final FootStepPlanner footStepPlanner;

   private final TwistCalculator twistCalculator;
   private final DoubleYoVariable swingTime;
   private final BooleanYoVariable inStance;
   private final DoubleYoVariable timeInStance;
   private final DoubleYoVariable transferTime;
   private final DoubleYoVariable groundClearance;
   private final DoubleYoVariable timeInSwing;
   private final HexapodReferenceFrames referenceFrames;
   private final FullRobotModel fullRobotModel;

   public HexapodStepController(String prefix, FullRobotModel fullRobotModel, TwistCalculator twistCalculator,
         SegmentDependentList<RobotSextant, SimulatedPlaneContactStateUpdater> contactStateUpdaters, YoGraphicsListRegistry yoGraphicsListRegistry,
         double controllerDt, YoVariableRegistry parentRegistry, HexapodReferenceFrames referenceFrames)
   {
      this.fullRobotModel = fullRobotModel;
      this.twistCalculator = twistCalculator;
      this.contactStateUpdaters = contactStateUpdaters;
      this.controllerDt = controllerDt;
      this.yoGraphicsListRegistry = yoGraphicsListRegistry;
      this.referenceFrames = referenceFrames;

      replanTrajectories = new BooleanYoVariable(prefix + "replanTrajectories", registry);
      swingTime = new DoubleYoVariable(prefix + "SwingTime", registry);
      inStance = new BooleanYoVariable(prefix + "InStance", registry);
      timeInStance = new DoubleYoVariable(prefix + "TimeInStance", registry);
      transferTime = new DoubleYoVariable(prefix + "TransferTime", registry);
      groundClearance = new DoubleYoVariable(prefix + "GroundClearance", registry);
      timeInSwing = new DoubleYoVariable(prefix + "TimeInSwing", registry);

      footStepPlanner = new FootStepPlanner(prefix, fullRobotModel, referenceFrames, twistCalculator, yoGraphicsListRegistry, registry);
      transferTime.set(0.01);
      swingTime.set(0.5);
      groundClearance.set(0.03);
      inStance.set(true);

      RhinoBeetleJointNameMap jointMap = new RhinoBeetleJointNameMap();

      int i = 0;
      for (RobotSextant robotSextant : RobotSextant.values)
      {
         String name = prefix + robotSextant.toString();
         String jointName = jointMap.getJointNameBeforeFoot(robotSextant);
         OneDoFJoint oneDoFJoint = fullRobotModel.getOneDoFJointByName(jointName);
         RigidBody shinRigidBody = oneDoFJoint.getSuccessor();
         ReferenceFrame footFrame = referenceFrames.getFootFrame(robotSextant);
         FramePose footInShinFrame = new FramePose(footFrame);
         footInShinFrame.changeFrame(shinRigidBody.getBodyFixedFrame());
         shinRigidBodies.set(robotSextant, shinRigidBody);
         rigidBodiesToControl[i] = shinRigidBody;
         swingTrajectoryGenerators.set(robotSextant, new TwoWaypointSwingGenerator(name, 0.02, groundClearance.getDoubleValue(), registry, yoGraphicsListRegistry));

         YoFramePoint desiredPosition = new YoFramePoint(name + "desiredPosition", ReferenceFrame.getWorldFrame(), registry);
         desiredPositions.set(robotSextant, desiredPosition);

         YoFramePoint currentPosition = new YoFramePoint(name + "currentPosition", ReferenceFrame.getWorldFrame(), registry);
         currentPositions.set(robotSextant, currentPosition);

         SpatialFeedbackControlCommand spatialFeedbackControlCommand = new SpatialFeedbackControlCommand();
         spatialFeedbackControlCommand.set(fullRobotModel.getPelvis(), shinRigidBody);
         spatialFeedbackControlCommand.setWeightsForSolver(new Vector3d(0.0, 0.0, 0.0), new Vector3d(100.0, 100.0, 100.0));
         spatialFeedbackControlCommand.setControlFrameFixedInEndEffector(footInShinFrame);
         spatialFeedbackControlCommands.set(robotSextant, spatialFeedbackControlCommand);

         i++;
      }

      parentRegistry.addChild(registry);
   }

   public void doControl(HexapodControllerParameters parameters, FrameVector desiredBodyLinearVelocity, FrameVector desiredAngularVelocity)
   {
      contactStateCommands.clear();
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

   private void replanTrajectories(FrameVector desiredBodyLinearVelocity, FrameVector desiredAngularVelocity)
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

   private final FramePoint currentPosition = new FramePoint();
   private final Twist currentTwist = new Twist();
   private final FrameVector currentVelocity = new FrameVector();
   private final FrameVector finalDesiredVelocity = new FrameVector();
   private final FramePoint pointFixedInBodyFrame = new FramePoint();

   private void initializeTrajectories(FrameVector desiredBodyLinearVelocity, FrameVector desiredAngularVelocity)
   {
      for (RobotSextant robotSextant : legsSwinging)
      {
         timeInSwing.set(0.0);

         //update current foot position
         ReferenceFrame footFrame = referenceFrames.getFootFrame(robotSextant);
         currentPosition.setToZero(footFrame);
         currentPosition.changeFrame(ReferenceFrame.getWorldFrame());

         //get current velocity of foot
         twistCalculator.getTwistOfBody(currentTwist, shinRigidBodies.get(robotSextant));
         currentTwist.changeFrame(currentTwist.getBaseFrame());

         pointFixedInBodyFrame.setToZero(footFrame);
         pointFixedInBodyFrame.changeFrame(currentTwist.getBaseFrame());

         currentTwist.getLinearVelocityOfPointFixedInBodyFrame(currentVelocity, pointFixedInBodyFrame);
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

   private final FrameVector desiredLinearVelocity = new FrameVector();
   private final FrameVector feedForwardLinearAcceleration = new FrameVector();

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
      spatialFeedbackControlCommand.set(desiredPosition, desiredLinearVelocity, feedForwardLinearAcceleration);
      feedbackControlCommandList.addCommand(spatialFeedbackControlCommand);

      PlaneContactStateCommand contactState = contactStateUpdaters.get(robotSextant).getNotInContactState();
      contactStateCommands.addCommand(contactState);
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
      contactStateCommands.addCommand(inContactState);
   }

   public InverseDynamicsCommandList getContactStates()
   {
      return contactStateCommands;
   }

   public RigidBody[] getRigidBodiesToControl()
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
