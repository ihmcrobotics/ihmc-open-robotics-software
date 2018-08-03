package us.ihmc.exampleSimulations.simple3DWalker.FullRobotModel;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.plotting.artifact.Artifact;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.math.filters.AlphaFilteredYoVariable;
import us.ihmc.robotics.math.trajectories.YoMinimumJerkTrajectory;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.simple3dWalker.Robot.HipAngleCapturePointCalculator3D;
import us.ihmc.simulationConstructionSetTools.tools.RobotTools.SCSRobotFromInverseDynamicsRobotModel;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.*;

public class SimpleWalkerControllerForModel implements RobotController
{
   private final double MAX_HIP_ANGLE = 0.8;
   private double HIP_DEFUALT_P_GAIN = 100.0;
   private double HIP_DEFUALT_D_GAIN = 10.0;

   private double KNEE_DEFUALT_P_GAIN = 10000.0;
   private double KNEE_DEFUALT_D_GAIN = 1000.0;

   private double deltaT;

   private SimpleWalkerRobotModel robotModel;
   private YoVariableRegistry registry = new YoVariableRegistry("Controller");
   private SideDependentList<PIDController> kneeControllers = new SideDependentList<PIDController>();
   private SideDependentList<PIDController> hipPitchControllers = new SideDependentList<PIDController>();
   private SideDependentList<PIDController> hipRollControllers = new SideDependentList<PIDController>();
   private SideDependentList<PIDController> hipYawControllers = new SideDependentList<PIDController>();
   private SideDependentList<PIDController> anklePitchControllers = new SideDependentList<PIDController>();
   private SideDependentList<PIDController> ankleRollControllers = new SideDependentList<PIDController>();
   private SideDependentList<PIDController> ankleYawControllers = new SideDependentList<PIDController>();
   private SideDependentList<PIDController> angularMomentumPitchControllers = new SideDependentList<PIDController>();
   private SideDependentList<PIDController> angularMomentumRollControllers = new SideDependentList<PIDController>();

   private YoDouble desiredKneeExtension = new YoDouble("desiredKneeExtension", registry);
   private YoDouble desiredPitch = new YoDouble("desiredPitch", registry);
   private YoDouble desiredRoll = new YoDouble("desiredRoll", registry);
   private YoDouble desiredHeight = new YoDouble("desiredHeight", registry);
   private YoDouble swingTime = new YoDouble("swingTime", registry);
   private YoDouble desiredSwingLegHipPitchAngle = new YoDouble("desiredSwingLegHipPitchAngle", registry);
   private YoDouble desiredSwingLegHipRollAngle = new YoDouble("desiredSwingLegHipRollAngle", registry);
   private YoDouble scaleForVelToAngle = new YoDouble("scaleForVelToAngle", registry);
   private YoDouble desiredKneeStance = new YoDouble("desiredKneeStance", registry);
   private YoDouble pitchAngleForCapture = new YoDouble("pitchAngleForCapture", registry);
   private YoDouble rollAngleForCapture = new YoDouble("rollAngleForCapture", registry);
   private YoDouble feedForwardAngle = new YoDouble("feedForwardAngle", registry);
   private YoDouble velocityErrorPitchAngle = new YoDouble("velocityErrorPitchAngle", registry);
   private YoDouble velocityErrorRollAngle = new YoDouble("velocityErrorRollAngle", registry);
   private YoDouble feedForwardGain = new YoDouble("feedForwardGain", registry);
   private YoDouble lastStepHipPitchAngle = new YoDouble("lastStepHipPitchAngle", registry);
   private YoDouble lastStepHipRollAngle = new YoDouble("lastStepHipRollAngle", registry);
   private YoDouble stepToStepHipAngleDelta = new YoDouble("stepToStepHipAngleDelta", registry);

   private YoDouble swingTimeForThisStep = new YoDouble("swingTimeForThisStep", registry);
   private YoBoolean initalizedKneeExtension = new YoBoolean("initalizedKneeExtension", registry);
   private YoBoolean initalizedKneeDoubleExtension = new YoBoolean("initalizedKneeDoubleExtension", registry);

   private YoDouble kneeMoveStartTime = new YoDouble("kneeMoveStartTime", registry);
   private YoDouble startingHipPitchAngle = new YoDouble("startingHipPitchAngle", registry);
   private YoDouble startingHipRollAngle = new YoDouble("startingHipRollAngle", registry);
   private YoDouble yoTimeInState = new YoDouble("timeInState", registry);
   private YoDouble maxVelocityErrorAngle = new YoDouble("maxVelocityErrorAngle", registry);

   private YoDouble ankleControlTorquePitch = new YoDouble("ankleControlTorquePitch", registry);
   private YoDouble ankleControlTorqueRoll = new YoDouble("ankleControlTorqueRoll", registry);
   private YoDouble coPLocationX = new YoDouble("CoPX", registry);
   private YoDouble coPLocationY = new YoDouble("CoPY", registry);

   private YoDouble desiredBodyVelocityX = new YoDouble("desiredBodyVelocityX", registry);
   private YoDouble alphaFilterVariableX = new YoDouble("alphaFilterVariableX", registry);
   private AlphaFilteredYoVariable filteredDesiredVelocityX = new AlphaFilteredYoVariable("filteredDesiredVelocityX", registry, alphaFilterVariableX,
                                                                                          desiredBodyVelocityX);

   private YoDouble desiredBodyVelocityY = new YoDouble("desiredBodyVelocityY", registry);
   private YoDouble alphaFilterVariableY = new YoDouble("alphaFilterVariableY", registry);
   private AlphaFilteredYoVariable filteredDesiredVelocityY = new AlphaFilteredYoVariable("filteredDesiredVelocityY", registry, alphaFilterVariableY,
                                                                                          desiredBodyVelocityY);

   private YoMinimumJerkTrajectory trajectorySwingHipPitch;
   private YoMinimumJerkTrajectory trajectorySwingHipRoll;
   private YoMinimumJerkTrajectory trajectorySwingKnee;

   private final YoFramePoint3D centerOfMassPosition;
   private final YoFramePoint2D centerOfMassPosition2D;
   private final YoFrameVector3D centerOfMassVelocity;
   private final YoFrameVector3D centerOfMassAcceleration;

   private final YoFramePoint3D desiredICP;

   private final YoFramePoint3D currentICP;
   private final YoFramePoint2D currentICP2D;

   private final YoFramePoint3D currentCOP;
   private final YoFramePoint2D currentCOP2D;

   private final YoDouble omega;

   private YoEnum<RobotSide> swingLeg = new YoEnum<RobotSide>("swingLeg", registry, RobotSide.class);

   private StateMachine<ControllerState, State> stateMachine;

   private YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

   private YoGraphicPosition currentCoPGraphic;
   private Artifact currentCoPGraphicArtifact;

   private YoGraphicPosition currentCoMGraphic;
   private Artifact currentCoMGraphicArtifact;

   private YoGraphicPosition currentICPGraphic;
   private Artifact currentICPGraphicArtifact;

   private final SCSRobotFromInverseDynamicsRobotModel scsRobot;


   public SimpleWalkerControllerForModel(SimpleWalkerRobotModel robotModel, double deltaT)
   {
      this.robotModel = robotModel;
      this.deltaT = deltaT;
      scsRobot = robotModel.getSCSRobot();

      centerOfMassPosition = new YoFramePoint3D("centerOfMass", ReferenceFrame.getWorldFrame(), registry);
      centerOfMassPosition2D = new YoFramePoint2D("centerOfMass2D", ReferenceFrame.getWorldFrame(), registry);
      centerOfMassVelocity  = new YoFrameVector3D("centerOfMassVelocity", ReferenceFrame.getWorldFrame(), registry);
      centerOfMassAcceleration  = new YoFrameVector3D("centerOfMassAcceleration", ReferenceFrame.getWorldFrame(), registry);
      omega = new YoDouble("omega", registry);

      desiredICP = new YoFramePoint3D("desiredICP", ReferenceFrame.getWorldFrame(), registry);

      currentICP = new YoFramePoint3D("currentICP", ReferenceFrame.getWorldFrame(), registry);
      currentICP2D = new YoFramePoint2D("currentICP2D", ReferenceFrame.getWorldFrame(), registry);

      currentCOP = new YoFramePoint3D("currentCOP", ReferenceFrame.getWorldFrame(), registry);
      currentCOP2D = new YoFramePoint2D("currentCOP2D", ReferenceFrame.getWorldFrame(),registry);

      YoFramePoint3D centerOfPressure = new YoFramePoint3D("centerOfPressure", ReferenceFrame.getWorldFrame(), registry);
      YoFrameVector3D reactionForce = new YoFrameVector3D("reactionForce", ReferenceFrame.getWorldFrame(), registry);

      currentCoPGraphic = new YoGraphicPosition("CoPgraph",currentCOP2D,0.05,YoAppearance.Blue());
      yoGraphicsListRegistry.registerYoGraphic("CoPgraphRegistry", currentCoPGraphic);
      currentCoPGraphicArtifact = currentCoPGraphic.createArtifact();
      yoGraphicsListRegistry.registerArtifact("CoPArtifact", currentCoPGraphicArtifact);

      currentCoMGraphic = new YoGraphicPosition("CoMgraph",centerOfMassPosition2D,0.01 ,YoAppearance.Black());
      yoGraphicsListRegistry.registerYoGraphic("CoMgraphRegistry", currentCoMGraphic);
      currentCoMGraphicArtifact = currentCoMGraphic.createArtifact();
      yoGraphicsListRegistry.registerArtifact("CoPArtifact", currentCoMGraphicArtifact);

      currentICPGraphic = new YoGraphicPosition("ICPgraph",currentICP2D,0.03 ,YoAppearance.Red());
      yoGraphicsListRegistry.registerYoGraphic("ICPgraphRegistry", currentICPGraphic);
      currentICPGraphicArtifact = currentICPGraphic.createArtifact();
      yoGraphicsListRegistry.registerArtifact("ICPArtifact", currentICPGraphicArtifact);





      for (RobotSide robotSide : RobotSide.values)
      {
         for(GroundContactPoint gCpoint : robotModel.getgCpoints().get(robotSide))
         {
            YoFrameVector3D reactionForcePoint = gCpoint.getYoForce();
            reactionForce.add(reactionForcePoint);
         }

         YoGraphicVector reactionForceViz = new YoGraphicVector("reactionForceViz", currentCOP, reactionForce, YoAppearance.Red());
         reactionForceViz.setVisible(true);
         reactionForceViz.setDrawArrowhead(true);

         PIDController pidController = new PIDController(robotSide.getSideNameFirstLetter() + "_Knee", registry);
         pidController.setProportionalGain(KNEE_DEFUALT_P_GAIN);
         pidController.setDerivativeGain(KNEE_DEFUALT_D_GAIN);
         kneeControllers.put(robotSide, pidController);

         pidController = new PIDController(robotSide.getSideNameFirstLetter() + "_HipPitch", registry);
         pidController.setProportionalGain(HIP_DEFUALT_P_GAIN);
         pidController.setDerivativeGain(HIP_DEFUALT_D_GAIN);
         hipPitchControllers.put(robotSide, pidController);

         pidController = new PIDController(robotSide.getSideNameFirstLetter() + "_HipRoll", registry);
         pidController.setProportionalGain(HIP_DEFUALT_P_GAIN);
         pidController.setDerivativeGain(HIP_DEFUALT_D_GAIN);
         hipRollControllers.put(robotSide, pidController);

         pidController = new PIDController(robotSide.getSideNameFirstLetter() + "_HipYaw", registry);
         pidController.setProportionalGain(HIP_DEFUALT_P_GAIN);
         pidController.setDerivativeGain(HIP_DEFUALT_D_GAIN);
         hipYawControllers.put(robotSide, pidController);

         pidController = new PIDController(robotSide.getSideNameFirstLetter() + "_AnklePitch", registry);
         pidController.setProportionalGain(5);
         pidController.setDerivativeGain(1);
         anklePitchControllers.put(robotSide, pidController);

         pidController = new PIDController(robotSide.getSideNameFirstLetter() + "_AnkleRoll", registry);
         pidController.setProportionalGain(5);
         pidController.setDerivativeGain(1);
         ankleRollControllers.put(robotSide, pidController);

         pidController = new PIDController(robotSide.getSideNameFirstLetter() + "_AnkleYaw", registry);
         pidController.setProportionalGain(0.1);
         pidController.setIntegralGain(0.1);
         pidController.setDerivativeGain(0.03);
         ankleYawControllers.put(robotSide, pidController);

         pidController = new PIDController(robotSide.getSideNameFirstLetter() + "_AngularMomentumPitch", registry);
         pidController.setProportionalGain(10);
         pidController.setIntegralGain(0.5);
         pidController.setDerivativeGain(1);
         angularMomentumPitchControllers.put(robotSide, pidController);

         pidController = new PIDController(robotSide.getSideNameFirstLetter() + "_AngularMomentumRoll", registry);
         pidController.setProportionalGain(10);
         pidController.setIntegralGain(0.5);
         pidController.setDerivativeGain(1);
         angularMomentumRollControllers.put(robotSide, pidController);
      }

      trajectorySwingHipPitch = new YoMinimumJerkTrajectory("trajectorySwingHipPitch", registry);
      trajectorySwingHipRoll = new YoMinimumJerkTrajectory("trajectorySwingHipRoll", registry);
      trajectorySwingKnee = new YoMinimumJerkTrajectory("trajectorySwingKnee", registry);

      desiredHeight.set(robotModel.nominalHeight);
      desiredKneeStance.set(robotModel.lowerLinkLength / 2.0);
      swingTime.set(0.3);
      scaleForVelToAngle.set(0.3);
      feedForwardGain.set(0.1);
      stepToStepHipAngleDelta.set(0.3);
      maxVelocityErrorAngle.set(1);
      alphaFilterVariableX.set(0.9999);
      alphaFilterVariableY.set(0.9999);

      stateMachine = initializeStateMachine();
   }

   @Override
   public void initialize()
   {

   }

   private StateMachine<ControllerState, State> initializeStateMachine()
   {
      StateMachineFactory<ControllerState, State> factory = new StateMachineFactory<>(ControllerState.class);
      factory.setNamePrefix("controllerState").setRegistry(registry).buildYoClock(scsRobot.getYoTime());
      factory.addStateAndDoneTransition(ControllerState.START, new

            StartState(), ControllerState.RIGHT_SUPPORT);
      factory.addStateAndDoneTransition(ControllerState.RIGHT_SUPPORT, new SingleSupportState(RobotSide.RIGHT), ControllerState.LEFT_SUPPORT);
      factory.addStateAndDoneTransition(ControllerState.LEFT_SUPPORT, new SingleSupportState(RobotSide.LEFT), ControllerState.RIGHT_SUPPORT);

      return factory.build(ControllerState.START);
   }

   private class StartState implements State
   {
      @Override
      public void doAction(double timeInState)
      {
      }

      @Override
      public boolean isDone(double timeInState)
      {
         return true;
      }

      @Override
      public void onEntry()
      {
      }

      @Override
      public void onExit()
      {
      }
   }

   private class SingleSupportState implements State
   {
      private RobotSide supportLeg;

      public SingleSupportState(RobotSide supportLeg)
      {
         this.supportLeg = supportLeg;
      }

      @Override
      public void doAction(double timeInState)
      {
         yoTimeInState.set(timeInState);

         /**
          * Part with all trajectories.
          */
         //Swing Leg
         if ((timeInState > swingTimeForThisStep.getDoubleValue() / 2.0) && !initalizedKneeExtension.getBooleanValue())
         {
            double currentKneePosition = robotModel.getKneePosition(swingLeg.getEnumValue());
            trajectorySwingKnee
                  .setParams(currentKneePosition, 0.0, 0.0, desiredKneeStance.getDoubleValue(), 0.0, 0.0, 0.0, swingTimeForThisStep.getDoubleValue() / 2.0);
            initalizedKneeExtension.set(true);
            kneeMoveStartTime.set(timeInState);
         }
         else if ((timeInState > swingTimeForThisStep.getDoubleValue() && !initalizedKneeDoubleExtension.getBooleanValue()))
         {
            double currentKneePosition = robotModel.getKneePosition(swingLeg.getEnumValue());
            trajectorySwingKnee.setParams(currentKneePosition, 0.0, 0.0, desiredKneeStance.getDoubleValue() + 0.5, 0.0, 0.0, 0.0, 0.125);
            initalizedKneeDoubleExtension.set(true);
            kneeMoveStartTime.set(timeInState);
         }

         trajectorySwingKnee.computeTrajectory(timeInState - kneeMoveStartTime.getDoubleValue());
         double desiredKneePosition = trajectorySwingKnee.getPosition();
         double desiredKneeVelocity = trajectorySwingKnee.getVelocity();
         controlKneeToPosition(swingLeg.getEnumValue(), desiredKneePosition, desiredKneeVelocity);

         desiredSwingLegHipPitchAngle.set(getDesiredHipPitchAngle());
         trajectorySwingHipPitch.setParams(startingHipPitchAngle.getDoubleValue(), 0.0, 0.0, desiredSwingLegHipPitchAngle.getDoubleValue(), 0.0, 0.0, 0.0,
                                           swingTimeForThisStep.getDoubleValue());

         desiredSwingLegHipRollAngle.set(getDesiredHipRollAngle());
         trajectorySwingHipRoll.setParams(startingHipRollAngle.getDoubleValue(), 0.0, 0.0, desiredSwingLegHipRollAngle.getDoubleValue(), 0.0, 0.0, 0.0,

                                          swingTimeForThisStep.getDoubleValue());

         /**
          * Control part.
          */
         // pitch

         trajectorySwingHipPitch.computeTrajectory(timeInState);
         double desiredHipPitchAngle = trajectorySwingHipPitch.getPosition();
         double currentHipPitchAngle = robotModel.getHipPitchPosition(swingLeg.getEnumValue());
         double currentHipPitchAngleRate = robotModel.getHipPitchVelocity(swingLeg.getEnumValue());

         PIDController pidControllerPitch = hipPitchControllers.get(swingLeg.getEnumValue());
         double controlEffortPitch = pidControllerPitch.compute(currentHipPitchAngle, desiredHipPitchAngle, currentHipPitchAngleRate, 0.0, deltaT);
         robotModel.setHipPitchTorque(swingLeg.getEnumValue(), controlEffortPitch);

         // roll
         trajectorySwingHipRoll.computeTrajectory(timeInState);
         double desiredHipRollAngle = trajectorySwingHipRoll.getPosition();
         double currentHipRollAngle = robotModel.getHipRollPosition(swingLeg.getEnumValue());
         double currentHipRollAngleRate = robotModel.getHipRollVelocity(swingLeg.getEnumValue());

         PIDController pidControllerRoll = hipRollControllers.get(swingLeg.getEnumValue());
         double controlEffortRoll = pidControllerRoll.compute(currentHipRollAngle, desiredHipRollAngle, currentHipRollAngleRate, 0.0, deltaT);
         robotModel.setHipRollTorque(swingLeg.getEnumValue(), controlEffortRoll);

         //foot

            PIDController pidControllerAnkleSwingPitch = anklePitchControllers.get(swingLeg.getEnumValue());
            PIDController pidControllerAnkleSwingRoll = ankleRollControllers.get(swingLeg.getEnumValue());

            double controlEffortAnkleSwingRoll = pidControllerAnkleSwingRoll
                  .compute(robotModel.getAnkleRollPosition(swingLeg.getEnumValue()), 0.0, 0.0, 0.0, deltaT);
            robotModel.setAnkleRollTorque(swingLeg.getEnumValue(), -controlEffortAnkleSwingRoll);

            double controlEffortAnkleSwingPitch = pidControllerAnkleSwingPitch
                  .compute(robotModel.getAnklePitchPosition(swingLeg.getEnumValue()), 0.0, 0.0, 0.0, deltaT);
            robotModel.setAnklePitchTorque(swingLeg.getEnumValue(), controlEffortAnkleSwingPitch);

            addCoPRollControl(supportLeg);
            addCoPPitchControl(supportLeg);


         //Stance leg
         controlHipToMaintainPitch(supportLeg);
         controlHipToMaintainRoll(supportLeg);

         //add swing leg torque to stand leg
         addOppositeLegHipPitchTorque(supportLeg);
         addOppositeLegHipRollTorque(supportLeg);

         addInertiaControlPitch(supportLeg);
         addInertiaControlRoll(supportLeg);

         // controlKneeToMaintainBodyHeight(supportLeg);
         controlKneeToPosition(supportLeg, desiredKneeStance.getDoubleValue(), 0.0);




      }

      @Override
      public boolean isDone(double timeInState)
      {
         return initalizedKneeExtension.getBooleanValue() && robotModel.isFootOnGround(swingLeg.getEnumValue());
      }

      @Override
      public void onEntry()
      {
         scsRobot.updateJointPositions_ID_to_SCS();
         scsRobot.updateJointVelocities_ID_to_SCS();
         scsRobot.updateJointTorques_ID_to_SCS();
         scsRobot.update();
         robotModel.updateFrames();

         swingLeg.set(supportLeg.getOppositeSide());
         swingTimeForThisStep.set(swingTime.getDoubleValue());
         initalizedKneeExtension.set(false);
         initalizedKneeDoubleExtension.set(false);
         kneeMoveStartTime.set(0.0);

         startingHipPitchAngle.set(robotModel.getHipPitchPosition(swingLeg.getEnumValue()));
         startingHipRollAngle.set(robotModel.getHipRollPosition(swingLeg.getEnumValue()));

         double currentKneePosition = robotModel.getKneePosition(swingLeg.getEnumValue());
         double desiredRetractedPosition = 0.1;
         trajectorySwingKnee.setParams(currentKneePosition, 0.0, 0.0, desiredRetractedPosition, 0.0, 0.0, 0.0, swingTimeForThisStep.getDoubleValue() / 2.0);

         //retract knee
         robotModel.setKneeTorque(swingLeg.getEnumValue(), -10.0);
      }

      @Override
      public void onExit()
      {
         scsRobot.updateJointPositions_ID_to_SCS();
         scsRobot.updateJointVelocities_ID_to_SCS();
         scsRobot.updateJointTorques_ID_to_SCS();
         scsRobot.update();
         robotModel.updateFrames();

         lastStepHipPitchAngle.set(desiredSwingLegHipPitchAngle.getDoubleValue());
         lastStepHipRollAngle.set(desiredSwingLegHipRollAngle.getDoubleValue());
      }
   }

   private double getDesiredHipPitchAngle()
   {
      double legLength = robotModel.upperLinkLength + desiredKneeStance.getDoubleValue();
      pitchAngleForCapture.set(HipAngleCapturePointCalculator3D.getHipPitchAngle(robotModel.getBodyVelocityX(), legLength));
      pitchAngleForCapture.set(-pitchAngleForCapture.getDoubleValue() * Math.signum(robotModel.getBodyVelocityX()));

      //only use a fraction of it
      //angleForCapture.set(0.8 *angleForCapture.getDoubleValue()
      // );

      //limit this angle
      pitchAngleForCapture.set(MathTools.clamp(pitchAngleForCapture.getDoubleValue(), MAX_HIP_ANGLE));

      //angle is opposite sign of desired velocity
      double velocityError = (filteredDesiredVelocityX.getDoubleValue() - robotModel.getBodyVelocityX());
      velocityErrorPitchAngle.set(velocityError * scaleForVelToAngle.getDoubleValue());
      velocityErrorPitchAngle.set(MathTools.clamp(velocityErrorPitchAngle.getDoubleValue(), maxVelocityErrorAngle.getDoubleValue()));

      feedForwardAngle.set(filteredDesiredVelocityX.getDoubleValue() * feedForwardGain.getDoubleValue());
      double angle = pitchAngleForCapture.getDoubleValue() + feedForwardAngle.getDoubleValue() + velocityErrorPitchAngle.getDoubleValue();

      angle = MathTools.clamp(angle, pitchAngleForCapture.getDoubleValue() - stepToStepHipAngleDelta.getDoubleValue(),
                              pitchAngleForCapture.getDoubleValue() + stepToStepHipAngleDelta.getDoubleValue());
      return angle;
   }

   private double getDesiredHipRollAngle()
   {
      double legLength = robotModel.upperLinkLength + desiredKneeStance.getDoubleValue();
      rollAngleForCapture.set(HipAngleCapturePointCalculator3D.getHipRollAngle(robotModel.getBodyVelocityY(), legLength));
      rollAngleForCapture.set(rollAngleForCapture.getDoubleValue() * Math.signum(robotModel.getBodyVelocityY()));

      //only use a fraction of it
      //angleForCapture.set(0.8 *angleForCapture.getDoubleValue());

      //limit this angle
      rollAngleForCapture.set(MathTools.clamp(rollAngleForCapture.getDoubleValue(), MAX_HIP_ANGLE));

      //angle is opposite sign of desired velocity
      double velocityError = (filteredDesiredVelocityY.getDoubleValue() - robotModel.getBodyVelocityY());
      velocityErrorRollAngle.set(-velocityError * scaleForVelToAngle.getDoubleValue());
      velocityErrorRollAngle.set(MathTools.clamp(velocityErrorRollAngle.getDoubleValue(), maxVelocityErrorAngle.getDoubleValue()));

      feedForwardAngle.set(-filteredDesiredVelocityY.getDoubleValue() * feedForwardGain.getDoubleValue());
      double angle = rollAngleForCapture.getDoubleValue() + feedForwardAngle.getDoubleValue() + velocityErrorRollAngle.getDoubleValue();

      angle = MathTools.clamp(angle, rollAngleForCapture.getDoubleValue() - stepToStepHipAngleDelta.getDoubleValue(),
                              rollAngleForCapture.getDoubleValue() + stepToStepHipAngleDelta.getDoubleValue());
      return angle;

   }

   private void controlHipToMaintainPitch(RobotSide robotSide)
   {
      double currentPitch = robotModel.getBodyPitch();
      double currentPitchRate = robotModel.getBodyPitchVelocity();

      double controlEffort = -hipPitchControllers.get(robotSide).compute(currentPitch, desiredPitch.getDoubleValue(), currentPitchRate, 0.0, deltaT);
      robotModel.setHipPitchTorque(robotSide, controlEffort);
   }

   private void controlHipToMaintainRoll(RobotSide robotSide)
   {
      double currentRoll = robotModel.getBodyRoll();
      double currentRollRate = robotModel.getBodyRollVelocity();

      double controlEffort = -hipRollControllers.get(robotSide).compute(currentRoll, desiredRoll.getDoubleValue(), currentRollRate, 0.0, deltaT);
      robotModel.setHipRollTorque(robotSide, controlEffort);
   }

   private void controlHipToMaintainYaw(RobotSide robotSide)
   {
      double currentYaw = robotModel.getBodyYaw();
      double currentYawRate = robotModel.getBodyYawVelocity();

      double controlEffort = -hipYawControllers.get(robotSide).compute(currentYaw, 0.0, currentYawRate, 0.0, deltaT);
      robotModel.setHipYawTorque(robotSide, controlEffort);
   }

   private void addOppositeLegHipPitchTorque(RobotSide legToAddTorque)
   {
      double oppositeLegTorque = robotModel.getHipPitchTorque(legToAddTorque.getOppositeSide());
      double currentTorque = robotModel.getHipPitchTorque(legToAddTorque);
      robotModel.setHipPitchTorque(legToAddTorque, currentTorque - oppositeLegTorque);
   }

   private void addOppositeLegHipRollTorque(RobotSide legToAddTorque)
   {
      double oppositeLegTorque = robotModel.getHipRollTorque(legToAddTorque.getOppositeSide());
      double currentTorque = robotModel.getHipRollTorque(legToAddTorque);
      robotModel.setHipRollTorque(legToAddTorque, currentTorque - oppositeLegTorque);
   }

   private void addInertiaControlPitch(RobotSide robotSide)
   {
      double tauHipInertia = -1 * angularMomentumPitchControllers.get(robotSide).compute(robotModel.getBodyVelocityX(), filteredDesiredVelocityX.getDoubleValue(),
                                                                                         robotModel.getBodyAccelerationX(), 0.0, deltaT);
      double currentTorque = robotModel.getHipPitchTorque(robotSide);
      robotModel.setHipPitchTorque(robotSide, currentTorque + tauHipInertia);
   }

   private void addInertiaControlRoll(RobotSide robotSide)
   {
      double tauHipInertia = 1 * angularMomentumRollControllers.get(robotSide).compute(robotModel.getBodyVelocityY(), filteredDesiredVelocityY.getDoubleValue(),
                                                                                       robotModel.getBodyAccelerationY(), 0.0, deltaT);
      double currentTorque = robotModel.getHipRollTorque(robotSide);
      robotModel.setHipRollTorque(robotSide, currentTorque + tauHipInertia);
   }

   private void addCoPPitchControl(RobotSide robotSide)
   {

      double anklePositionX = robotModel.getBodyPositionX() - Math.tan(robotModel.getHipPitchPosition(robotSide)) / robotModel.getZ0();
      double omega0sq = -robotModel.getGravity() / robotModel.getZ0();
      double CoPX = (omega0sq * robotModel.getBodyPositionX() - robotModel.getBodyAccelerationX()) / omega0sq;

      double dCoPX = -anklePitchControllers.get(robotSide)
                                           .compute(robotModel.getBodyVelocityX(), filteredDesiredVelocityX.getDoubleValue(), 0.0 * robotModel.getBodyAccelerationX(), 0.0,
                                                    deltaT);
      coPLocationX.set(CoPX - anklePositionX);
      if (coPLocationX.getDoubleValue() < robotModel.getFootSizeX())
      {
         double tauAnkle = dCoPX * Math.cos(robotModel.getHipPitchPosition(robotSide)) * robotModel.getKneeTorque(robotSide);
         ankleControlTorquePitch.set(MathTools.clamp(tauAnkle, -0.4 * robotModel.getFootSizeX() * robotModel.getBodyMass() * robotModel.getGravity()));
         robotModel.setAnklePitchTorque(robotSide, ankleControlTorquePitch.getDoubleValue());
      }
   }

   private void addCoPRollControl(RobotSide robotSide)
   {

      double anklePositionY = robotModel.getBodyPositionY() + Math.tan(robotModel.getHipRollPosition(robotSide)) / robotModel.getZ0();
      double omega0sq = -robotModel.getGravity() / robotModel.getZ0();
      double CoPY = (omega0sq * robotModel.getBodyPositionY() - robotModel.getBodyAccelerationY()) / omega0sq;

      double dCoPY = ankleRollControllers.get(robotSide)
                                         .compute(robotModel.getBodyVelocityY(), filteredDesiredVelocityY.getDoubleValue(), 0.0 * robotModel.getBodyAccelerationY(), 0.0,
                                                  deltaT);
      coPLocationY.set(CoPY - anklePositionY);
      if (coPLocationY.getDoubleValue() < robotModel.getFootSizeY())
      {
         double tauAnkle = dCoPY * Math.cos(robotModel.getHipRollPosition(robotSide)) * robotModel.getKneeTorque(robotSide);
         ankleControlTorqueRoll.set(MathTools.clamp(tauAnkle, -0.4 * robotModel.getFootSizeY() * robotModel.getBodyMass() * robotModel.getGravity()));
         robotModel.setAnkleRollTorque(robotSide, ankleControlTorqueRoll.getDoubleValue());
      }
   }

   private void addAnkleYawTorque(RobotSide robotSide)
   {
      double torque =
            ((robotModel.getHipRollTorque(robotSide) - robotModel.getAnkleRollTorque(robotSide)) / robotModel.getZ0()) * (Math.tan(
                  robotModel.getHipPitchPosition(robotSide))
                  / robotModel.getZ0()) + ((robotModel.getHipPitchTorque(robotSide) - robotModel.getAnklePitchTorque(robotSide)) / robotModel.getZ0()) * (
                  Math.tan(robotModel.getHipRollPosition(robotSide)) / robotModel.getZ0()) + robotModel.getHipYawTorque(robotSide);
      robotModel.setAnkleYawTorque(robotSide, torque);
   }

   private void controlKneeToMaintainBodyHeight(RobotSide robotSide)
   {
      double currentHeight = robotModel.getBodyHeight();
      double currentHeightRate = robotModel.getBodyHeightVelocity();

      double controlEffort = kneeControllers.get(robotSide).compute(currentHeight, desiredHeight.getDoubleValue(), currentHeightRate, 0.0, deltaT);
      robotModel.setKneeTorque(robotSide, controlEffort);
   }

   private void controlKneeToPosition(RobotSide robotSide, double desiredPosition, double desiredVelocity)
   {
      double kneePosition = robotModel.getKneePosition(robotSide);
      double kneePositionRate = robotModel.getKneeVelocity(robotSide);

      double controlEffort = kneeControllers.get(robotSide).compute(kneePosition, desiredPosition, kneePositionRate, desiredVelocity, deltaT);
      robotModel.setKneeTorque(robotSide, controlEffort);
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return null;
   }

   @Override
   public String getDescription()
   {
      return null;
   }

   @Override
   public void doControl()
   {
      //      for (RobotSide robotSide : RobotSide.values)
      //      {
      //         double currentPosition = robotModel.getKneePosition(robotSide);
      //         double currentVelocity = robotModel.getKneeVelocity(robotSide);
      //
      //         double effort = kneeControllers.get(robotSide).compute(currentPosition, desiredKneeExtension.getDoubleValue(), currentVelocity, 0.0, deltaT);
      //         robotModel.setKneeTorque(robotSide, effort);
      //      }

      scsRobot.updateJointPositions_SCS_to_ID();
      scsRobot.updateJointVelocities_SCS_to_ID();

      robotModel.updateFrames();

      filteredDesiredVelocityX.update();
      filteredDesiredVelocityY.update();

      centerOfMassPosition.set(robotModel.getBodyPositionX(), robotModel.getBodyPositionY(), robotModel.getBodyHeight());
      centerOfMassVelocity.set(robotModel.getBodyVelocityX(), robotModel.getBodyVelocityY(), robotModel.getBodyHeightVelocity());
      centerOfMassAcceleration.set(robotModel.getBodyAccelerationX(), robotModel.getBodyAccelerationY(), 0.0);
      centerOfMassPosition2D.set(robotModel.getBodyPositionX(), robotModel.getBodyPositionY());

      omega.set(Math.sqrt(-robotModel.getGravity() / robotModel.getBodyHeight()));

      updateCapturePointEstimate();
      updateCenterOfPressureEstimate();

      stateMachine.doActionAndTransition();

      scsRobot.updateJointTorques_ID_to_SCS();
      scsRobot.update();


   }

   private void updateCapturePointEstimate()
   {


      desiredICP.set(filteredDesiredVelocityX.getDoubleValue(), filteredDesiredVelocityY.getDoubleValue(), desiredHeight.getDoubleValue());
      desiredICP.scale(1.0 / omega.getDoubleValue());
      desiredICP.add(centerOfMassPosition);

      currentICP.set(centerOfMassVelocity);
      currentICP.scale(1.0 / omega.getDoubleValue());
      currentICP.add(centerOfMassPosition);
      currentICP2D.set(currentICP.getX(),currentICP.getY());
   }

   private void updateCenterOfPressureEstimate()
   {
      currentCOP.set(centerOfMassAcceleration);
      currentCOP.scale(-1.0 / (Math.pow(omega.getDoubleValue(), 2.0)));
      currentCOP.add(centerOfMassPosition);
      currentCOP2D.set(currentCOP.getX(),currentCOP.getY());
   }


   public YoGraphicsListRegistry getYoGraphicsListRegistry()
   {
      return yoGraphicsListRegistry;
   }

   public Artifact getCurrentCoPGraphicArtifact()
   {
      return currentCoPGraphicArtifact;
   }

   public Artifact getCurrentCoMGraphicArtifact()
   {
      return currentCoMGraphicArtifact;
   }

   public Artifact getCurrentICPGraphicArtifact()
   {
      return currentICPGraphicArtifact;
   }

   public enum ControllerState
   {
      START, LEFT_SUPPORT, RIGHT_SUPPORT;
   }

}
