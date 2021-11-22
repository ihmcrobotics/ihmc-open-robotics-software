package us.ihmc.exampleSimulations.springflamingo;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.math.trajectories.yoVariables.YoPolynomial;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class SpringFlamingoLeapOfFaithController implements RobotController
{
   private final YoRegistry registry = new YoRegistry(getName());
   private final SpringFlamingoRobot robot;

   private final YoDouble capturePointToStartSwing = new YoDouble("capturePointToStartSwing", registry);

   private final SideDependentList<YoDouble> thighAngles = new SideDependentList<>();
   private final SideDependentList<YoDouble> thighVelocities = new SideDependentList<>();

   private final SideDependentList<YoDouble> q_d_hips = new SideDependentList<>();
   private final SideDependentList<YoDouble> q_d_knees = new SideDependentList<>();
   private final SideDependentList<YoDouble> q_d_ankles = new SideDependentList<>();

   public final YoDouble capturePointX = new YoDouble("capturePointX", registry);
   public final YoDouble capturePointLeftHeelX = new YoDouble("capturePointLeftHeelX", registry);
   public final YoDouble capturePointRightHeelX = new YoDouble("capturePointRightHeelX", registry);

   private final SideDependentList<YoDouble> capturePointHeelsX = new SideDependentList<>(capturePointLeftHeelX, capturePointRightHeelX);

   private final YoDouble minimumLoadingDuration = new YoDouble("minimumLoadingDuration", registry);

   private final YoDouble q_d_pitch = new YoDouble("q_d_pitch", registry);
   private final YoDouble bodyOrientationKp = new YoDouble("bodyOrientationKp", registry);
   private final YoDouble bodyOrientationKd = new YoDouble("bodyOrientationKd", registry);

   private final YoDouble kneeSupportKp = new YoDouble("kneeSupportKp", registry);
   private final YoDouble kneeSupportKd = new YoDouble("kneeSupportKd", registry);

   private final YoDouble kneeSwingKp = new YoDouble("kneeSwingKp", registry);
   private final YoDouble kneeSwingKd = new YoDouble("kneeSwingKd", registry);

   private final YoDouble ankleToeOffKp = new YoDouble("ankleToeOffKp", registry);
   private final YoDouble ankleToeOffKd = new YoDouble("ankleToeOffKd", registry);

   private final YoDouble ankleSwingKp = new YoDouble("ankleSwingKp", registry);
   private final YoDouble ankleSwingKd = new YoDouble("ankleSwingKd", registry);

   private final YoDouble minimumKneeSupportForce = new YoDouble("minimumKneeSupportForce", registry);

   private final YoDouble thighSwingKp = new YoDouble("thighSwingKp", registry);
   private final YoDouble thighSwingKd = new YoDouble("thighSwingKd", registry);

   private final YoDouble previousTickTime = new YoDouble("previousTickTime", registry);
   private final YoDouble deltaTime = new YoDouble("deltaTime", registry);
   private final YoDouble maxToeOffAnkleAngle = new YoDouble("maxToeOffAnkleAngle", registry);

   private final YoDouble totalMass = new YoDouble("totalMass", registry);
   private final YoDouble minimumKneeForce = new YoDouble("minimumKneeForce", registry);
   private final YoDouble maximumKneeForce = new YoDouble("maximumKneeForce", registry);

   private final YoDouble capturePointXToStartBraking = new YoDouble("capturePointXToStartBraking", registry);
   private final YoDouble ankleBrakingKd = new YoDouble("ankleBrakingKd", registry);
   private final YoDouble maxAnkleBrakingTorque = new YoDouble("maxAnkleBrakingTorque", registry);

   private final YoDouble toeOffAnkleTorque = new YoDouble("toeOffAnkleTorque", registry);
   private final YoDouble toeOffAnkleAcceleration = new YoDouble("toeOffAnkleAcceleration", registry);
   private final YoDouble toeOffAnkleVelocity = new YoDouble("toeOffAnkleVelocity", registry);

   private final YoDouble dropSupportKneeAcceleration = new YoDouble("dropSupportKneeAcceleration", registry);
   private final YoDouble dropSupportKneeVelocity = new YoDouble("dropSupportKneeVelocity", registry);

   private final YoDouble finalSwingThighAngle = new YoDouble("finalSwingThighAngle", registry);
   private final YoDouble swingDuration = new YoDouble("swingDuration", registry);

   private final YoDouble finalRetractThighAngle = new YoDouble("finalRetractThighAngle", registry);
   private final YoDouble retractDuration = new YoDouble("retractDuration", registry);

   private final YoDouble swingTime = new YoDouble("swingTime", registry);
   private final YoDouble collapseKneeForSwingVelocity = new YoDouble("collapseKneeForSwingVelocity", registry);
   private final YoDouble collapseKneeForSwingAngle = new YoDouble("collapseKneeForSwingAngle", registry);
   private final YoDouble straightenKneeForSwingVelocity = new YoDouble("straightenKneeForSwingVelocity", registry);
   private final YoDouble kneeEndSwingAngle = new YoDouble("kneeEndSwingAngle", registry);

   private final YoDouble extendKneeDuringStanceVelocity = new YoDouble("extendKneeDuringStanceVelocity", registry);

   private final YoDouble minimumSupportKneeAngle = new YoDouble("minimumSupportKneeAngle", registry);
   private final YoDouble maximumSupportKneeAngle = new YoDouble("maximumSupportKneeAngle", registry);

   private final StateMachine<LeapOfFaithState, State> stateMachine;

   public SpringFlamingoLeapOfFaithController(SpringFlamingoRobot robot)
   {
      this.robot = robot;

      for (RobotSide robotSide : RobotSide.values)
      {
         YoDouble thighAngle = new YoDouble(robotSide.getCamelCaseName() + "ThighAngle", registry);
         YoDouble thighVelocity = new YoDouble(robotSide.getCamelCaseName() + "ThighVelocity", registry);

         YoDouble q_d_hip = new YoDouble("q_d_" + robotSide.getCamelCaseNameForMiddleOfExpression() + "Hip", registry);
         YoDouble q_d_knee = new YoDouble("q_d_" + robotSide.getCamelCaseNameForMiddleOfExpression() + "Knee", registry);
         YoDouble q_d_ankle = new YoDouble("q_d_" + robotSide.getCamelCaseNameForMiddleOfExpression() + "Ankle", registry);

         thighAngles.set(robotSide, thighAngle);
         thighVelocities.set(robotSide, thighVelocity);

         q_d_hips.set(robotSide, q_d_hip);
         q_d_knees.set(robotSide, q_d_knee);
         q_d_ankles.set(robotSide, q_d_ankle);
      }

      Point3D comPoint = new Point3D();
      totalMass.set(robot.computeCenterOfMass(comPoint));

      capturePointXToStartBraking.set(0.14);
      ankleBrakingKd.set(60.0);
      maxAnkleBrakingTorque.set(-10.0);
      toeOffAnkleTorque.set(-14.0);
      capturePointToStartSwing.set(0.14);

      minimumLoadingDuration.set(0.04);

      finalSwingThighAngle.set(0.8);
      swingDuration.set(0.4);
      swingTime.set(0.4);
      finalRetractThighAngle.set(0.38);
      retractDuration.set(0.3);

      kneeEndSwingAngle.set(-0.2);

      toeOffAnkleAcceleration.set(4.0);
      collapseKneeForSwingVelocity.set(5.0);
      collapseKneeForSwingAngle.set(1.5);
      straightenKneeForSwingVelocity.set(8.0);
      extendKneeDuringStanceVelocity.set(0.6);
      dropSupportKneeAcceleration.set(12.0);

      minimumSupportKneeAngle.set(0.0);
      maximumSupportKneeAngle.set(1.2);
      maxToeOffAnkleAngle.set(1.1);

      bodyOrientationKp.set(100.0);
      bodyOrientationKd.set(12.0);

      kneeSupportKp.set(200.0);
      kneeSupportKd.set(5.0);

      kneeSwingKp.set(200.0);
      kneeSwingKd.set(4.0);

      thighSwingKp.set(200.0);
      thighSwingKd.set(4.0);

      ankleToeOffKp.set(30.0);
      ankleToeOffKd.set(1.0);

      ankleSwingKp.set(10.0);
      ankleSwingKd.set(1.0);

      minimumKneeSupportForce.set(2.0);

      minimumKneeForce.set(-totalMass.getDoubleValue() * 9.81 * 0.2);
      maximumKneeForce.set(totalMass.getDoubleValue() * 9.81 * 1.1);

      stateMachine = setupStateMachine(robot);

      initControl();
      computeThighAngles();
      computeCapturePoint();
   }

   private StateMachine<LeapOfFaithState, State> setupStateMachine(SpringFlamingoRobot robot)
   {
      // Create the factory:
      StateMachineFactory<LeapOfFaithState, State> leftFactory = new StateMachineFactory<>(LeapOfFaithState.class);
      leftFactory.setNamePrefix("state");
      leftFactory.setRegistry(registry);
      leftFactory.buildYoClock(robot.t);

      // Create the states:
      LoadingToeOffState rightLoadingLeftToeOffState = new LoadingToeOffState(LeapOfFaithState.RightLoadingLeftToeOff, LeapOfFaithState.RightSupportLeftSwing);

      SupportSwingState rightSupportLeftSwingState = new SupportSwingState(LeapOfFaithState.RightSupportLeftSwing,
                                                                           LeapOfFaithState.RightDropLeftRetract,
                                                                           registry);

      DropRetractState rightDropLeftRetractState = new DropRetractState(LeapOfFaithState.RightDropLeftRetract,
                                                                        LeapOfFaithState.LeftLoadingRightToeOff,
                                                                        registry);

      LoadingToeOffState leftLoadingRightToeOffState = new LoadingToeOffState(LeapOfFaithState.LeftLoadingRightToeOff, LeapOfFaithState.LeftSupportRightSwing);

      SupportSwingState leftSupportRightSwingState = new SupportSwingState(LeapOfFaithState.LeftSupportRightSwing,
                                                                           LeapOfFaithState.LeftDropRightRetract,
                                                                           registry);

      DropRetractState leftDropRightRetractState = new DropRetractState(LeapOfFaithState.LeftDropRightRetract,
                                                                        LeapOfFaithState.RightLoadingLeftToeOff,
                                                                        registry);

      // Add the states:
      leftFactory.addState(LeapOfFaithState.RightLoadingLeftToeOff, rightLoadingLeftToeOffState);
      leftFactory.addState(LeapOfFaithState.RightSupportLeftSwing, rightSupportLeftSwingState);
      leftFactory.addState(LeapOfFaithState.RightDropLeftRetract, rightDropLeftRetractState);
      leftFactory.addState(LeapOfFaithState.LeftLoadingRightToeOff, leftLoadingRightToeOffState);
      leftFactory.addState(LeapOfFaithState.LeftSupportRightSwing, leftSupportRightSwingState);
      leftFactory.addState(LeapOfFaithState.LeftDropRightRetract, leftDropRightRetractState);

      // Add the transition conditions:
      leftFactory.addDoneTransition(LeapOfFaithState.RightLoadingLeftToeOff, LeapOfFaithState.RightSupportLeftSwing);
      leftFactory.addDoneTransition(LeapOfFaithState.RightSupportLeftSwing, LeapOfFaithState.RightDropLeftRetract);
      leftFactory.addDoneTransition(LeapOfFaithState.RightDropLeftRetract, LeapOfFaithState.LeftLoadingRightToeOff);

      leftFactory.addDoneTransition(LeapOfFaithState.LeftLoadingRightToeOff, LeapOfFaithState.LeftSupportRightSwing);
      leftFactory.addDoneTransition(LeapOfFaithState.LeftSupportRightSwing, LeapOfFaithState.LeftDropRightRetract);
      leftFactory.addDoneTransition(LeapOfFaithState.LeftDropRightRetract, LeapOfFaithState.RightLoadingLeftToeOff);

      return leftFactory.build(LeapOfFaithState.RightDropLeftRetract);
   }

   private void initControl()
   {
      // Initialize the variables.
      robot.initializeForBallisticWalking();

      //      vel.set(0.0);
      //
      //      left_force.set(0.0);
      //      left_cop.set(0.5);
      //      right_force.set(131.83);
      //      right_cop.set(0.557913);
      //      left_hip_set.set(0.358627);
      //
      //      left_heel.set(-0.392589);
      //      right_hip_set.set(0.358627);
      //
      //      right_heel.set(-0.231515);
      //      max_hip_torque.set(0.0);
      //
      //      DefaultParameterReader defaultParameterReader = new DefaultParameterReader();
      //      defaultParameterReader.readParametersInRegistry(registry);
   }

   private class LoadingToeOffState implements State
   {
      private RobotSide loadingSide, toeOffSide;

      public LoadingToeOffState(LeapOfFaithState stateEnum, LeapOfFaithState nextStateEnum)
      {
         loadingSide = stateEnum.getLoadingSide();
         toeOffSide = stateEnum.getToeOffSide();
      }

      @Override
      public void onEntry()
      {
         q_d_knees.get(loadingSide).set(robot.getKneeAngle(loadingSide));
         q_d_knees.get(toeOffSide).set(robot.getKneeAngle(toeOffSide));

         q_d_ankles.get(toeOffSide).set(0.0); //robot.getAnkleAngle(toeOffSide) - 0.1);
         q_d_ankles.get(loadingSide).set(0.0);

         toeOffAnkleVelocity.set(0.0);
      }

      @Override
      public boolean isDone(double timeInState)
      {
         if (-capturePointHeelsX.get(loadingSide).getDoubleValue() > capturePointToStartSwing.getDoubleValue())
         {
            if (timeInState > minimumLoadingDuration.getValue())
            {
               return true;
            }
         }

         return false;
      }

      @Override
      public void doAction(double timeInState)
      {
         double supportHipTorque = bodyOrientationKp.getDoubleValue() * (q_d_pitch.getDoubleValue() - robot.getBodyAngle())
               - bodyOrientationKd.getDoubleValue() * robot.getBodyAngularVelocity();
         Vector3D loadingGroundForce = robot.getFootForce(loadingSide);
         Vector3D toeOffGroundForce = robot.getFootForce(toeOffSide);

         double loadingMagnitude = loadingGroundForce.getZ();
         double toeOffMagnitude = toeOffGroundForce.getZ();

         if (loadingMagnitude < 0.0)
            loadingMagnitude = 0.0;
         if (toeOffMagnitude < 0.0)
            toeOffMagnitude = 0.0;
         double totalMagnitude = loadingMagnitude + toeOffMagnitude;

         if (totalMagnitude > totalMass.getDoubleValue() * 9.81 * 0.05)
         {
            double percentLoading = loadingMagnitude / totalMagnitude;
            double percentToeOff = toeOffMagnitude / totalMagnitude;

            robot.setHipTorque(loadingSide, -supportHipTorque * percentLoading);
            robot.setHipTorque(toeOffSide, -supportHipTorque * percentToeOff);
         }

         YoDouble toeOffAnkleDesiredAngle = q_d_ankles.get(toeOffSide);

         toeOffAnkleVelocity.add(deltaTime.getDoubleValue() * toeOffAnkleAcceleration.getDoubleValue());
         toeOffAnkleDesiredAngle.sub(deltaTime.getDoubleValue() * toeOffAnkleVelocity.getDoubleValue());

         if (toeOffAnkleDesiredAngle.getDoubleValue() < -maxToeOffAnkleAngle.getDoubleValue())
         {
            toeOffAnkleDesiredAngle.set(-maxToeOffAnkleAngle.getDoubleValue());
         }

         double loadingKneeTorque = kneeSupportKp.getDoubleValue() * (q_d_knees.get(loadingSide).getDoubleValue() - robot.getKneeAngle(loadingSide))
               - kneeSupportKd.getDoubleValue() * robot.getKneeVelocity(loadingSide);
         robot.setKneeTorque(loadingSide, loadingKneeTorque);

         if (loadingKneeTorque < minimumKneeForce.getDoubleValue())
         {
            loadingKneeTorque = minimumKneeForce.getDoubleValue();
         }

         double toeOffKneeTorque = kneeSupportKp.getDoubleValue() * (q_d_knees.get(toeOffSide).getDoubleValue() - robot.getKneeAngle(toeOffSide))
               - kneeSupportKd.getDoubleValue() * robot.getKneeVelocity(toeOffSide);

         //         if (toeOffKneeTorque < minimumKneeSupportForce.getDoubleValue())
         //         {
         //            toeOffKneeTorque = minimumKneeSupportForce.getDoubleValue();
         //            if (q_d_knees.get(toeOffSide).getDoubleValue() < robot.getKneeAngle(toeOffSide))
         //            {
         //               q_d_knees.get(toeOffSide).set(robot.getKneeAngle(toeOffSide));
         //            }
         //         }

         robot.setKneeTorque(toeOffSide, toeOffKneeTorque);

         //         toeOffAnkleTorque.set(ankleToeOffKp.getDoubleValue() * (q_d_ankles.get(toeOffSide).getDoubleValue() - robot.getAnkleAngle(toeOffSide))
         //               - ankleToeOffKd.getDoubleValue() * robot.getAnkleVelocity(toeOffSide));

         robot.setAnkleTorque(toeOffSide, toeOffAnkleTorque.getDoubleValue());
         doBrakingOnSupportAnkleIfTooFast(loadingSide);

         robot.setAnkleTorque(loadingSide, 0.0);
      }
   };

   private class SupportSwingState implements State
   {
      private RobotSide supportSide, swingSide;
      private final YoPolynomial swingTrajectory;
      private final YoDouble initialSwingThighAngle;
      private final YoDouble desiredSwingThighAngle;

      public SupportSwingState(LeapOfFaithState stateEnum, LeapOfFaithState nextStateEnum, YoRegistry parentRegistry)
      {
         supportSide = stateEnum.getSupportSide();
         swingSide = stateEnum.getSwingSide();

         String swingSideName = swingSide.getCamelCaseName();

         initialSwingThighAngle = new YoDouble(swingSideName + "InitialSwingThighAngle", parentRegistry);

         swingTrajectory = new YoPolynomial(swingSideName + "Swing", 4, parentRegistry);

         desiredSwingThighAngle = new YoDouble(swingSideName + "DesiredSwingThighAngle", parentRegistry);
      }

      @Override
      public void onEntry()
      {
         initialSwingThighAngle.set(thighAngles.get(swingSide).getDoubleValue());
         swingTrajectory.setCubic(0.0, swingDuration.getValue(), initialSwingThighAngle.getValue(), finalSwingThighAngle.getValue());

         swingTrajectory.initialize();

         q_d_knees.get(supportSide).set(robot.getKneeAngle(supportSide));
         q_d_knees.get(swingSide).set(robot.getKneeAngle(swingSide));
         q_d_ankles.get(swingSide).set(robot.getAnkleAngle(swingSide));
      }

      @Override
      public boolean isDone(double timeInState)
      {
         if (timeInState > swingTime.getDoubleValue())
         {
            return true;
         }

         return false;
      }

      @Override
      public void doAction(double timeInState)
      {
         double supportHipTorque = bodyOrientationKp.getDoubleValue() * (q_d_pitch.getDoubleValue() - robot.getBodyAngle())
               - bodyOrientationKd.getDoubleValue() * robot.getBodyAngularVelocity();
         robot.setHipTorque(supportSide, -supportHipTorque);

         swingTrajectory.compute(Math.min(timeInState, swingDuration.getValue()));
         desiredSwingThighAngle.set(swingTrajectory.getValue());

         double swingHipTorque = thighSwingKp.getDoubleValue() * (desiredSwingThighAngle.getDoubleValue() - thighAngles.get(swingSide).getDoubleValue())
               - thighSwingKd.getDoubleValue() * thighVelocities.get(swingSide).getDoubleValue();
         robot.setHipTorque(swingSide, swingHipTorque);

         q_d_knees.get(supportSide).add(deltaTime.getDoubleValue() * extendKneeDuringStanceVelocity.getDoubleValue());
         if (q_d_knees.get(supportSide).getDoubleValue() > 0.0)
         {
            q_d_knees.get(supportSide).set(0.0);
         }

         q_d_knees.get(swingSide).sub(deltaTime.getDoubleValue() * collapseKneeForSwingVelocity.getDoubleValue());
         if (q_d_knees.get(swingSide).getDoubleValue() < -collapseKneeForSwingAngle.getValue())
         {
            q_d_knees.get(swingSide).set(-collapseKneeForSwingAngle.getValue());
         }

         double supportKneeTorque = kneeSupportKp.getDoubleValue() * (q_d_knees.get(supportSide).getDoubleValue() - robot.getKneeAngle(supportSide))
               - kneeSupportKd.getDoubleValue() * robot.getKneeVelocity(supportSide);
         //         if (supportKneeTorque < minimumKneeSupportForce.getDoubleValue())
         //         {
         //            supportKneeTorque = minimumKneeSupportForce.getDoubleValue();
         //         }
         robot.setKneeTorque(supportSide, supportKneeTorque);

         double swingKneeTorque = kneeSwingKp.getDoubleValue() * (q_d_knees.get(swingSide).getDoubleValue() - robot.getKneeAngle(swingSide))
               - kneeSwingKd.getDoubleValue() * robot.getKneeVelocity(swingSide);
         robot.setKneeTorque(swingSide, swingKneeTorque);

         q_d_ankles.get(swingSide).add(deltaTime.getDoubleValue() * collapseKneeForSwingVelocity.getDoubleValue());
         if (q_d_ankles.get(swingSide).getDoubleValue() > 0.0)
         {
            q_d_ankles.get(swingSide).set(0.0);
         }

         double swingAnkleTorque = ankleSwingKp.getDoubleValue() * (q_d_ankles.get(swingSide).getDoubleValue() - robot.getAnkleAngle(swingSide))
               - ankleSwingKd.getDoubleValue() * robot.getAnkleVelocity(swingSide);

         robot.setAnkleTorque(swingSide, swingAnkleTorque);

         doBrakingOnSupportAnkleIfTooFast(supportSide);
      }

   };

   private void doBrakingOnSupportAnkleIfTooFast(RobotSide supportSide)
   {
      if (-capturePointHeelsX.get(supportSide).getValue() > capturePointXToStartBraking.getValue())
      {
         double ankleBrakingTorque = ankleBrakingKd.getValue() * (capturePointXToStartBraking.getValue() + capturePointHeelsX.get(supportSide).getValue());
         if (ankleBrakingTorque < maxAnkleBrakingTorque.getValue())
         {
            ankleBrakingTorque = maxAnkleBrakingTorque.getValue();
         }
         robot.setAnkleTorque(supportSide, ankleBrakingTorque);
      }
      else
      {
         robot.setAnkleTorque(supportSide, 0.0);
      }
   }

   private class DropRetractState implements State
   {
      private RobotSide supportSide, swingSide;
      private final YoPolynomial retractTrajectory;
      private final YoDouble initialRetractThighAngle;
      private final YoDouble desiredRetractThighAngle, desiredRetractThighVelocity;

      public DropRetractState(LeapOfFaithState stateEnum, LeapOfFaithState nextStateEnum, YoRegistry parentRegistry)
      {
         supportSide = stateEnum.getSupportSide();
         swingSide = stateEnum.getSwingSide();

         String swingSideName = swingSide.getCamelCaseName();

         initialRetractThighAngle = new YoDouble(swingSideName + "InitialRetractThighAngle", parentRegistry);

         retractTrajectory = new YoPolynomial(swingSideName + "Retract", 4, parentRegistry);

         desiredRetractThighAngle = new YoDouble(swingSideName + "DesiredRetractThighAngle", parentRegistry);
         desiredRetractThighVelocity = new YoDouble(swingSideName + "DesiredRetractThighVelocity", parentRegistry);
      }

      @Override
      public void onEntry()
      {
         initialRetractThighAngle.set(thighAngles.get(swingSide).getDoubleValue());
         retractTrajectory.setCubic(0.0, retractDuration.getValue(), initialRetractThighAngle.getValue(), finalRetractThighAngle.getValue());
         retractTrajectory.initialize();

         q_d_knees.get(swingSide).set(robot.getKneeAngle(swingSide));
         //         q_d_knees.get(supportSide).set(0.0); Keep support desired knee angle the same as it was before.
         dropSupportKneeVelocity.set(0.0);
      }

      @Override
      public boolean isDone(double timeInState)
      {
         if (robot.hasFootMadeContact(swingSide))
         {
            return true;
         }

         return false;
      }

      @Override
      public void doAction(double timeInState)
      {
         double supportHipTorque = bodyOrientationKp.getDoubleValue() * (q_d_pitch.getDoubleValue() - robot.getBodyAngle())
               - bodyOrientationKd.getDoubleValue() * robot.getBodyAngularVelocity();
         robot.setHipTorque(supportSide, -supportHipTorque);

         retractTrajectory.compute(Math.min(timeInState, retractDuration.getValue()));
         desiredRetractThighAngle.set(retractTrajectory.getValue());
         desiredRetractThighVelocity.set(retractTrajectory.getVelocity());

         double swingHipTorque = thighSwingKp.getDoubleValue() * (desiredRetractThighAngle.getDoubleValue() - thighAngles.get(swingSide).getDoubleValue())
               + thighSwingKd.getDoubleValue() * (desiredRetractThighVelocity.getValue() - thighVelocities.get(swingSide).getDoubleValue());
         robot.setHipTorque(swingSide, swingHipTorque);

         //         q_d_knees.get(swingSide).set(0.0);

         q_d_knees.get(swingSide).add(deltaTime.getDoubleValue() * straightenKneeForSwingVelocity.getDoubleValue());
         if (q_d_knees.get(swingSide).getDoubleValue() > kneeEndSwingAngle.getDoubleValue())
         {
            q_d_knees.get(swingSide).set(kneeEndSwingAngle.getDoubleValue());
         }

         dropSupportKneeVelocity.add(deltaTime.getDoubleValue() * dropSupportKneeAcceleration.getDoubleValue() * -robot.getCenterOfMassVelocityX());
         q_d_knees.get(supportSide).sub(deltaTime.getDoubleValue() * dropSupportKneeVelocity.getDoubleValue());
         if (q_d_knees.get(supportSide).getDoubleValue() > -minimumSupportKneeAngle.getDoubleValue())
         {
            q_d_knees.get(supportSide).set(-minimumSupportKneeAngle.getDoubleValue());
         }
         if (q_d_knees.get(supportSide).getDoubleValue() < -maximumSupportKneeAngle.getDoubleValue())
         {
            q_d_knees.get(supportSide).set(-maximumSupportKneeAngle.getDoubleValue());
         }

         double supportKneeTorque = kneeSupportKp.getDoubleValue() * (q_d_knees.get(supportSide).getDoubleValue() - robot.getKneeAngle(supportSide))
               - kneeSupportKd.getDoubleValue() * robot.getKneeVelocity(supportSide);
         //         if (supportKneeTorque < minimumKneeSupportForce.getDoubleValue())
         //         {
         //            supportKneeTorque = minimumKneeSupportForce.getDoubleValue();
         //         }
         robot.setKneeTorque(supportSide, supportKneeTorque);

         double swingKneeTorque = kneeSwingKp.getDoubleValue() * (q_d_knees.get(swingSide).getDoubleValue() - robot.getKneeAngle(swingSide))
               - kneeSwingKd.getDoubleValue() * robot.getKneeVelocity(swingSide);
         robot.setKneeTorque(swingSide, swingKneeTorque);

         double swingAnkleTorque = ankleSwingKp.getDoubleValue() * (q_d_ankles.get(swingSide).getDoubleValue() - robot.getAnkleAngle(swingSide))
               - ankleSwingKd.getDoubleValue() * robot.getAnkleVelocity(swingSide);

         robot.setAnkleTorque(swingSide, swingAnkleTorque);

         doBrakingOnSupportAnkleIfTooFast(supportSide);
      }
   };

   @Override
   public void initialize()
   {
   }

   @Override
   public YoRegistry getYoRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return getClass().getSimpleName();
   }

   @Override
   public String getDescription()
   {
      return getName();
   }

   @Override
   public void doControl()
   {
      deltaTime.set(robot.getTime() - previousTickTime.getDoubleValue());
      if (deltaTime.getDoubleValue() < 0.0)
      {
         deltaTime.set(0.0);
      }
      previousTickTime.set(robot.getTime());

      computeThighAngles();
      computeCapturePoint();

      stateMachine.doAction();
      stateMachine.doTransitions();
   }

   private void computeThighAngles()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         thighAngles.get(robotSide).set(robot.getThighAngle(robotSide));
         thighVelocities.get(robotSide).set(robot.getThighAngularVelocity(robotSide));
      }
   }

   private void computeCapturePoint()
   {
      capturePointX.set(robot.getCenterOfMassPositionX() + 0.3 * robot.getCenterOfMassVelocityX());

      capturePointLeftHeelX.set(capturePointX.getDoubleValue() - robot.getHeelXPosition(RobotSide.LEFT));
      capturePointRightHeelX.set(capturePointX.getDoubleValue() - robot.getHeelXPosition(RobotSide.RIGHT));
   }

   public enum LeapOfFaithState
   {
      RightLoadingLeftToeOff, RightSupportLeftSwing, RightDropLeftRetract, LeftLoadingRightToeOff, LeftSupportRightSwing, LeftDropRightRetract;

      public RobotSide getLoadingSide()
      {
         if (this == LeftLoadingRightToeOff)
            return RobotSide.LEFT;
         if (this == RightLoadingLeftToeOff)
            return RobotSide.RIGHT;
         throw new RuntimeException("No Loading in this state!");
      }

      public RobotSide getToeOffSide()
      {
         return getLoadingSide().getOppositeSide();
      }

      public RobotSide getSwingSide()
      {
         if (this == RightSupportLeftSwing)
            return RobotSide.LEFT;
         if (this == LeftSupportRightSwing)
            return RobotSide.RIGHT;
         if (this == RightDropLeftRetract)
            return RobotSide.LEFT;
         if (this == LeftDropRightRetract)
            return RobotSide.RIGHT;
         throw new RuntimeException("No Swing in this state! " + this);
      }

      public RobotSide getSupportSide()
      {
         return getSwingSide().getOppositeSide();
      }

   }

}