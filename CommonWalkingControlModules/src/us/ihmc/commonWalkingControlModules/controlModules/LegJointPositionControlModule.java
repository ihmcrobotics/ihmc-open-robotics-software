package us.ihmc.commonWalkingControlModules.controlModules;

import java.util.EnumMap;

import us.ihmc.commonWalkingControlModules.outputs.ProcessedOutputsInterface;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointPositions;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointVelocities;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegTorques;
import us.ihmc.sensorProcessing.ProcessedSensorsInterface;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.GUISetterUpper;
import us.ihmc.simulationconstructionset.gui.GUISetterUpperRegistry;
import us.ihmc.tools.containers.ContainerTools;
import us.ihmc.humanoidRobotics.partNames.LegJointName;
import us.ihmc.humanoidRobotics.partNames.RobotSpecificJointNames;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoUtilities.controllers.PIDController;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class LegJointPositionControlModule
{
   private final boolean useDesiredVelocities;

   private final RobotSide robotSide;
   private final LegJointName[] legJointNames;

   private final ProcessedSensorsInterface processedSensors;
   private final YoVariableRegistry registry;

   private final DoubleYoVariable desiredVelocityScale;

   private final EnumMap<LegJointName, DoubleYoVariable> kpGains = ContainerTools.createEnumMap(LegJointName.class);
   private final EnumMap<LegJointName, DoubleYoVariable> kdGains = ContainerTools.createEnumMap(LegJointName.class);
   private final EnumMap<LegJointName, DoubleYoVariable> jointPositionErrors = ContainerTools.createEnumMap(LegJointName.class);
   private final EnumMap<LegJointName, DoubleYoVariable> jointVelocityErrors = ContainerTools.createEnumMap(LegJointName.class);
   private final EnumMap<LegJointName, DoubleYoVariable> desiredJointPositions = ContainerTools.createEnumMap(LegJointName.class);
   private final EnumMap<LegJointName, DoubleYoVariable> desiredJointVelocities = ContainerTools.createEnumMap(LegJointName.class);

   private final DoubleYoVariable legGainScale, ankleTorqueScale;

   private static final String kpPrefix = "kp";
   private static final String kdPrefix = "kd";
   private static final String positionErrorPrefix = "jointPosErr";
   private static final String velocityErrorPrefix = "jointVelErr";
   private static final String desiredPositionPrefix = "desiredJointPos";
   private static final String desiredVelocityPrefix = "desiredJointVel";

   private final DoubleYoVariable softScaleFactor;

   private final EnumMap<LegJointName, PIDController> controllers = ContainerTools.createEnumMap(LegJointName.class);

   private final double controlDT;


   public LegJointPositionControlModule(RobotSpecificJointNames robotJointNames, ProcessedSensorsInterface processedSensors,
         ProcessedOutputsInterface processedOutputs, YoVariableRegistry yoVariableParentRegistry, GUISetterUpperRegistry guiSetterUpperRegistry,
         RobotSide robotSide, double controlDT,
         boolean useDesiredVelocities)
 {
      this(robotJointNames.getLegJointNames(), processedSensors, yoVariableParentRegistry, guiSetterUpperRegistry, robotSide, controlDT, useDesiredVelocities);
 }
   
   public LegJointPositionControlModule(LegJointName[] robotJointNames, ProcessedSensorsInterface processedSensors,
           YoVariableRegistry yoVariableParentRegistry, GUISetterUpperRegistry guiSetterUpperRegistry, RobotSide robotSide,
           double controlDT, boolean useDesiredVelocities)
   {
      this.legJointNames = robotJointNames;
      this.robotSide = robotSide;
      this.processedSensors = processedSensors;
      this.controlDT = controlDT;
      this.useDesiredVelocities = useDesiredVelocities;

      registry = new YoVariableRegistry("LegJointPositionControlModule_" + robotSide.getCamelCaseNameForStartOfExpression());

      String robotSideStartString = robotSide.getCamelCaseNameForStartOfExpression();
      String robotSideMiddleString = robotSide.getCamelCaseNameForMiddleOfExpression();

      desiredVelocityScale = new DoubleYoVariable(robotSideStartString + "DesiredVelocityScale", registry);

      for (LegJointName legJointName : legJointNames)
      {
         kpGains.put(legJointName,
                     new DoubleYoVariable(kpPrefix + robotSideMiddleString + legJointName.getCamelCaseNameForMiddleOfExpression(),
                                          "Position control gain for joint " + legJointName.getCamelCaseNameForMiddleOfExpression(), registry));

         kdGains.put(legJointName,
                     new DoubleYoVariable(kdPrefix + robotSideMiddleString + legJointName.getCamelCaseNameForMiddleOfExpression(),
                                          "Damping control gain for joint " + legJointName.getCamelCaseNameForMiddleOfExpression(), registry));

         jointPositionErrors.put(legJointName,
                                 new DoubleYoVariable(positionErrorPrefix + robotSideMiddleString + legJointName.getCamelCaseNameForMiddleOfExpression(),
                                    "Position error for joint " + legJointName.getCamelCaseNameForMiddleOfExpression(), registry));

         jointVelocityErrors.put(legJointName,
                                 new DoubleYoVariable(velocityErrorPrefix + robotSideMiddleString + legJointName.getCamelCaseNameForMiddleOfExpression(),
                                    "Velocity error for joint " + legJointName.getCamelCaseNameForMiddleOfExpression(), registry));

         desiredJointPositions.put(legJointName,
                                   new DoubleYoVariable(desiredPositionPrefix + robotSideMiddleString + legJointName.getCamelCaseNameForMiddleOfExpression(),
                                      "Desired position for joint " + legJointName.getCamelCaseNameForMiddleOfExpression(), registry));

         desiredJointVelocities.put(legJointName,
                                    new DoubleYoVariable(desiredVelocityPrefix + robotSideMiddleString + legJointName.getCamelCaseNameForMiddleOfExpression(),
                                       "Desired velocity for joint " + legJointName.getCamelCaseNameForMiddleOfExpression(), registry));

         controllers.put(legJointName, new PIDController(robotSideMiddleString + legJointName.getCamelCaseNameForMiddleOfExpression(), registry));
      }

      legGainScale = new DoubleYoVariable(robotSideStartString + "LegGainScale", "", registry);
      ankleTorqueScale = new DoubleYoVariable(robotSideStartString + "AnkleTorqueScale", "", registry);
      softScaleFactor = new DoubleYoVariable(robotSideStartString + "SoftScaleFactor", "This is the scale factor on ankle kp.", registry);

      // Set default gains to NaN to ensure that the user calls one of the
      // default gain setters. If not, should get NaN errors down the road.
      setDefaultGainsToNaN();
      resetScalesToDefault();

      yoVariableParentRegistry.addChild(registry);
      
      if (guiSetterUpperRegistry != null)
      {
         guiSetterUpperRegistry.registerGUISetterUpper(createGUISetterUpper());
      }
   }

   private void setDefaultGainsToNaN()
   {
      for (LegJointName legJointName : legJointNames)
      {
         kpGains.get(legJointName).set(Double.NaN);
         kdGains.get(legJointName).set(Double.NaN);

         softScaleFactor.set(Double.NaN);
      }

   }

   public void setDefaultGainsForM2V2()
   {
      kpGains.get(LegJointName.HIP_YAW).set(120.0);
      kpGains.get(LegJointName.HIP_ROLL).set(200.0);
      kpGains.get(LegJointName.HIP_PITCH).set(200.0);
      kpGains.get(LegJointName.KNEE).set(120.0);
      kpGains.get(LegJointName.ANKLE_ROLL).set(8.0);
      kpGains.get(LegJointName.ANKLE_PITCH).set(8.0);

      kdGains.get(LegJointName.HIP_YAW).set(2.0);
      kdGains.get(LegJointName.HIP_ROLL).set(8.0);
      kdGains.get(LegJointName.HIP_PITCH).set(8.0);
      kdGains.get(LegJointName.KNEE).set(6.0);
      kdGains.get(LegJointName.ANKLE_ROLL).set(1.0);
      kdGains.get(LegJointName.ANKLE_PITCH).set(1.0);

      softScaleFactor.set(0.1); // 0.25);
   }

   public void setDefaultGainsForR2()
   {
      // kpGains.get(LegJointName.HIP_YAW).val = 5000.0;    // 1000.0;
      // kpGains.get(LegJointName.HIP_ROLL).val = 5000.0;    // 1000.0;
      // kpGains.get(LegJointName.HIP_PITCH).val = 5000.0;    // 1000.0;
      // kpGains.get(LegJointName.KNEE).val = 2500.0;    // 1000.0;
      // kpGains.get(LegJointName.ANKLE_PITCH).val = 50.0;    // 20.0;
      // kpGains.get(LegJointName.ANKLE_ROLL).val = 50.0;    // 20.0;
      //
      // kdGains.get(LegJointName.HIP_YAW).val = 50.0;    // 100.0;
      // kdGains.get(LegJointName.HIP_ROLL).val = 50.0;    // 100.0;
      // kdGains.get(LegJointName.HIP_PITCH).val = 50;    // 100.0;
      // kdGains.get(LegJointName.KNEE).val = 50.0;    // 100.0;
      // kdGains.get(LegJointName.ANKLE_PITCH).val = 5.0;    // 2.0;//5.0;
      // kdGains.get(LegJointName.ANKLE_ROLL).val = 5.0;    // 2.0;

      kpGains.get(LegJointName.HIP_YAW).set(1400.0);    // 1000.0;
      kpGains.get(LegJointName.HIP_ROLL).set(1400.0);    // 1000.0;
      kpGains.get(LegJointName.HIP_PITCH).set(1400.0);    // 1000.0;
      kpGains.get(LegJointName.KNEE).set(1000.0);    // 1000.0;    // 1000.0;
      kpGains.get(LegJointName.ANKLE_ROLL).set(10.0);    // 100.0;    // 20.0;
      kpGains.get(LegJointName.ANKLE_PITCH).set(10.0);    // 250.0;    // 20.0;

      kdGains.get(LegJointName.HIP_YAW).set(50.0);    // 100.0;
      kdGains.get(LegJointName.HIP_ROLL).set(50.0);    // 100.0;
      kdGains.get(LegJointName.HIP_PITCH).set(50.0);    // 100.0;
      kdGains.get(LegJointName.KNEE).set(50.0);    // 100.0;    // 100.0;
      kdGains.get(LegJointName.ANKLE_ROLL).set(1.0);    // 3.0); // 10.0;    // 2.0;
      kdGains.get(LegJointName.ANKLE_PITCH).set(3.0);    // 20.0;    // 2.0;//5.0;
      
      softScaleFactor.set(0.25);    // (0.05);
   }

   public void packTorquesForLegJointsPositionControl(double scaleFactorBasedOnJacobianDeterminant, LegTorques legTorquesToPackForSwingLeg, LegJointPositions desiredLegJointPositions,
           LegJointVelocities desiredLegJointVelocities)
   {
      // RobotSide checks
      legTorquesToPackForSwingLeg.getRobotSide().checkRobotSideMatch(desiredLegJointPositions.getRobotSide());
      legTorquesToPackForSwingLeg.getRobotSide().checkRobotSideMatch(robotSide);

      if (desiredLegJointVelocities == null)
      {
         desiredLegJointVelocities = new LegJointVelocities(legJointNames, robotSide);
         desiredLegJointVelocities.setJointVelocitiesToNAN();
      }

      for (LegJointName legJointName : legJointNames)
      {
         desiredJointPositions.get(legJointName).set(desiredLegJointPositions.getJointPosition(legJointName));
         desiredJointVelocities.get(legJointName).set(desiredLegJointVelocities.getJointVelocity(legJointName));

         double actualJointPosition = processedSensors.getLegJointPosition(robotSide, legJointName);

         desiredVelocityScale.set(MathTools.clipToMinMax(scaleFactorBasedOnJacobianDeterminant, 0.0, 1.0));
         
         double kpGain = kpGains.get(legJointName).getDoubleValue() * legGainScale.getDoubleValue();
         double kdGain = kdGains.get(legJointName).getDoubleValue() * legGainScale.getDoubleValue();    // * kdScale.getDoubleValue();

         controllers.get(legJointName).setProportionalGain(kpGain);
         controllers.get(legJointName).setDerivativeGain(kdGain);

         double torque;
//         if (doDerivativeControlInThisModuleForJoint.get(legJointName))
//         {
            double desiredJointVelocity = useDesiredVelocities
                                          ? desiredJointVelocities.get(legJointName).getDoubleValue() * desiredVelocityScale.getDoubleValue() : 0.0;

            double actualJointVelocity = processedSensors.getLegJointVelocity(robotSide, legJointName);

            torque = controllers.get(legJointName).compute(actualJointPosition, desiredJointPositions.get(legJointName).getDoubleValue(), actualJointVelocity,
                                     desiredJointVelocity, controlDT);

//         }
//         else
//         {
//            processedOutputs.setDampingParameter(robotSide, legJointName, kdGains.get(legJointName).getDoubleValue());
//            double desiredJointVelocity = useDesiredVelocities
//                                          ? desiredJointVelocities.get(legJointName).getDoubleValue() * desiredVelocityScale.getDoubleValue() : 0.0;
//            processedOutputs.setDesiredJointVelocity(robotSide, legJointName, desiredJointVelocity);
//
//            // double torque = controllers.get(legJointName).compute(actualJointPosition, desiredJointPositions.get(legJointName).getDoubleValue(), actualJointVelocity,
//            // desiredJointVelocity, controlDT); // if this were called in the low level loop
//
//            torque = controllers.get(legJointName).compute(actualJointPosition, desiredJointPositions.get(legJointName).getDoubleValue(), 0.0, 0.0, controlDT);    // don't do derivative stuff here; that's for the low level loop
//            
//
//         }

         legTorquesToPackForSwingLeg.setTorque(legJointName, torque);
      }

      // For soft touchdowns reduce the ankle torques more...
      legTorquesToPackForSwingLeg.setTorque(LegJointName.ANKLE_PITCH,
              legTorquesToPackForSwingLeg.getTorque(LegJointName.ANKLE_PITCH) * ankleTorqueScale.getDoubleValue());

      return;
   }

   public void setProportionalGain(LegJointName legJointName, double proportionalGain)
   {
      kpGains.get(legJointName).set(proportionalGain);
   }

   public void setDerivativeGain(LegJointName legJointName, double derivativeGain)
   {
      kdGains.get(legJointName).set(derivativeGain);
   }

   public void setAnkleGainsSoft()
   {
      ankleTorqueScale.set(softScaleFactor.getDoubleValue());
   }

   public void scaleLegGains(double scaleFactor)
   {
      scaleFactor = MathTools.clipToMinMax(scaleFactor, 0.0, 1.0);
      legGainScale.set(scaleFactor);
   }

   public void resetScalesToDefault()
   {
      legGainScale.set(1.0);
      ankleTorqueScale.set(1.0);
   }

//   public void disableJointDamping()
//   {
//      for (LegJointName legJointName : legJointNames)
//      {
//         processedOutputs.setDampingParameter(robotSide, legJointName, 0.0);
//      }
//   }

   
   private GUISetterUpper createGUISetterUpper()
   {
      GUISetterUpper ret = new GUISetterUpper()
      {
         public void setupGUI(SimulationConstructionSet scs)
         {
            int numberOfLegJointNames = legJointNames.length;

            String[][][] positionGraphGroupStrings = new String[numberOfLegJointNames][][];
            String[][][] velocityGraphGroupStrings = new String[numberOfLegJointNames][][];

            String[] entryBoxGroupStrings = new String[2 * numberOfLegJointNames];    // times two, because we want both kp and kd

            for (int jointNameIndex = 0; jointNameIndex < numberOfLegJointNames; jointNameIndex++)
            {
               LegJointName jointName = legJointNames[jointNameIndex];

               // positions
               String desiredPositionName = desiredJointPositions.get(jointName).getName();
               String actualPositionName = processedSensors.getLegJointPositionName(robotSide, jointName);

               positionGraphGroupStrings[jointNameIndex] = new String[][]
                                                                        {
                     {desiredPositionName, actualPositionName}, {"auto"}
                                                                        };

               // velocities
               String desiredVelocityName = desiredJointVelocities.get(jointName).getName();
               String actualVelocityName = processedSensors.getLegJointVelocityName(robotSide, jointName);

               velocityGraphGroupStrings[jointNameIndex] = new String[][]
                                                                        {
                     {desiredVelocityName, actualVelocityName}, {"auto"}
                                                                        };

               // kp, kd
               entryBoxGroupStrings[jointNameIndex] = kpGains.get(jointName).getName();
               entryBoxGroupStrings[numberOfLegJointNames + jointNameIndex] = kdGains.get(jointName).getName();
            }

            String sideString = "(" + robotSide.getCamelCaseNameForStartOfExpression() + ")";
            scs.setupGraphGroup("LegJointPositionControlModule - positions " + sideString, positionGraphGroupStrings, 2);
            scs.setupGraphGroup("LegJointPositionControlModule - velocities " + sideString, velocityGraphGroupStrings, 2);
            scs.setupEntryBoxGroup("LegJointPositionControlModule", entryBoxGroupStrings);
            scs.setupConfiguration("LegJointPositionControlModule", "all", "LegJointPositionControlModule - velocities", "LegJointPositionControlModule");
         }
      };
      return ret;
   }

}

