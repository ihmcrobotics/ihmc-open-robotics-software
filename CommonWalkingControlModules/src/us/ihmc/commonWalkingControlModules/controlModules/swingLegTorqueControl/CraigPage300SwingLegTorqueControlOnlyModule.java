package us.ihmc.commonWalkingControlModules.controlModules.swingLegTorqueControl;

import java.util.Collection;
import java.util.EnumMap;

import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.SwingLegTorqueControlOnlyModule;
import us.ihmc.commonWalkingControlModules.controlModules.LegJointPositionControlModule;
import us.ihmc.commonWalkingControlModules.couplingRegistry.CouplingRegistry;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointAccelerations;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointPositions;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointVelocities;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegTorques;
import us.ihmc.sensorProcessing.ProcessedSensorsInterface;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.GUISetterUpper;
import us.ihmc.simulationconstructionset.gui.GUISetterUpperRegistry;
import us.ihmc.humanoidRobotics.frames.CommonHumanoidReferenceFrames;
import us.ihmc.humanoidRobotics.model.FullHumanoidRobotModel;
import us.ihmc.humanoidRobotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.InverseDynamicsCalculator;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.yoUtilities.dataStructure.listener.VariableChangedListener;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;

public class CraigPage300SwingLegTorqueControlOnlyModule implements SwingLegTorqueControlOnlyModule
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final LegJointName[] legJointNames;

   private final ProcessedSensorsInterface processedSensors;

   private final CommonHumanoidReferenceFrames referenceFrames;
   private final FullHumanoidRobotModel fullRobotModel;
   private final CouplingRegistry couplingRegistry;
   private final SideDependentList<EnumMap<LegJointName, DoubleYoVariable>> desiredYoLegJointPositions =
      SideDependentList.createListOfEnumMaps(LegJointName.class);
   private final SideDependentList<EnumMap<LegJointName, DoubleYoVariable>> desiredYoLegJointVelocities =
      SideDependentList.createListOfEnumMaps(LegJointName.class);
   private final SideDependentList<EnumMap<LegJointName, DoubleYoVariable>> jointPositionErrors = SideDependentList.createListOfEnumMaps(LegJointName.class);
   private final SideDependentList<EnumMap<LegJointName, DoubleYoVariable>> jointVelocityErrors = SideDependentList.createListOfEnumMaps(LegJointName.class);
   private final EnumMap<LegJointName, DoubleYoVariable> kpGains = new EnumMap<LegJointName, DoubleYoVariable>(LegJointName.class);
   private final EnumMap<LegJointName, DoubleYoVariable> kdGains = new EnumMap<LegJointName, DoubleYoVariable>(LegJointName.class);

   private final DoubleYoVariable masterKpGain = new DoubleYoVariable("masterKpGain", registry);
   private final DoubleYoVariable masterKdGain = new DoubleYoVariable("masterKdGain", registry);

   private final DoubleYoVariable softScaleFactor = new DoubleYoVariable("softScaleFactor", registry);
   private final DoubleYoVariable ankleTorqueScale = new DoubleYoVariable("ankleTorqueScale", registry);
   
   private final SideDependentList<LegJointPositions> desiredLegJointPositions = new SideDependentList<LegJointPositions>();
   private final SideDependentList<LegJointVelocities> desiredLegJointVelocities = new SideDependentList<LegJointVelocities>();

   private final SideDependentList<InverseDynamicsCalculator> inverseDynamicsCalculators;
   private final SideDependentList<LegJointPositionControlModule> legJointPositionControlModules;
   private boolean useBodyAcceleration;


   public CraigPage300SwingLegTorqueControlOnlyModule(LegJointName[] legJointNames, ProcessedSensorsInterface processedSensors,
           CommonHumanoidReferenceFrames referenceFrames, FullHumanoidRobotModel fullRobotModel, CouplingRegistry couplingRegistry,
           SideDependentList<InverseDynamicsCalculator> inverseDynamicsCalculators, SideDependentList<LegJointPositionControlModule> legJointPositionControlModules, YoVariableRegistry parentRegistry,
           GUISetterUpperRegistry guiSetterUpperRegistry)
   {
      this.legJointNames = legJointNames;
      this.processedSensors = processedSensors;
      this.referenceFrames = referenceFrames;
      this.fullRobotModel = fullRobotModel;
      this.couplingRegistry = couplingRegistry;
      this.inverseDynamicsCalculators = inverseDynamicsCalculators;
      this.legJointPositionControlModules = legJointPositionControlModules;

      for (RobotSide robotSide : RobotSide.values)
      {
         this.desiredLegJointPositions.put(robotSide, new LegJointPositions(robotSide));
         this.desiredLegJointVelocities.put(robotSide, new LegJointVelocities(legJointNames, robotSide));

         for (LegJointName legJointName : legJointNames)
         {
            String jointName = robotSide.getCamelCaseNameForStartOfExpression() + legJointName.getCamelCaseNameForMiddleOfExpression();
            desiredYoLegJointPositions.get(robotSide).put(legJointName, new DoubleYoVariable(jointName + "DesiredPosition", registry));
            desiredYoLegJointVelocities.get(robotSide).put(legJointName, new DoubleYoVariable(jointName + "DesiredVelocity", registry));
            jointPositionErrors.get(robotSide).put(legJointName, new DoubleYoVariable(jointName + "PositionError", registry));
            jointVelocityErrors.get(robotSide).put(legJointName, new DoubleYoVariable(jointName + "VelocityError", registry));
         }
      }

      for (LegJointName legJointName : legJointNames)
      {
         String jointName = legJointName.getCamelCaseNameForMiddleOfExpression();
         kpGains.put(legJointName, new DoubleYoVariable(jointName + "KpGain", registry));
         kdGains.put(legJointName, new DoubleYoVariable(jointName + "KdGain", registry));
      }


      masterKpGain.addVariableChangedListener(new MasterGainVariableChangedListener(kpGains.values()));
      masterKdGain.addVariableChangedListener(new MasterGainVariableChangedListener(kdGains.values()));
      
      ankleTorqueScale.set(1.0);

      parentRegistry.addChild(registry);
      if (guiSetterUpperRegistry != null)
         guiSetterUpperRegistry.registerGUISetterUpper(createGUISetterUpper());
   }

   public void compute(LegTorques legTorquesToPackForSwingLeg, LegJointPositions jointPositions, LegJointVelocities jointVelocities, LegJointAccelerations jointAccelerations)
   {
      // robotSides
      RobotSide swingSide = legTorquesToPackForSwingLeg.getRobotSide();



      
      
      // set body acceleration
      if (useBodyAcceleration)
      {
         SpatialAccelerationVector bodyAcceleration = processedSensors.getAccelerationOfPelvisWithRespectToWorld();    // FIXME: set to LIPM-based predicted body acceleration
         bodyAcceleration.setAngularPart(new Vector3d());    // zero desired angular acceleration
         bodyAcceleration.setLinearPart(new Vector3d());    // zero linear acceleration as well for now
         fullRobotModel.getRootJoint().setDesiredAcceleration(bodyAcceleration);
      }


      desiredLegJointPositions.get(swingSide).set(jointPositions);
      desiredLegJointVelocities.get(swingSide).set(jointVelocities);
      
      // Compute ID once to find upper body wrench (including accelerations or PD terms in this computation will cause bad vibrations)
      for (LegJointName legJointName : legJointNames)
         fullRobotModel.getLegJoint(swingSide, legJointName).setQddDesired(0.0);
      inverseDynamicsCalculators.get(swingSide).compute();
      setUpperBodyWrench();

      // Compute ID once more, now with augmented accelerations to compensate for position and velocity errors.
      LegJointName[] legJointNames = fullRobotModel.getRobotSpecificJointNames().getLegJointNames();
      for (LegJointName legJointName : legJointNames)
      {
         OneDoFJoint revoluteJoint = fullRobotModel.getLegJoint(swingSide, legJointName);         
         double qddDesired = jointAccelerations.getJointAcceleration(legJointName);

         double desiredJointPosition = desiredLegJointPositions.get(swingSide).getJointPosition(legJointName);
         desiredYoLegJointPositions.get(swingSide).get(legJointName).set(desiredJointPosition);
         double positionError = desiredJointPosition - processedSensors.getLegJointPosition(swingSide, legJointName);
         jointPositionErrors.get(swingSide).get(legJointName).set(positionError);

         double desiredJointVelocity = desiredLegJointVelocities.get(swingSide).getJointVelocity(legJointName);
         desiredYoLegJointVelocities.get(swingSide).get(legJointName).set(desiredJointVelocity);
         double velocityError = desiredJointVelocity - processedSensors.getLegJointVelocity(swingSide, legJointName);
         jointVelocityErrors.get(swingSide).get(legJointName).set(velocityError);

         double kpGain = kpGains.get(legJointName).getDoubleValue();
         double kdGain = kdGains.get(legJointName).getDoubleValue();
         double qddDesiredWithPD = positionError * kpGain + velocityError * kdGain + qddDesired;
         
         revoluteJoint.setQddDesired(qddDesiredWithPD);
      }

      // control
      inverseDynamicsCalculators.get(swingSide).compute();

      
      
      /*
       * The legJointPositionControlModule does damping of all joints by setting desired velocity and damping in the low level controller. The higher loop rate allows higher values for damping. 
       * 
       * The torques for Hip Pitch/Roll and the Knee get overwritten by values from the CraigPage300 controller, but the damping (speed control) is still applied
       * by the low level side!
       *  
       */
      legJointPositionControlModules.get(swingSide).packTorquesForLegJointsPositionControl(1.0, legTorquesToPackForSwingLeg, jointPositions, jointVelocities);
      
      
      
      for (LegJointName legJointName : legTorquesToPackForSwingLeg.getLegJointNames())
      {
         double tauInverseDynamics = fullRobotModel.getLegJoint(swingSide, legJointName).getTau();

         boolean isAnAnkleJoint = (legJointName == LegJointName.ANKLE_PITCH) || (legJointName == LegJointName.ANKLE_ROLL);
         if (isAnAnkleJoint)
            tauInverseDynamics *= ankleTorqueScale.getDoubleValue();

//        legTorquesToPackForSwingLeg.addTorque(legJointName, tauInverseDynamics);
         
         if(legJointName != LegJointName.HIP_YAW && !isAnAnkleJoint)
         {
            legTorquesToPackForSwingLeg.setTorque(legJointName, tauInverseDynamics);
         }
      }
      setUpperBodyWrench();
   }

   public void setAnkleGainsSoft(RobotSide swingSide)
   {
      ankleTorqueScale.set(softScaleFactor.getDoubleValue());
   }

   public void setAnkleGainsDefault(RobotSide swingSide)
   {
      ankleTorqueScale.set(1.0);
   }


   private void setUpperBodyWrench()
   {
      Wrench upperBodyWrench = new Wrench();
      fullRobotModel.getRootJoint().packWrench(upperBodyWrench);
      upperBodyWrench.changeBodyFrameAttachedToSameBody(referenceFrames.getPelvisFrame());
      upperBodyWrench.changeFrame(referenceFrames.getPelvisFrame());
      couplingRegistry.setDesiredUpperBodyWrench(upperBodyWrench);
      
   }

   public void setParametersForR2()
   {
      useBodyAcceleration = false;

      for (LegJointName legJointName : legJointNames)
      {
         kpGains.get(legJointName).set(1000.0);
         kdGains.get(legJointName).set(50.0);
      }
      
      softScaleFactor.set(0.1);
   }

   public void setParametersForM2V2()
   {
      useBodyAcceleration = true;

//      masterKpGain.set(150.0);
//      masterKdGain.set(2.0); TODO

      masterKpGain.set(150.0);
      masterKdGain.set(3.0);

      softScaleFactor.set(0.1); // 0.25);
   }
   
   public void setParametersForOptimalSwing()
   {

      masterKpGain.set(0.0);
      masterKdGain.set(0.0);
   }

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

            for (RobotSide robotSide : RobotSide.values)
            {
               for (int jointNameIndex = 0; jointNameIndex < numberOfLegJointNames; jointNameIndex++)
               {
                  LegJointName jointName = legJointNames[jointNameIndex];

                  // positions
                  String desiredPositionName = desiredYoLegJointPositions.get(robotSide).get(jointName).getName();
                  String actualPositionName = processedSensors.getLegJointPositionName(robotSide, jointName);

                  positionGraphGroupStrings[jointNameIndex] = new String[][]
                  {
                     {desiredPositionName, actualPositionName}, {"auto"}
                  };

                  // velocities
                  String desiredVelocityName = desiredYoLegJointVelocities.get(robotSide).get(jointName).getName();
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
               scs.setupGraphGroup("CraigPage300 - positions " + sideString, positionGraphGroupStrings, 2);
               scs.setupGraphGroup("CraigPage300 - velocities " + sideString, velocityGraphGroupStrings, 2);
               scs.setupEntryBoxGroup("CraigPage300", entryBoxGroupStrings);
               scs.setupConfiguration("CraigPage300", "all", "CraigPage300 - velocities", "CraigPage300");
            }
         }
      };

      return ret;
   }

   private class MasterGainVariableChangedListener implements VariableChangedListener
   {
      private Collection<DoubleYoVariable> slaves;

      public MasterGainVariableChangedListener(Collection<DoubleYoVariable> slaves)
      {
         this.slaves = slaves;
      }

      public void variableChanged(YoVariable master)
      {
         for (DoubleYoVariable slave : slaves)
         {
            slave.set(master.getValueAsDouble());
         }
      }
   }

   public void computePreSwing(LegTorques legTorquesToPack)
   {
      RobotSide swingSide = legTorquesToPack.getRobotSide();
      fullRobotModel.getRootJoint().setDesiredAccelerationToZero();

      for (LegJointName legJointName : legJointNames)
      {
         fullRobotModel.getLegJoint(swingSide, legJointName).setQddDesired(0.0);
      }

      inverseDynamicsCalculators.get(swingSide).compute();
      setUpperBodyWrench();
      
      for (LegJointName legJointName : legJointNames)
      {
         legTorquesToPack.setTorque(legJointName, fullRobotModel.getLegJoint(swingSide, legJointName).getTau());
      }
   }

//   public void setDampingToZero(RobotSide swingSide)
//   {
//      legJointPositionControlModules.get(swingSide).disableJointDamping();
//   }
}
