package us.ihmc.commonWalkingControlModules.controlModules.swingLegTorqueControl;

import java.util.EnumMap;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.SdfLoader.partNames.LegJointName;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.SwingLegTorqueControlModule;
import us.ihmc.commonWalkingControlModules.couplingRegistry.CouplingRegistry;
import us.ihmc.commonWalkingControlModules.kinematics.DesiredJointAccelerationCalculatorInWorldFrame;
import us.ihmc.commonWalkingControlModules.kinematics.DesiredJointVelocityCalculator;
import us.ihmc.commonWalkingControlModules.kinematics.InverseKinematicsException;
import us.ihmc.commonWalkingControlModules.kinematics.LegInverseKinematicsCalculator;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointPositions;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointVelocities;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegTorques;
import us.ihmc.sensorProcessing.ProcessedSensorsInterface;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.simulationconstructionset.MasterVariableChangedListener;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.GUISetterUpper;
import us.ihmc.simulationconstructionset.gui.GUISetterUpperRegistry;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.InverseDynamicsCalculator;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.Wrench;

public class CraigPage300SwingLegTorqueControlModule implements SwingLegTorqueControlModule
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final LegJointName[] legJointNames;

   private final ProcessedSensorsInterface processedSensors;

   private final CommonHumanoidReferenceFrames referenceFrames;
   private final FullHumanoidRobotModel fullRobotModel;
   private final CouplingRegistry couplingRegistry;
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

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
   
   private final DoubleYoVariable dampedLeastSquaresAlpha = new DoubleYoVariable("dampedLeastSquaresAlpha", registry);

   private final SideDependentList<LegJointPositions> desiredLegJointPositions = new SideDependentList<LegJointPositions>();
   private final SideDependentList<LegJointVelocities> desiredLegJointVelocities = new SideDependentList<LegJointVelocities>();

   private final LegInverseKinematicsCalculator inverseKinematicsCalculator;
   private final SideDependentList<DesiredJointVelocityCalculator> desiredJointVelocityCalculators;
   private final SideDependentList<DesiredJointAccelerationCalculatorInWorldFrame> desiredJointAccelerationCalculators;

   private final SideDependentList<InverseDynamicsCalculator> inverseDynamicsCalculators;

   private final BooleanYoVariable inverseKinematicsExceptionHasBeenThrown = new BooleanYoVariable("kinematicException", registry);
//   private final DoubleYoVariable jacobianDeterminant = new DoubleYoVariable("jacobianDeterminant", registry);
//   private final DoubleYoVariable inverseDynamicsPercentScaling = new DoubleYoVariable("inverseDynamicsPercentScaling", registry);
   private boolean useBodyAcceleration;


   public CraigPage300SwingLegTorqueControlModule(LegJointName[] legJointNames, ProcessedSensorsInterface processedSensors,
           CommonHumanoidReferenceFrames referenceFrames, FullHumanoidRobotModel fullRobotModel, CouplingRegistry couplingRegistry,
           LegInverseKinematicsCalculator inverseKinematicsCalculator, SideDependentList<DesiredJointVelocityCalculator> desiredJointVelocityCalculators,
           SideDependentList<DesiredJointAccelerationCalculatorInWorldFrame> desiredJointAccelerationCalculators,
           SideDependentList<InverseDynamicsCalculator> inverseDynamicsCalculators, YoVariableRegistry parentRegistry,
           GUISetterUpperRegistry guiSetterUpperRegistry)
   {
      this.legJointNames = legJointNames;
      this.processedSensors = processedSensors;
      this.referenceFrames = referenceFrames;
      this.fullRobotModel = fullRobotModel;
      this.couplingRegistry = couplingRegistry;
      this.inverseKinematicsCalculator = inverseKinematicsCalculator;
      this.desiredJointVelocityCalculators = desiredJointVelocityCalculators;
      this.desiredJointAccelerationCalculators = desiredJointAccelerationCalculators;
      this.inverseDynamicsCalculators = inverseDynamicsCalculators;

      for (RobotSide robotSide : RobotSide.values)
      {
         this.desiredLegJointPositions.put(robotSide, new LegJointPositions(robotSide));
         this.desiredLegJointVelocities.put(robotSide, new LegJointVelocities(legJointNames, robotSide));

         for (LegJointName legJointName : legJointNames)
         {
            String jointName = robotSide.getCamelCaseNameForStartOfExpression() + legJointName.getCamelCaseNameForMiddleOfExpression();
            desiredYoLegJointPositions.get(robotSide).put(legJointName, new DoubleYoVariable(jointName + "DesiredPosition", parentRegistry));
            desiredYoLegJointVelocities.get(robotSide).put(legJointName, new DoubleYoVariable(jointName + "DesiredVelocity", parentRegistry));
            jointPositionErrors.get(robotSide).put(legJointName, new DoubleYoVariable(jointName + "PositionError", parentRegistry));
            jointVelocityErrors.get(robotSide).put(legJointName, new DoubleYoVariable(jointName + "VelocityError", parentRegistry));
         }
      }

      for (LegJointName legJointName : legJointNames)
      {
         String jointName = legJointName.getCamelCaseNameForMiddleOfExpression();
         kpGains.put(legJointName, new DoubleYoVariable(jointName + "KpGain", parentRegistry));
         kdGains.put(legJointName, new DoubleYoVariable(jointName + "KdGain", parentRegistry));
      }


      masterKpGain.addVariableChangedListener(new MasterVariableChangedListener(kpGains.values()));
      masterKdGain.addVariableChangedListener(new MasterVariableChangedListener(kdGains.values()));

      parentRegistry.addChild(registry);
      if (guiSetterUpperRegistry != null)
         guiSetterUpperRegistry.registerGUISetterUpper(createGUISetterUpper());
   }

   public void compute(LegTorques legTorquesToPackForSwingLeg, FramePoint desiredFootPosition, FrameOrientation desiredFootOrientation,
                       FrameVector desiredFootVelocity, FrameVector desiredFootAngularVelocity, FrameVector desiredFootAcceleration,
                       FrameVector desiredFootAngularAcceleration)
   {
      // robotSides
      RobotSide swingSide = legTorquesToPackForSwingLeg.getRobotSide();

      // reference frames
      ReferenceFrame pelvisFrame = referenceFrames.getPelvisFrame();
      ReferenceFrame footFrame = referenceFrames.getFootFrame(swingSide);
      ReferenceFrame footCoMFrame = fullRobotModel.getFoot(swingSide).getBodyFixedFrame();
      ReferenceFrame elevatorFrame = fullRobotModel.getElevatorFrame();

      // Desired positions
      RigidBodyTransform footToPelvis = computeDesiredTransform(pelvisFrame, desiredFootPosition, desiredFootOrientation);
      Twist desiredTwistOfSwingFootWithRespectToWorld = computeDesiredTwist(worldFrame, footFrame, desiredFootVelocity, desiredFootAngularVelocity);
      desiredTwistOfSwingFootWithRespectToWorld.changeFrame(footCoMFrame);
      desiredTwistOfSwingFootWithRespectToWorld.changeBodyFrameNoRelativeTwist(footCoMFrame);

      Matrix3d footToPelvisOrientation = new Matrix3d();
      footToPelvis.get(footToPelvisOrientation);
      double desiredHipYaw = RotationTools.getYaw(footToPelvisOrientation);    // TODO: wrong and not necessary for R2, but ok for now.
      try
      {
         inverseKinematicsCalculator.solve(desiredLegJointPositions.get(swingSide), footToPelvis, swingSide, desiredHipYaw);
         inverseKinematicsExceptionHasBeenThrown.set(false);
      }
      catch (InverseKinematicsException e)
      {
         inverseKinematicsExceptionHasBeenThrown.set(true);
      }

      // Desired velocities
      DesiredJointVelocityCalculator desiredJointVelocityCalculator = desiredJointVelocityCalculators.get(swingSide);
      desiredJointVelocityCalculator.packDesiredJointVelocities(desiredLegJointVelocities.get(swingSide), desiredTwistOfSwingFootWithRespectToWorld, dampedLeastSquaresAlpha.getDoubleValue());

      // set body acceleration
      if (useBodyAcceleration)
      {
         SpatialAccelerationVector bodyAcceleration = processedSensors.getAccelerationOfPelvisWithRespectToWorld();    // FIXME: set to LIPM-based predicted body acceleration
         bodyAcceleration.setAngularPart(new Vector3d());    // zero desired angular acceleration
         bodyAcceleration.setLinearPart(new Vector3d());    // zero linear acceleration as well for now
         fullRobotModel.getRootJoint().setDesiredAcceleration(bodyAcceleration);
      }

      // Desired acceleration
      SpatialAccelerationVector desiredAccelerationOfSwingFootWithRespectToWorld = computeDesiredSwingFootSpatialAcceleration(elevatorFrame, footFrame,
                                                                                      desiredFootAcceleration, desiredFootAngularAcceleration);
      desiredAccelerationOfSwingFootWithRespectToWorld.changeFrameNoRelativeMotion(footCoMFrame);
      desiredAccelerationOfSwingFootWithRespectToWorld.changeBodyFrameNoRelativeAcceleration(footCoMFrame);
//      jacobianDeterminant.set(desiredJointVelocityCalculator.swingFullLegJacobianDeterminant());
      desiredJointAccelerationCalculators.get(swingSide).compute(desiredAccelerationOfSwingFootWithRespectToWorld, dampedLeastSquaresAlpha.getDoubleValue());

//      inverseDynamicsPercentScaling.set(getPercentScalingBasedOnJacobianDeterminant(jacobianDeterminant.getDoubleValue()));

      // Compute ID once to find upper body wrench (including PD terms in this computation will cause bad vibrations)
      inverseDynamicsCalculators.get(swingSide).compute();
      setUpperBodyWrench();

      // Compute ID once more, now with augmented accelerations to compensate for position and velocity errors.
      LegJointName[] legJointNames = fullRobotModel.getRobotSpecificJointNames().getLegJointNames();
      for (LegJointName legJointName : legJointNames)
      {
         OneDoFJoint revoluteJoint = fullRobotModel.getLegJoint(swingSide, legJointName);
         double qddDesired = revoluteJoint.getQddDesired();

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
      inverseDynamicsCalculators.get(swingSide).compute();

      // Set joint torques
      for (LegJointName legJointName : legTorquesToPackForSwingLeg.getLegJointNames())
      {
         double tauInverseDynamics = fullRobotModel.getLegJoint(swingSide, legJointName).getTau();

         boolean isAnAnkleJoint = (legJointName == LegJointName.ANKLE_PITCH) || (legJointName == LegJointName.ANKLE_ROLL);
         if (isAnAnkleJoint)
            tauInverseDynamics *= ankleTorqueScale.getDoubleValue();

         legTorquesToPackForSwingLeg.addTorque(legJointName, tauInverseDynamics);
      }

   }

   public void computePreSwing(RobotSide swingSide)
   {
      fullRobotModel.getRootJoint().setDesiredAccelerationToZero();

      for (LegJointName legJointName : legJointNames)
      {
         fullRobotModel.getLegJoint(swingSide, legJointName).setQddDesired(0.0);
      }

      inverseDynamicsCalculators.get(swingSide).compute();
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

   private RigidBodyTransform computeDesiredTransform(ReferenceFrame pelvisFrame, FramePoint desiredFootPosition, FrameOrientation desiredFootOrientation)
   {
      desiredFootOrientation.changeFrame(pelvisFrame);
      desiredFootPosition.changeFrame(pelvisFrame);
      RigidBodyTransform footToPelvis = createTransform(desiredFootOrientation, desiredFootPosition);

      return footToPelvis;
   }

   private Twist computeDesiredTwist(ReferenceFrame worldFrame, ReferenceFrame footFrame, FrameVector desiredFootVelocity,
                                     FrameVector desiredFootAngularVelocity)
   {
      desiredFootVelocity.changeFrame(footFrame);
      desiredFootAngularVelocity.changeFrame(footFrame);
      Twist desiredTwistOfSwingFootWithRespectToStanceFoot = new Twist(footFrame, worldFrame, footFrame, desiredFootVelocity.getVector(),
                                                                desiredFootAngularVelocity.getVector());

      return desiredTwistOfSwingFootWithRespectToStanceFoot;
   }

   private SpatialAccelerationVector computeDesiredSwingFootSpatialAcceleration(ReferenceFrame elevatorFrame, ReferenceFrame footFrame,
           FrameVector desiredSwingFootAcceleration, FrameVector desiredSwingFootAngularAcceleration)
   {
      //TODO: Error below. Can't just changeFrame. The accelerations will be different and depend on rotational velocity.
      // Maybe use a SixDoFJointSpatialAccelerationCalculator...
      
      desiredSwingFootAcceleration.changeFrame(footFrame);
      desiredSwingFootAngularAcceleration.changeFrame(footFrame);
      SpatialAccelerationVector desiredAccelerationOfSwingFootWithRespectToWorld = new SpatialAccelerationVector(footFrame, elevatorFrame, footFrame,
                                                                                      desiredSwingFootAcceleration.getVector(),
                                                                                      desiredSwingFootAngularAcceleration.getVector());

      return desiredAccelerationOfSwingFootWithRespectToWorld;
   }

   private static RigidBodyTransform createTransform(FrameOrientation orientation, FramePoint framePoint)
   {
      orientation.checkReferenceFrameMatch(framePoint);
      Matrix3d rotationMatrix = orientation.getMatrix3dCopy();
      RigidBodyTransform ret = new RigidBodyTransform(rotationMatrix, new Vector3d(framePoint.getPoint()));

      return ret;
   }

//   private double getPercentScalingBasedOnJacobianDeterminant(double jacobianDeterminant)
//   {
//      double determinantThresholdOne = 0.06;    // 0.05;    // 0.025;
//      double determinantThresholdTwo = 0.03;    // 0.02; //0.01;
//
//      double percent = (Math.abs(jacobianDeterminant) - determinantThresholdTwo) / (determinantThresholdOne - determinantThresholdTwo);
//      percent = MathTools.clipToMinMax(percent, 0.0, 1.0);
//
//      return percent;
//   }

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
      useBodyAcceleration = false;

//      kpGains.get(LegJointName.HIP_YAW).set(500.0);
//      kdGains.get(LegJointName.HIP_YAW).set(20.0);
//      
//      kpGains.get(LegJointName.HIP_ROLL).set(200.0);
//      kdGains.get(LegJointName.HIP_ROLL).set(20.0);
//      
//      kpGains.get(LegJointName.HIP_PITCH).set(200.0);
//      kdGains.get(LegJointName.HIP_PITCH).set(20.0);
//      
//      kpGains.get(LegJointName.KNEE).set(200.0);
//      kdGains.get(LegJointName.KNEE).set(20.0);
//      
//      kpGains.get(LegJointName.ANKLE_PITCH).set(1000.0);
//      kdGains.get(LegJointName.ANKLE_PITCH).set(20.0);
//
//      kpGains.get(LegJointName.ANKLE_ROLL).set(1000.0);
//      kdGains.get(LegJointName.ANKLE_ROLL).set(20.0);
      
      masterKpGain.set(50.0);
      masterKdGain.set(2.0);
      
      softScaleFactor.set(0.1); // 0.25);
      dampedLeastSquaresAlpha.set(0.05);
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

}
