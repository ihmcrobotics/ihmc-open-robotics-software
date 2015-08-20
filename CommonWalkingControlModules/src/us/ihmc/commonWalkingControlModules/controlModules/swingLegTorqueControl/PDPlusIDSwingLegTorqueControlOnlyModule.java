package us.ihmc.commonWalkingControlModules.controlModules.swingLegTorqueControl;

import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.SdfLoader.partNames.LegJointName;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.SwingLegTorqueControlOnlyModule;
import us.ihmc.commonWalkingControlModules.controlModules.LegJointPositionControlModule;
import us.ihmc.commonWalkingControlModules.couplingRegistry.CouplingRegistry;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointAccelerations;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointPositions;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointVelocities;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegTorques;
import us.ihmc.sensorProcessing.ProcessedSensorsInterface;
import us.ihmc.humanoidRobotics.frames.CommonHumanoidReferenceFrames;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.InverseDynamicsCalculator;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

public class PDPlusIDSwingLegTorqueControlOnlyModule implements SwingLegTorqueControlOnlyModule
{
   private final YoVariableRegistry registry = new YoVariableRegistry("PDPlusIDSwingLegTorqueControlModule");
   private final LegJointName[] legJointNames;

   private final ProcessedSensorsInterface processedSensors;
   
   private final CommonHumanoidReferenceFrames referenceFrames;
   private final FullHumanoidRobotModel fullRobotModel;
   private final CouplingRegistry couplingRegistry;
   private final SideDependentList<LegJointPositions> desiredLegJointPositions = new SideDependentList<LegJointPositions>();
   private final SideDependentList<LegJointVelocities> desiredLegJointVelocities = new SideDependentList<LegJointVelocities>();
   
   private final SideDependentList<LegJointPositionControlModule> legJointPositionControlModules;
   private final SideDependentList<InverseDynamicsCalculator> inverseDynamicsCalculators;
   
   private boolean useBodyAcceleration;

   
   public PDPlusIDSwingLegTorqueControlOnlyModule(LegJointName[] legJointNames, ProcessedSensorsInterface processedSensors,
         CommonHumanoidReferenceFrames referenceFrames, FullHumanoidRobotModel fullRobotModel, CouplingRegistry couplingRegistry,
         SideDependentList<LegJointPositionControlModule> legJointPositionControlModules, 
         SideDependentList<InverseDynamicsCalculator> inverseDynamicsCalculators, YoVariableRegistry parentRegistry)
   {
      this.legJointNames = legJointNames;
      this.processedSensors = processedSensors;
      this.referenceFrames = referenceFrames;
      this.fullRobotModel = fullRobotModel;
      this.couplingRegistry = couplingRegistry;
      this.legJointPositionControlModules = legJointPositionControlModules;
      this.inverseDynamicsCalculators = inverseDynamicsCalculators;
      
      for (RobotSide robotSide : RobotSide.values)
      {
         this.desiredLegJointPositions.put(robotSide, new LegJointPositions(robotSide));
         this.desiredLegJointVelocities.put(robotSide, new LegJointVelocities(legJointNames, robotSide));
      }
   
      
      parentRegistry.addChild(registry);
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

      
      for (LegJointName legJointName : legJointNames)
      {
         OneDoFJoint revoluteJoint = fullRobotModel.getLegJoint(swingSide, legJointName);         
         double qddDesired = jointAccelerations.getJointAcceleration(legJointName);
         revoluteJoint.setQddDesired(qddDesired);
      }
      

      double percentScaling = 1.0; 


      // control
      legJointPositionControlModules.get(swingSide).packTorquesForLegJointsPositionControl(percentScaling, legTorquesToPackForSwingLeg,
                                         desiredLegJointPositions.get(swingSide), desiredLegJointVelocities.get(swingSide));

      inverseDynamicsCalculators.get(swingSide).compute();

      for (LegJointName legJointName : legTorquesToPackForSwingLeg.getLegJointNames())
      {
         double tauInverseDynamics = fullRobotModel.getLegJoint(swingSide, legJointName).getTau();
         legTorquesToPackForSwingLeg.addTorque(legJointName, tauInverseDynamics);
      }

      setUpperBodyWrench();      
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
   }
   
   public void setAnkleGainsSoft(RobotSide swingSide)
   {
      legJointPositionControlModules.get(swingSide).setAnkleGainsSoft();      
   }
   
   public void setAnkleGainsDefault(RobotSide swingSide)
   {
      legJointPositionControlModules.get(swingSide).resetScalesToDefault();
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
   }
   
   public void setParametersForM2V2()
   {
      useBodyAcceleration = false;
   }

//   public void setDampingToZero(RobotSide swingSide)
//   {
//      legJointPositionControlModules.get(swingSide).disableJointDamping();      
//   }
}
