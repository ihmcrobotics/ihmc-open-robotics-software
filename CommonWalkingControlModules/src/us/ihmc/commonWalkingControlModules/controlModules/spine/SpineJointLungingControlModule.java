package us.ihmc.commonWalkingControlModules.controlModules.spine;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.List;
import java.util.ListIterator;

import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.SpineControlModule;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.R2SpineLinkName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.SpineJointName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.SpineTorques;
import us.ihmc.commonWalkingControlModules.sensors.ProcessedSensorsInterface;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.containers.ContainerTools;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.InverseDynamicsCalculator;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.utilities.screwTheory.Wrench;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.PIDController;

public class SpineJointLungingControlModule implements SpineControlModule
{
   private final YoVariableRegistry registry = new YoVariableRegistry("SpineJointLungingControlModule");

   private final EnumMap<SpineJointName, DoubleYoVariable> desiredAngles = ContainerTools.createEnumMap(SpineJointName.class);
   private final EnumMap<SpineJointName, PIDController> spineControllers = ContainerTools.createEnumMap(SpineJointName.class);

//   private final EnumMap<SpineJointName, DoubleYoVariable> actualAngles;
//   private final EnumMap<SpineJointName, DoubleYoVariable> actualAngleVelocities;

   private final ProcessedSensorsInterface processedSensors;
   private final double controlDT;
   
   private InverseDynamicsCalculator spineJointIDCalc;
   
   private Vector3d desiredSpineTorqueVector;
 
   public SpineJointLungingControlModule(ProcessedSensorsInterface processedSensors, InverseDynamicsCalculator spineJointIDCalc, double controlDT, YoVariableRegistry parentRegistry)
   {
      this.spineJointIDCalc = spineJointIDCalc;
      this.processedSensors = processedSensors;
      this.controlDT = controlDT;
      
      populateYoVariables();
      populateControllers();
      setDesireds();
      setGains();
      parentRegistry.addChild(registry);

//      actualAngles = processedSensors.getYoSpineJointPositions();
//      actualAngleVelocities = processedSensors.getYoSpineJointVelocities();
   }

   public void doSpineControl(SpineTorques spineTorquesToPack)
   {
      spineTorquesToPack.setTorquesToZero();
      
//      spineJointIDCalc.setExternalWrench(rigidBody, externalWrench)
      
      
      
      //TODO: Set Spine Pitch,Roll,Yaw torques to create desiredSpineTorqueVector here 
      // (could use inverse dynamics with virtual external wrench that mirrors desiredSpineTorqueVector)
      
      // control
      for (SpineJointName spineJointName : SpineJointName.values())
      {
         PIDController pidController = spineControllers.get(spineJointName);
         
         double desiredPosition = desiredAngles.get(spineJointName).getDoubleValue();
         double desiredVelocity = 0.0;

         double actualPosition = processedSensors.getSpineJointPosition(spineJointName); // actualAngles.get(spineJointName).getDoubleValue();
         double actualVelocity = processedSensors.getSpineJointVelocity(spineJointName); //actualAngleVelocities.get(spineJointName).getDoubleValue();

         double torque = pidController.compute(actualPosition, desiredPosition, actualVelocity, desiredVelocity, controlDT);
         spineTorquesToPack.setTorque(spineJointName, torque);
      }
   }

   private void setHipTorqueFromDeltaCMP(Vector2d deltaCMP, SpineTorques spineTorquesToPack)
   {
//      double mass = LIPMWithReactionMassParameters.getMass();
//      double gravity = LIPMWithReactionMassParameters.getGravity();
//      hipTorqueVector.setY(mass * gravity * deltaCMP.getX());
//      hipTorqueVector.setX(mass * gravity * -deltaCMP.getY());
      
      spineTorquesToPack.setTorque(SpineJointName.SPINE_PITCH, deltaCMP.getX());
      spineTorquesToPack.setTorque(SpineJointName.SPINE_ROLL, -deltaCMP.getY());
   }
   

   private void populateYoVariables()
   {
      for (SpineJointName spineJointName : SpineJointName.values())
      {
         String name = "desired" + spineJointName.getCamelCaseNameForMiddleOfExpression();
         DoubleYoVariable variable = new DoubleYoVariable(name, registry);
         desiredAngles.put(spineJointName, variable);
      }
   }

   private void populateControllers()
   {
      for (SpineJointName spineJointName : SpineJointName.values())
      {
         spineControllers.put(spineJointName, new PIDController(spineJointName.getCamelCaseNameForStartOfExpression(), registry));
      }
   }

   private void setDesireds()
   {
      /*
       * 100610 pdn: I tried setting this to 0.1 immediately and the robot fell
       * I could start off with it 0.05 and once it got walking, change it to 0.1
       */
      desiredAngles.get(SpineJointName.SPINE_PITCH).set(0.0);
   }

   private void setGains()
   {
      spineControllers.get(SpineJointName.SPINE_YAW).setProportionalGain(3000.0);
      spineControllers.get(SpineJointName.SPINE_PITCH).setProportionalGain(3000.0);
      spineControllers.get(SpineJointName.SPINE_ROLL).setProportionalGain(3000.0);

      spineControllers.get(SpineJointName.SPINE_YAW).setDerivativeGain(200.0);
      spineControllers.get(SpineJointName.SPINE_PITCH).setDerivativeGain(200.0);
      spineControllers.get(SpineJointName.SPINE_ROLL).setDerivativeGain(200.0);
   }
   

   
   
}

