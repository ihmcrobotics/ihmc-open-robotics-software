package us.ihmc.commonWalkingControlModules.controlModules.spine;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.List;
import java.util.ListIterator;

import javax.vecmath.Vector2d;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.SpineLungingControlModule;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.SpineControlModule;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.R2SpineLinkName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.SpineJointName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.SpineTorques;
import us.ihmc.commonWalkingControlModules.sensors.ProcessedSensorsInterface;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.containers.ContainerTools;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.InverseDynamicsCalculator;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.RevoluteJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.utilities.screwTheory.Wrench;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.PIDController;

public class SpineJointLungingControlModule implements SpineLungingControlModule
{
   private final YoVariableRegistry registry = new YoVariableRegistry("SpineJointLungingControlModule");

   private final EnumMap<SpineJointName, DoubleYoVariable> desiredAngles = ContainerTools.createEnumMap(SpineJointName.class);
   private final EnumMap<SpineJointName, PIDController> spineControllers = ContainerTools.createEnumMap(SpineJointName.class);

//   private final EnumMap<SpineJointName, DoubleYoVariable> actualAngles;
//   private final EnumMap<SpineJointName, DoubleYoVariable> actualAngleVelocities;

   private final ProcessedSensorsInterface processedSensors;
   private final double controlDT;
   
   private InverseDynamicsCalculator spineJointIDCalc;
   private RigidBody pelvisRigidBody;
   private ArrayList<RevoluteJoint> spineRevoluteJointList;
   
   private FrameVector desiredTorqueBetweenPelvisAndChest;
   private SpineTorques spineTorques = new SpineTorques();
 
   public SpineJointLungingControlModule(ProcessedSensorsInterface processedSensors, double controlDT, YoVariableRegistry parentRegistry, InverseDynamicsCalculator spineJointIDCalc, RigidBody pelvisRigidBody, ArrayList<RevoluteJoint> spineRevoluteJointList)
   {
      this.spineJointIDCalc = spineJointIDCalc;
      this.pelvisRigidBody = pelvisRigidBody;
      this.spineRevoluteJointList = spineRevoluteJointList;
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
      for (SpineJointName spineJointName : SpineJointName.values())
      {
         PIDController pidController = spineControllers.get(spineJointName);
         
         double desiredPosition = desiredAngles.get(spineJointName).getDoubleValue();
         double desiredVelocity = 0.0;

         double actualPosition = processedSensors.getSpineJointPosition(spineJointName); // actualAngles.get(spineJointName).getDoubleValue();
         double actualVelocity = processedSensors.getSpineJointVelocity(spineJointName); //actualAngleVelocities.get(spineJointName).getDoubleValue();

         double torque = pidController.compute(actualPosition, desiredPosition, actualVelocity, desiredVelocity, controlDT);
         spineTorques.setTorque(spineJointName, torque);
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
   
   public void doMaintainDesiredChestOrientation()
   {
      // TODO replace with ID control
      for (SpineJointName spineJointName : SpineJointName.values())
      {
         PIDController pidController = spineControllers.get(spineJointName);
         
         double desiredPosition = desiredAngles.get(spineJointName).getDoubleValue();
         double desiredVelocity = 0.0;

         double actualPosition = processedSensors.getSpineJointPosition(spineJointName); // actualAngles.get(spineJointName).getDoubleValue();
         double actualVelocity = processedSensors.getSpineJointVelocity(spineJointName); //actualAngleVelocities.get(spineJointName).getDoubleValue();

         double torque = pidController.compute(actualPosition, desiredPosition, actualVelocity, desiredVelocity, controlDT);
         spineTorques.setTorque(spineJointName, torque);
      }      
   }
   
   public void setWrenchOnChest(Wrench wrench)
   {
      spineTorques.setTorquesToZero();
      
      Vector3d virtualForce = new Vector3d(0.0, 0.0, 0.0);
      FrameVector virtualTorque = new FrameVector(ReferenceFrame.getWorldFrame(), 0.0, 0.0, 0.0);
      virtualTorque.changeFrame(pelvisRigidBody.getBodyFixedFrame());
      
      Wrench virtualWrenchOnPelvis = new Wrench(pelvisRigidBody.getBodyFixedFrame(), pelvisRigidBody.getBodyFixedFrame(), virtualForce, virtualTorque.getVectorCopy());
      spineJointIDCalc.setExternalWrench(pelvisRigidBody, virtualWrenchOnPelvis);
      spineJointIDCalc.compute();
      
      for (SpineJointName spineJointName : SpineJointName.values())
      {
         
         double actualPosition = processedSensors.getSpineJointPosition(spineJointName); // actualAngles.get(spineJointName).getDoubleValue();
         double actualVelocity = processedSensors.getSpineJointVelocity(spineJointName); //actualAngleVelocities.get(spineJointName).getDoubleValue();
         
         RevoluteJoint spineRevoluteJoint = spineRevoluteJointList.get(spineJointName.ordinal());
         spineRevoluteJoint.setQddDesired( -10.0*(0.0 - actualPosition) - 1.0*(0.0 - actualVelocity) );
         spineTorques.setTorque(spineJointName, spineRevoluteJoint.getTau());
      }
   }

   public void getSpineTorques(SpineTorques spineTorquesToPack)
   {
      spineTorquesToPack.setTorques(this.spineTorques.getTorquesCopy());
   }
}

