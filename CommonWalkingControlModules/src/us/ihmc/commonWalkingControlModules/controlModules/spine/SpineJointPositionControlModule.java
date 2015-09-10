package us.ihmc.commonWalkingControlModules.controlModules.spine;

import java.util.EnumMap;

import us.ihmc.SdfLoader.partNames.SpineJointName;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.SpineControlModule;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.SpineTorques;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.sensorProcessing.ProcessedSensorsInterface;
import us.ihmc.tools.containers.ContainerTools;
import us.ihmc.yoUtilities.controllers.PIDController;


public class SpineJointPositionControlModule implements SpineControlModule
{
   private final YoVariableRegistry registry = new YoVariableRegistry("SpineJointPositionControlModule");

   private final EnumMap<SpineJointName, DoubleYoVariable> desiredAngles = ContainerTools.createEnumMap(SpineJointName.class);
   private final EnumMap<SpineJointName, PIDController> spineControllers = ContainerTools.createEnumMap(SpineJointName.class);

//   private final EnumMap<SpineJointName, DoubleYoVariable> actualAngles;
//   private final EnumMap<SpineJointName, DoubleYoVariable> actualAngleVelocities;

   private final ProcessedSensorsInterface processedSensors;
   private final double controlDT;
   
   public SpineJointPositionControlModule(ProcessedSensorsInterface processedSensors, double controlDT, YoVariableRegistry parentRegistry)
   {
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
      
      // control
      for (SpineJointName spineJointName : SpineJointName.values)
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


   private void populateYoVariables()
   {
      for (SpineJointName spineJointName : SpineJointName.values)
      {
         String name = "desired" + spineJointName.getCamelCaseNameForMiddleOfExpression();
         DoubleYoVariable variable = new DoubleYoVariable(name, registry);
         desiredAngles.put(spineJointName, variable);
      }
   }

   private void populateControllers()
   {
      for (SpineJointName spineJointName : SpineJointName.values)
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
