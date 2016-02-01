package us.ihmc.commonWalkingControlModules.controlModules.upperBody;

import java.util.EnumMap;

import us.ihmc.SdfLoader.models.FullRobotModel;
import us.ihmc.SdfLoader.partNames.NeckJointName;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.ArmControlModule;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.SpineControlModule;
import us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait.UpperBodySubController;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.NeckTorques;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.UpperBodyTorques;
import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.sensorProcessing.ProcessedSensorsInterface;
import us.ihmc.tools.containers.ContainerTools;


public class SimpleUpperBodySubController implements UpperBodySubController
{
   private final ProcessedSensorsInterface processedSensors;
   private final YoVariableRegistry registry = new YoVariableRegistry("SimpleUpperBodySubController");

   private final ArmControlModule armControlModule;
   private final SpineControlModule spineControlModule;

   private final EnumMap<NeckJointName, PIDController> neckControllers = ContainerTools.createEnumMap(NeckJointName.class);
   private final EnumMap<NeckJointName, DoubleYoVariable> desiredNeckPositions = ContainerTools.createEnumMap(NeckJointName.class);

   private final NeckJointName[] neckJointNames;
   private final double controlDT;

   public SimpleUpperBodySubController(FullRobotModel fullRobotModel, ProcessedSensorsInterface processedSensors, double controlDT, ArmControlModule armControlModule, SpineControlModule spineControlModule,
           YoVariableRegistry parentRegistry)
   {
      this.processedSensors = processedSensors;
      this.controlDT = controlDT;
      this.armControlModule = armControlModule;
      this.spineControlModule = spineControlModule;

      neckJointNames = fullRobotModel.getRobotSpecificJointNames().getNeckJointNames();

      populateYoVariables();

      populateControllers();
      parentRegistry.addChild(registry);


      setGains();
   }



   public void doUpperBodyControl(UpperBodyTorques upperBodyTorquesToPack)
   {
      armControlModule.doArmControl(upperBodyTorquesToPack.getArmTorques());

      upperBodyTorquesToPack.setNeckTorques(doNeckControl());

      spineControlModule.doSpineControl(upperBodyTorquesToPack.getSpineTorques());

      spineControlModule.doSpineControl(upperBodyTorquesToPack.getSpineTorques());
   }

   private NeckTorques doNeckControl()
   {
      NeckTorques neckTorques = new NeckTorques();

      for (NeckJointName neckJointName : neckJointNames)
      {
         PIDController pidController = neckControllers.get(neckJointName);
         double desiredPosition = desiredNeckPositions.get(neckJointName).getDoubleValue();
         double desiredVelocity = 0.0;

         double actualPosition = processedSensors.getNeckJointPosition(neckJointName);
         double actualVelcoity = processedSensors.getNeckJointVelocity(neckJointName);

         double torque = pidController.compute(actualPosition, desiredPosition, actualVelcoity, desiredVelocity, controlDT);
         neckTorques.setTorque(neckJointName, torque);
      }

      return neckTorques;
   }

   private void populateYoVariables()
   {
      for (NeckJointName neckJointName : neckJointNames)
      {
         String name = "desired" + neckJointName.getCamelCaseNameForMiddleOfExpression();
         DoubleYoVariable variable = new DoubleYoVariable(name, registry);

         desiredNeckPositions.put(neckJointName, variable);
      }
   }

   private void populateControllers()
   {
      for (NeckJointName neckJointName : neckJointNames)
      {
         neckControllers.put(neckJointName, new PIDController(neckJointName.getCamelCaseNameForStartOfExpression(), registry));
      }
   }

   private void setGains()
   {
      neckControllers.get(NeckJointName.LOWER_NECK_PITCH).setProportionalGain(100.0);
      neckControllers.get(NeckJointName.NECK_YAW).setProportionalGain(100.0);
      neckControllers.get(NeckJointName.UPPER_NECK_PITCH).setProportionalGain(100.0);

      neckControllers.get(NeckJointName.LOWER_NECK_PITCH).setDerivativeGain(5.0);
      neckControllers.get(NeckJointName.NECK_YAW).setDerivativeGain(5.0);
      neckControllers.get(NeckJointName.UPPER_NECK_PITCH).setDerivativeGain(5.0);
   }

}

