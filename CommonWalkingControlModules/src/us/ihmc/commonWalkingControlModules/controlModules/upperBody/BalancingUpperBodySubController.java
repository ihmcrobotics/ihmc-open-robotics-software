package us.ihmc.commonWalkingControlModules.controlModules.upperBody;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.HashMap;

import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.ArmControlModule;
import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.SpineControlModule;
import us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait.UpperBodySubController;
import us.ihmc.commonWalkingControlModules.couplingRegistry.CouplingRegistry;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.NeckJointName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.NeckTorques;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.UpperBodyTorques;
import us.ihmc.commonWalkingControlModules.sensors.ProcessedSensorsInterface;
import us.ihmc.utilities.containers.ContainerTools;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.InverseDynamicsCalculator;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.utilities.screwTheory.Wrench;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.PIDController;

public class BalancingUpperBodySubController implements UpperBodySubController
{
   private final CouplingRegistry couplingRegistry;
   private final ProcessedSensorsInterface processedSensors;
   private final YoVariableRegistry registry = new YoVariableRegistry("SimpleUpperBodySubController");

   private final ArmControlModule armControlModule;
   private final SpineControlModule spineControlModule;

   private final EnumMap<NeckJointName, PIDController> neckControllers = ContainerTools.createEnumMap(NeckJointName.class);
   private final EnumMap<NeckJointName, DoubleYoVariable> desiredNeckPositions = ContainerTools.createEnumMap(NeckJointName.class);

   private final double controlDT;
   
   public BalancingUpperBodySubController(CouplingRegistry couplingRegistry, ProcessedSensorsInterface processedSensors, double controlDT, ArmControlModule armControlModule, SpineControlModule spineControlModule,
           YoVariableRegistry parentRegistry)
   {
      this.couplingRegistry = couplingRegistry;
      this.processedSensors = processedSensors;
      this.controlDT = controlDT;
      this.armControlModule = armControlModule;
      this.spineControlModule = spineControlModule;

      populateYoVariables();

      populateControllers();
      parentRegistry.addChild(registry);


      setGains();
   }

   
   public void doUpperBodyControl(UpperBodyTorques upperBodyTorquesToPack)
   {
      armControlModule.doArmControl(upperBodyTorquesToPack.getArmTorques());

      upperBodyTorquesToPack.setNeckTorques(doNeckControl());

      FramePoint icpInWorldFrame = couplingRegistry.getCapturePointInFrame(ReferenceFrame.getWorldFrame());
      
      spineControlModule.doSpineControl(upperBodyTorquesToPack.getSpineTorques());

   }

   private NeckTorques doNeckControl()
   {
      NeckTorques neckTorques = new NeckTorques();

      for (NeckJointName neckJointName : NeckJointName.values())
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
      for (NeckJointName neckJointName : NeckJointName.values())
      {
         String name = "desired" + neckJointName.getCamelCaseNameForMiddleOfExpression();
         DoubleYoVariable variable = new DoubleYoVariable(name, registry);

         desiredNeckPositions.put(neckJointName, variable);
      }
   }

   private void populateControllers()
   {
      for (NeckJointName neckJointName : NeckJointName.values())
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

