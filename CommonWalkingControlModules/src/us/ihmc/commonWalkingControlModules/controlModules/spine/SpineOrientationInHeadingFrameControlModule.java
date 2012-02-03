package us.ihmc.commonWalkingControlModules.controlModules.spine;

import java.util.EnumMap;

import us.ihmc.commonWalkingControlModules.controlModuleInterfaces.SpineControlModule;
import us.ihmc.commonWalkingControlModules.desiredHeadingAndVelocity.DesiredHeadingControlModule;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.SpineJointName;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.SpineTorques;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.YawPitchOrRoll;
import us.ihmc.commonWalkingControlModules.sensors.ProcessedSensorsInterface;
import us.ihmc.utilities.containers.ContainerTools;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.Orientation;
import us.ihmc.utilities.math.geometry.ReferenceFrame;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.PDController;

public class SpineOrientationInHeadingFrameControlModule implements SpineControlModule
{
   private static final EnumMap<SpineJointName, YawPitchOrRoll> angleMap = getAngleMap();
   private final ProcessedSensorsInterface processedSensors;

   private final EnumMap<YawPitchOrRoll, DoubleYoVariable> desiredAngles = ContainerTools.createEnumMap(YawPitchOrRoll.class);
   private final EnumMap<YawPitchOrRoll, PDController> spineControllers = ContainerTools.createEnumMap(YawPitchOrRoll.class);

   private final YoVariableRegistry registry = new YoVariableRegistry("SpineOrientationInWorldControlModule");

   private final EnumMap<YawPitchOrRoll, DoubleYoVariable> actualAngles = ContainerTools.createEnumMap(YawPitchOrRoll.class);
   private final EnumMap<YawPitchOrRoll, DoubleYoVariable> actualAngleVelocities = ContainerTools.createEnumMap(YawPitchOrRoll.class);

   private final DesiredHeadingControlModule desiredHeadingControlModule;

   public SpineOrientationInHeadingFrameControlModule(ProcessedSensorsInterface processedSensors, DesiredHeadingControlModule desiredHeadingControlModule,
           YoVariableRegistry parentRegistry)
   {
      this.processedSensors = processedSensors;
      this.desiredHeadingControlModule = desiredHeadingControlModule;

      populateYoVariables();
      populateControllers();
      setDesireds();
      setGains();
      parentRegistry.addChild(registry);
   }

   public void doSpineControl(SpineTorques spineTorquesToPack)
   {
      spineTorquesToPack.setTorquesToZero();
      
      ReferenceFrame desiredHeadingFrame = desiredHeadingControlModule.getDesiredHeadingFrame();

      // get positions
      Orientation chestOrientation = processedSensors.getChestOrientationInFrame(desiredHeadingFrame);
      double[] chestYawPitchRoll = chestOrientation.getYawPitchRoll();

      actualAngles.get(YawPitchOrRoll.YAW).set(chestYawPitchRoll[0]);
      actualAngles.get(YawPitchOrRoll.PITCH).set(chestYawPitchRoll[1]);
      actualAngles.get(YawPitchOrRoll.ROLL).set(chestYawPitchRoll[2]);

      boolean JUST_DAMP_JOINT_VELOCITIES_DIRECTLY = false;

      // get velocities
      if (JUST_DAMP_JOINT_VELOCITIES_DIRECTLY)
      {
         actualAngleVelocities.get(YawPitchOrRoll.YAW).set(processedSensors.getSpineJointVelocity(SpineJointName.SPINE_YAW));
         actualAngleVelocities.get(YawPitchOrRoll.PITCH).set(processedSensors.getSpineJointVelocity(SpineJointName.SPINE_PITCH));
         actualAngleVelocities.get(YawPitchOrRoll.ROLL).set(processedSensors.getSpineJointVelocity(SpineJointName.SPINE_ROLL));
      }

      else
      {
         /*
          * Compute the chest angular velocity, and compute the yaw/pitch/roll derivatives based on it.
          * TODO/note: tk 25-06-2010 the yaw/pitch/roll derivatives should actually be computed from
          * the angular velocity of the chest frame w.r.t. the heading frame, expressed in heading frame.
          * Currently, we're using the angular velocity of the chest frame w.r.t. the *world* frame, expressed in the heading frame,
          * so we're neglecting the relative motion of the heading frame and the world frame.
          * But that should be okay for now.
          */
         FrameVector chestAngularVelocityWithRespectToWorld = processedSensors.getChestAngularVelocityInChestFrame();    // angular vel. of chest w.r.t. world, expressed in chest.
         chestAngularVelocityWithRespectToWorld = chestAngularVelocityWithRespectToWorld.changeFrameCopy(desiredHeadingFrame);    // angular vel. of chest w.r.t. world, expressed in desiredHeading

         // LINE BELOW IS NOT ACTUALLY TRUE (see note above):
         FrameVector chestAngularVelocityWithRespectToDesiredHeading = new FrameVector(chestAngularVelocityWithRespectToWorld);    // angular vel. of chest w.r.t. desiredHeading, expressed in desiredHeading
         EnumMap<YawPitchOrRoll, Double> chestAngleSetDerivatives = getYawPitchRollDerivatives(chestAngularVelocityWithRespectToDesiredHeading,
                                                                       chestOrientation);

         for (YawPitchOrRoll angleName : YawPitchOrRoll.values())
         {
            actualAngleVelocities.get(angleName).set(chestAngleSetDerivatives.get(angleName));

//          actualAngleVelocities.get(angleName).set(0.0);
         }
      }


      // control
      for (SpineJointName spineJointName : SpineJointName.values())
      {
         YawPitchOrRoll angleName = angleMap.get(spineJointName);
         PDController pdController = spineControllers.get(angleName);

         double desiredPosition = desiredAngles.get(angleName).getDoubleValue();
         double desiredVelocity = 0.0;

         double actualPosition = actualAngles.get(angleName).getDoubleValue();
         double actualVelocity = actualAngleVelocities.get(angleName).getDoubleValue();

         double torque = pdController.computeForAngles(actualPosition, desiredPosition, actualVelocity, desiredVelocity);

         spineTorquesToPack.setTorque(spineJointName, torque);
      }
   }


   private void populateYoVariables()
   {
      for (YawPitchOrRoll angleName : YawPitchOrRoll.values())
      {
         String name = "desiredChest" + angleName.getCamelCaseNameForMiddleOfExpression() + "Position";
         DoubleYoVariable variable = new DoubleYoVariable(name, registry);
         desiredAngles.put(angleName, variable);
      }

      for (YawPitchOrRoll angleName : YawPitchOrRoll.values())
      {
         String name = "chest" + angleName.getCamelCaseNameForMiddleOfExpression() + "Position";
         DoubleYoVariable variable = new DoubleYoVariable(name, registry);
         actualAngles.put(angleName, variable);
      }

      for (YawPitchOrRoll angleName : YawPitchOrRoll.values())
      {
         String name = "chest" + angleName.getCamelCaseNameForMiddleOfExpression() + "Velocity";
         DoubleYoVariable variable = new DoubleYoVariable(name, registry);
         actualAngleVelocities.put(angleName, variable);
      }
   }

   private void populateControllers()
   {
      for (YawPitchOrRoll angleName : YawPitchOrRoll.values())
      {
         spineControllers.put(angleName, new PDController(angleName.getCamelCaseNameForStartOfExpression(), registry));
      }
   }

   private void setDesireds()
   {
      /*
       * 100610 pdn: I tried setting this to 0.1 immediately and the robot fell
       * I could start off with it 0.05 and once it got walking, change it to 0.1
       */
      desiredAngles.get(YawPitchOrRoll.PITCH).set(0.05);
   }

   private void setGains()
   {
      spineControllers.get(YawPitchOrRoll.YAW).setProportionalGain(3000.0);
      spineControllers.get(YawPitchOrRoll.PITCH).setProportionalGain(3000.0);
      spineControllers.get(YawPitchOrRoll.ROLL).setProportionalGain(3000.0);

      spineControllers.get(YawPitchOrRoll.YAW).setDerivativeGain(200.0);
      spineControllers.get(YawPitchOrRoll.PITCH).setDerivativeGain(200.0);
      spineControllers.get(YawPitchOrRoll.ROLL).setDerivativeGain(200.0);
   }

   private EnumMap<YawPitchOrRoll, Double> getYawPitchRollDerivatives(FrameVector angularVelocity, Orientation orientation)
   {
      angularVelocity.checkReferenceFrameMatch(orientation);

      double[] yawPitchRoll = orientation.getYawPitchRoll();
      double yaw = yawPitchRoll[0];
      double pitch = yawPitchRoll[1];

      double omegaX = angularVelocity.getX();
      double omegaY = angularVelocity.getY();
      double omegaZ = angularVelocity.getZ();

      // see matlab\AngleSetVelocities\yawPitchRoll.m
      double cYaw = Math.cos(yaw);
      double sYaw = Math.sin(yaw);
      double cPitch = Math.cos(pitch);
      double tPitch = Math.tan(pitch);

      double yawd = omegaZ + omegaX * cYaw * tPitch + omegaY * tPitch * sYaw;
      double pitchd = omegaY * cYaw - omegaX * sYaw;
      double rolld = (omegaX * cYaw) / cPitch + (omegaY * sYaw) / cPitch;

      EnumMap<YawPitchOrRoll, Double> ret = ContainerTools.createEnumMap(YawPitchOrRoll.class);
      ret.put(YawPitchOrRoll.YAW, yawd);
      ret.put(YawPitchOrRoll.PITCH, pitchd);
      ret.put(YawPitchOrRoll.ROLL, rolld);

      return ret;
   }

   private static EnumMap<SpineJointName, YawPitchOrRoll> getAngleMap()
   {
      EnumMap<SpineJointName, YawPitchOrRoll> ret = ContainerTools.createEnumMap(SpineJointName.class);
      ret.put(SpineJointName.SPINE_YAW, YawPitchOrRoll.YAW);
      ret.put(SpineJointName.SPINE_PITCH, YawPitchOrRoll.PITCH);
      ret.put(SpineJointName.SPINE_ROLL, YawPitchOrRoll.ROLL);

      return ret;
   }
}
