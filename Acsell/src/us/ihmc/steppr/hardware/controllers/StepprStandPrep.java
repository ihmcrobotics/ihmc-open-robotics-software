package us.ihmc.steppr.hardware.controllers;

import java.util.EnumMap;

import us.ihmc.commons.Conversions;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.controllers.PDController;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.math.trajectories.YoPolynomial;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.steppr.hardware.StepprJoint;
import us.ihmc.tools.maps.EnumDoubleMap;

public class StepprStandPrep implements StepprController
{
   public static enum StandPrepState
   {
      WAIT, INITIALIZE, EXECUTE
   }

   private final static double trajectoryTime = 3.0;

   private final YoVariableRegistry registry = new YoVariableRegistry("StepprStandPrep");

   private final DoubleYoVariable initialTime = new DoubleYoVariable("initialTime", registry);
   private final YoPolynomial trajectory = new YoPolynomial("trajectory", 4, registry);

   private final EnumMap<StepprJoint, OneDoFJoint> joints = new EnumMap<>(StepprJoint.class);
   private final EnumMap<StepprJoint, PDController> controllers = new EnumMap<>(StepprJoint.class);

   private final EnumMap<StepprStandPrepSetpoints, DoubleYoVariable> desiredPositions = new EnumMap<>(StepprStandPrepSetpoints.class);
   private final EnumMap<StepprStandPrepSetpoints, DoubleYoVariable> kps = new EnumMap<>(StepprStandPrepSetpoints.class);
   private final EnumMap<StepprStandPrepSetpoints, DoubleYoVariable> kds = new EnumMap<>(StepprStandPrepSetpoints.class);
   private final EnumMap<StepprStandPrepSetpoints, DoubleYoVariable> dampingValues = new EnumMap<>(StepprStandPrepSetpoints.class);

   private final EnumDoubleMap<StepprJoint> initialPositions = new EnumDoubleMap<>(StepprJoint.class);

   private final BooleanYoVariable startStandprep = new BooleanYoVariable("startStandprep", registry);
   private final EnumYoVariable<StandPrepState> standPrepState = new EnumYoVariable<>("standPrepState", registry, StandPrepState.class);

   private final DoubleYoVariable crouch = new DoubleYoVariable("crouch", registry);

   private final BooleanYoVariable enableOutput = new BooleanYoVariable("enableStandPrepOutput", registry);

   @Override
   public void setFullRobotModel(FullRobotModel fullRobotModel)
   {
      for (StepprJoint joint : StepprJoint.values)
      {
         joints.put(joint, fullRobotModel.getOneDoFJointByName(joint.getSdfName()));
         controllers.put(joint, new PDController(joint.getSdfName(), registry));
      }

      for (StepprStandPrepSetpoints setpoint : StepprStandPrepSetpoints.values)
      {
         DoubleYoVariable desiredPosition = new DoubleYoVariable(setpoint.getName() + "_q_d", registry);
         DoubleYoVariable kp = new DoubleYoVariable(setpoint.getName() + "_kp", registry);
         DoubleYoVariable kd = new DoubleYoVariable(setpoint.getName() + "_kd", registry);
         DoubleYoVariable damping = new DoubleYoVariable(setpoint.getName() + "_damping", registry);

         desiredPosition.set(setpoint.getQ());
         kp.set(setpoint.getKp());
         kd.set(setpoint.getKd());
         damping.set(setpoint.getDamping());

         desiredPositions.put(setpoint, desiredPosition);
         kps.put(setpoint, kp);
         kds.put(setpoint, kd);
         dampingValues.put(setpoint, damping);
      }

   }

   @Override
   public void initialize(long timestamp)
   {
      startStandprep.set(false);
      standPrepState.set(StandPrepState.WAIT);
      enableOutput.set(false);
   }

   @Override
   public void doControl(long timestamp)
   {
      switch (standPrepState.getEnumValue())
      {
      case WAIT:

         for (int i = 0; i < StepprJoint.values.length; i++)
         {
            StepprJoint joint = StepprJoint.values[i];
            OneDoFJoint oneDoFJoint = joints.get(joint);
            oneDoFJoint.setTau(0.0);
         }

         if (startStandprep.getBooleanValue())
         {
            standPrepState.set(StandPrepState.INITIALIZE);
         }
         break;

      case INITIALIZE:
         initialTime.set(Conversions.nanosecondsToSeconds(timestamp));
         trajectory.setCubic(0.0, trajectoryTime, 0.0, 0.0, 1.0, 0.0);

         for (StepprJoint joint : StepprJoint.values)
         {
            initialPositions.put(joint, joints.get(joint).getQ());
         }
         standPrepState.set(StandPrepState.EXECUTE);
         enableOutput.set(true);
         break;

      case EXECUTE:
         double timeInTrajectory = MathTools.clamp(Conversions.nanosecondsToSeconds(timestamp) - initialTime.getDoubleValue(), 0, trajectoryTime);
         trajectory.compute(timeInTrajectory);
         double positionScale = trajectory.getPosition();

         for (StepprStandPrepSetpoints jointGroup : StepprStandPrepSetpoints.values)
         {
            double setpoint = desiredPositions.get(jointGroup).getDoubleValue();
            double kp = kps.get(jointGroup).getDoubleValue();
            double kd = kds.get(jointGroup).getDoubleValue();
            double damping = dampingValues.get(jointGroup).getDoubleValue();

            for (int i = 0; i < jointGroup.getJoints().length; i++)
            {
               StepprJoint joint = jointGroup.getJoints()[i];
               OneDoFJoint oneDoFJoint = joints.get(joint);
               PDController controller = controllers.get(joint);
               double reflectedSetpoint = setpoint;
               if (i == 1)
               {
                  reflectedSetpoint *= jointGroup.getReflectRight();
               }

               double initialPosition = initialPositions.get(joint);

               double qDesired = initialPosition + (reflectedSetpoint - initialPosition) * positionScale;

               switch (jointGroup)
               {
               case KNEE:
                  qDesired += crouch.getDoubleValue();
                  break;
               case HIP_Y:
                  qDesired -= 0.5 * crouch.getDoubleValue();
                  break;
               case ANKLE_Y:
                  qDesired -= 0.5 * crouch.getDoubleValue();
                  break;
               default:
                  break;
               }

               double qdDesired = 0;
               controller.setProportionalGain(kp);
               controller.setDerivativeGain(kd);
               double tau = controller.compute(oneDoFJoint.getQ(), qDesired, oneDoFJoint.getQd(), qdDesired);
               oneDoFJoint.setTau(tau);
               oneDoFJoint.setKd(damping);

            }
         }
         break;
      }

   }

   public StandPrepState getStandPrepState()
   {
	   return standPrepState.getEnumValue();
   }

   @Override
   public boolean turnOutputOn()
   {
      boolean enable = enableOutput.getBooleanValue();
      enableOutput.set(false); // Outputs only need to be enabled once, after that you can turn them off
      return enable;
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public static void main(String[] args)
   {
      StepprSingleThreadedController.startController(new StepprStandPrep());
   }

}
