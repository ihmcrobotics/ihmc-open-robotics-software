package us.ihmc.wanderer.hardware.controllers;

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
import us.ihmc.tools.maps.EnumDoubleMap;
import us.ihmc.wanderer.hardware.WandererJoint;

public class WandererStandPrep implements WandererController
{
   public static enum StandPrepState
   {
      WAIT, INITIALIZE, EXECUTE
   }

   private final static double trajectoryTime = 3.0;

   private final YoVariableRegistry registry = new YoVariableRegistry("WandererStandPrep");

   private final DoubleYoVariable initialTime = new DoubleYoVariable("initialTime", registry);
   private final YoPolynomial trajectory = new YoPolynomial("trajectory", 4, registry);

   private final EnumMap<WandererJoint, OneDoFJoint> joints = new EnumMap<>(WandererJoint.class);
   private final EnumMap<WandererJoint, PDController> controllers = new EnumMap<>(WandererJoint.class);

   private final EnumMap<WandererStandPrepSetpoints, DoubleYoVariable> desiredPositions = new EnumMap<>(WandererStandPrepSetpoints.class);
   private final EnumMap<WandererStandPrepSetpoints, DoubleYoVariable> kps = new EnumMap<>(WandererStandPrepSetpoints.class);
   private final EnumMap<WandererStandPrepSetpoints, DoubleYoVariable> kds = new EnumMap<>(WandererStandPrepSetpoints.class);
   private final EnumMap<WandererStandPrepSetpoints, DoubleYoVariable> dampingValues = new EnumMap<>(WandererStandPrepSetpoints.class);

   private final EnumDoubleMap<WandererJoint> initialPositions = new EnumDoubleMap<>(WandererJoint.class);

   private final BooleanYoVariable startStandprep = new BooleanYoVariable("startStandprep", registry);
   private final EnumYoVariable<StandPrepState> standPrepState = new EnumYoVariable<>("standPrepState", registry, StandPrepState.class);

   private final DoubleYoVariable crouch = new DoubleYoVariable("crouch", registry);

   private final BooleanYoVariable enableOutput = new BooleanYoVariable("enableStandPrepOutput", registry);

   private double springCalibration_t0;
   private final BooleanYoVariable springCalibration_wasEnabled = new BooleanYoVariable("springCalibrationEnabled",registry);
   private final BooleanYoVariable springCalibration_isEnabled = new BooleanYoVariable("startSpringCalibration", registry);
   private final DoubleYoVariable springTime = new DoubleYoVariable("springTime",registry);

   @Override
   public void setFullRobotModel(FullRobotModel fullRobotModel)
   {
      for (WandererJoint joint : WandererJoint.values)
      {
         joints.put(joint, fullRobotModel.getOneDoFJointByName(joint.getSdfName()));
         controllers.put(joint, new PDController(joint.getSdfName(), registry));
      }

      for (WandererStandPrepSetpoints setpoint : WandererStandPrepSetpoints.values)
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

         for (int i = 0; i < WandererJoint.values.length; i++)
         {
            WandererJoint joint = WandererJoint.values[i];
            OneDoFJoint oneDoFJoint = joints.get(joint);
            oneDoFJoint.setTau(0.0);
         }

         if (startStandprep.getBooleanValue())
         {
            standPrepState.set(StandPrepState.INITIALIZE);
         }
         break;

      case INITIALIZE:
         initialTime.set(Conversions.nanoSecondstoSeconds(timestamp));
         trajectory.setCubic(0.0, trajectoryTime, 0.0, 0.0, 1.0, 0.0);

         for (WandererJoint joint : WandererJoint.values)
         {
            initialPositions.put(joint, joints.get(joint).getQ());
         }
         standPrepState.set(StandPrepState.EXECUTE);
         enableOutput.set(true);
         break;

      case EXECUTE:
         double timeInTrajectory = MathTools.clipToMinMax(Conversions.nanoSecondstoSeconds(timestamp) - initialTime.getDoubleValue(), 0, trajectoryTime);
         trajectory.compute(timeInTrajectory);
         double positionScale = trajectory.getPosition();

         for (WandererStandPrepSetpoints jointGroup : WandererStandPrepSetpoints.values)
         {
            double setpoint = desiredPositions.get(jointGroup).getDoubleValue();
            double kp = kps.get(jointGroup).getDoubleValue();
            double kd = kds.get(jointGroup).getDoubleValue();
            double damping = dampingValues.get(jointGroup).getDoubleValue();

            for (int i = 0; i < jointGroup.getJoints().length; i++)
            {
               WandererJoint joint = jointGroup.getJoints()[i];
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
               case HIP_X:
                  qDesired += springCalibrationScript(Conversions.nanoSecondstoSeconds(timestamp));
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

   private double springCalibrationScript(double time)
   {
      if(WandererStandPrepSetpoints.HIP_X.getReflectRight()!=1.0)
         return 0.0;

      if (springCalibration_isEnabled.getBooleanValue())
      {
         if(!springCalibration_wasEnabled.getBooleanValue())
         {
            springCalibration_t0 = time;
            springCalibration_wasEnabled.set(true);
            return 0.0;
         }
         else
         {
            springTime.set(time-springCalibration_t0);
            if(springTime.getDoubleValue()<20.0)
               return 0.03*(Math.abs(Math.IEEEremainder(springTime.getDoubleValue()+5.0, 20.0))-5.0); //Triangle wave with T=20.0, A=0.15
            else
            {
               springCalibration_isEnabled.set(false);
               return 0.0;
            }
         }

      }
      else
      {
         springCalibration_wasEnabled.set(false);
         return 0.0;
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
      WandererSingleThreadedController.startController(new WandererStandPrep());
   }

}
