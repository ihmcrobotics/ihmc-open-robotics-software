package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import java.util.ArrayList;
import java.util.LinkedHashMap;

import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointName;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.robotSide.SideDependentList;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.InverseDynamicsCalculator;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.RevoluteJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.ScrewTools;
import us.ihmc.utilities.screwTheory.SixDoFJoint;
import us.ihmc.utilities.screwTheory.SpatialAccelerationVector;
import us.ihmc.utilities.screwTheory.TotalMassCalculator;
import us.ihmc.utilities.screwTheory.TwistCalculator;
import us.ihmc.utilities.screwTheory.Wrench;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.SimulationConstructionSet;
import com.yobotics.simulationconstructionset.VariableChangedListener;
import com.yobotics.simulationconstructionset.YoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.GainCalculator;
import com.yobotics.simulationconstructionset.util.PDController;
import com.yobotics.simulationconstructionset.util.inputdevices.SliderBoardConfigurationManager;
import com.yobotics.simulationconstructionset.util.statemachines.State;

/**
 * Simple controller using an inverse dynamics calculator. Mainly used to check gravity compensation, and simple controls while having the robot hanging in the air.
 * @author Plenty of people :)
 */
public class InverseDynamicsJointController extends State<HighLevelState>
{
   private static final String CONTROLLER_PREFIX = "gravityComp_";

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   
   /** Simply add an external wrench on each foot to compensate for the robot weight when true */
   private static final boolean STAND_ON_FEET = false;
   
   /** Specify if the controller uses the mass matrix for the PD controllers */
   private static final boolean USE_MASS_MATRIX = true;
   
   /** Compensate for Coriolis and centrifugal terms */
   private static final boolean COMPENSATE_BIAS = false;
   
   private final TwistCalculator twistCalculator;
   private final InverseDynamicsCalculator inverseDynamicsCalculator;
   
   private final FullRobotModel fullRobotModel;
   private final SixDoFJoint rootJoint;
   private final SideDependentList<RigidBody> feet = new SideDependentList<>();

   private final InverseDynamicsJoint[] allJoints;
   private final RevoluteJoint[] allRevoluteJoints;
   private final SideDependentList<Wrench> footWrenches = new SideDependentList<>();
   
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   
   private final double totalMass, gravityZ;
   private final FrameVector weightVector = new FrameVector();
   
   private final DoubleYoVariable percentOfGravityCompensation = new DoubleYoVariable("percentOfGravityCompensation", registry);

   private final SpatialAccelerationVector desiredRootJointAcceleration;
   private final FrameVector desiredVerticalAccelerationVector = new FrameVector();

   private final DoubleYoVariable allJointGains = new DoubleYoVariable("gravityComp_allJointGains", registry);
   private final DoubleYoVariable allJointZetas = new DoubleYoVariable("gravityComp_allJointZetas", registry);
   
   private final LinkedHashMap<RevoluteJoint, PDController> pdControllers = new LinkedHashMap<>();
   private final LinkedHashMap<RevoluteJoint, DoubleYoVariable> desiredJointPositions = new LinkedHashMap<>();
   private final LinkedHashMap<RevoluteJoint, DoubleYoVariable> pdControllerOutputs = new LinkedHashMap<>();

   private final LinkedHashMap<RevoluteJoint, DoubleYoVariable> individualJointGainScalingMap = new LinkedHashMap<>();
   
   private final DoubleYoVariable gainScaling = new DoubleYoVariable("gravityComp_gainScaling", registry);
   private final DoubleYoVariable footForceScaling = new DoubleYoVariable("gravityComp_footForceScaling", registry);
   
   public InverseDynamicsJointController(FullRobotModel fullRobotModel, TwistCalculator twistCalculator, double gravityZ)
   {
      super(HighLevelState.INVERSE_DYNAMICS_JOINT_CONTROL);
      
      gainScaling.set(0.0);
      gainScaling.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable v)
         {
            gainScaling.set(MathTools.clipToMinMax(gainScaling.getDoubleValue(), 0.0, 1.0));
         }
      });
      
      this.fullRobotModel = fullRobotModel;
      
      this.gravityZ = gravityZ;
      this.percentOfGravityCompensation.set(0.0);
      percentOfGravityCompensation.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable v)
         {
            percentOfGravityCompensation.set(MathTools.clipToMinMax(percentOfGravityCompensation.getDoubleValue(), -0.3, 1.3));
         }
      });
      
      this.footForceScaling.set(0.0);
      footForceScaling.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable v)
         {
            footForceScaling.set(MathTools.clipToMinMax(footForceScaling.getDoubleValue(), 0.0, 1.0));
         }
      });
      
      
      rootJoint = fullRobotModel.getRootJoint();
      desiredRootJointAcceleration = new SpatialAccelerationVector(rootJoint.getFrameAfterJoint(), rootJoint.getFrameBeforeJoint(), rootJoint.getFrameAfterJoint());
      
      this.twistCalculator = twistCalculator;
      SpatialAccelerationVector rootAcceleration = ScrewTools.createGravitationalSpatialAcceleration(twistCalculator.getRootBody(), 0.0);
      LinkedHashMap<RigidBody, Wrench> externalWrenches = new LinkedHashMap<RigidBody, Wrench>();
      this.inverseDynamicsCalculator = new InverseDynamicsCalculator(worldFrame, rootAcceleration, externalWrenches,
            new ArrayList<InverseDynamicsJoint>(), COMPENSATE_BIAS, false, twistCalculator);
   
      this.totalMass = TotalMassCalculator.computeSubTreeMass(fullRobotModel.getElevator());
      
      allJoints = ScrewTools.computeSupportAndSubtreeJoints(rootJoint.getSuccessor());
      allRevoluteJoints = ScrewTools.filterJoints(allJoints, RevoluteJoint.class);
      
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody foot = fullRobotModel.getFoot(robotSide);
         feet.put(robotSide, foot);
         footWrenches.put(robotSide, new Wrench(foot.getBodyFixedFrame(), foot.getBodyFixedFrame()));
      }
      
      for (int i = 0; i < allRevoluteJoints.length; i++)
      {
         RevoluteJoint revoluteJoint = allRevoluteJoints[i];

         String prefix = CONTROLLER_PREFIX + revoluteJoint.getName();
         
         PDController pdController = new PDController(prefix, registry);
         pdControllers.put(revoluteJoint, pdController);

         DoubleYoVariable desiredJointPosition = new DoubleYoVariable(prefix + "_q_d", registry);
         desiredJointPositions.put(revoluteJoint, desiredJointPosition);
         
         DoubleYoVariable pdControllerOutput = new DoubleYoVariable(prefix + "_PDOutput", registry);
         pdControllerOutputs.put(revoluteJoint, pdControllerOutput);
      }
      
      VariableChangedListener gainsChangedListener = new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable v)
         {
            for (int i = 0; i < allRevoluteJoints.length; i++)
            {
               if (!USE_MASS_MATRIX)
               {
                  allJointGains.set(MathTools.clipToMinMax(allJointGains.getDoubleValue(), 0.0, 3.0));
                  allJointZetas.set(MathTools.clipToMinMax(allJointZetas.getDoubleValue(), 0.0, 0.3));
               }
               
               RevoluteJoint revoluteJoint = allRevoluteJoints[i];
               PDController pdController = pdControllers.get(revoluteJoint);
               double individualJointGainScaling = individualJointGainScalingMap.get(revoluteJoint).getDoubleValue();
               
               double kp = allJointGains.getDoubleValue() * gainScaling.getDoubleValue() * individualJointGainScaling;
               double kd;
               
               if (USE_MASS_MATRIX)
               {
                  kd = GainCalculator.computeDerivativeGain(kp, allJointZetas.getDoubleValue());
               }
               else
               {
                  double mass = TotalMassCalculator.computeSubTreeMass(revoluteJoint.getSuccessor());
                  kp *= mass;
                  kd = GainCalculator.computeDampingForSecondOrderSystem(mass, kp, allJointZetas.getDoubleValue());
               }
               
               pdController.setProportionalGain(kp);
               pdController.setDerivativeGain(kd);
            }
         }
      };
      
      if (USE_MASS_MATRIX)
      {
         allJointGains.set(2.0);
         allJointZetas.set(0.02);
      }
      else
      {
         allJointGains.set(1.0);
         allJointZetas.set(0.02);
      }
      allJointGains.addVariableChangedListener(gainsChangedListener);
      allJointZetas.addVariableChangedListener(gainsChangedListener);
      gainScaling.addVariableChangedListener(gainsChangedListener);

      for (int i = 0; i < allRevoluteJoints.length; i++)
      {
         RevoluteJoint revoluteJoint = allRevoluteJoints[i];
         String prefix = CONTROLLER_PREFIX + revoluteJoint.getName();

         DoubleYoVariable individualJointGainScaling = new DoubleYoVariable(prefix + "_gainScaling", registry);
         individualJointGainScaling.set(1.0);
         individualJointGainScaling.addVariableChangedListener(gainsChangedListener);
         individualJointGainScalingMap.put(revoluteJoint, individualJointGainScaling);
      }
      
      gainsChangedListener.variableChanged(null);
   }

   private final DoubleYoVariable test_qd_LeftHiptPitch = new DoubleYoVariable("test_qd_LeftHiptPitch", registry);
   
   @Override
   public void doAction()
   {
      inverseDynamicsCalculator.reset();
      for(InverseDynamicsJoint joint : allJoints)
      {
         joint.setDesiredAccelerationToZero();
      }
      
      if (USE_MASS_MATRIX)
      {
         doPDControl();
         
         for (int i = 0; i < allRevoluteJoints.length; i++)
         {
            RevoluteJoint revoluteJoint = allRevoluteJoints[i];
            
            double qddDesired = pdControllerOutputs.get(revoluteJoint).getDoubleValue();
            revoluteJoint.setQddDesired(qddDesired);
         }
      }
      
      test_qd_LeftHiptPitch.set(fullRobotModel.getLegJointVelocities(RobotSide.LEFT).getJointVelocity(LegJointName.HIP_PITCH));
      
      desiredVerticalAccelerationVector.set(worldFrame, 0.0, 0.0, percentOfGravityCompensation.getDoubleValue() * gravityZ);
      desiredVerticalAccelerationVector.changeFrame(rootJoint.getFrameAfterJoint());
      desiredRootJointAcceleration.setLinearPart(desiredVerticalAccelerationVector.getVector());
      fullRobotModel.getRootJoint().setDesiredAcceleration(desiredRootJointAcceleration);
      
      if (STAND_ON_FEET)
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            RigidBody foot = feet.get(robotSide);

            weightVector.set(worldFrame, 0.0, 0.0, totalMass * gravityZ / 2.0 * footForceScaling.getDoubleValue());
            weightVector.changeFrame(foot.getBodyFixedFrame());

            Wrench footWrench = footWrenches.get(robotSide);
            footWrench.setLinearPart(weightVector.getVector());

            inverseDynamicsCalculator.setExternalWrench(foot, footWrench);
         }
      }
      
      twistCalculator.compute();
      inverseDynamicsCalculator.compute();

      if (!USE_MASS_MATRIX)
      {
         doPDControl();

         for (int i = 0; i < allRevoluteJoints.length; i++)
         {
            RevoluteJoint revoluteJoint = allRevoluteJoints[i];
            double tauFromPDControl = pdControllerOutputs.get(revoluteJoint).getDoubleValue();
            revoluteJoint.setTau(revoluteJoint.getTau() + tauFromPDControl);
         }
      }
   }

   private void doPDControl()
   {
      for (int i = 0; i < allRevoluteJoints.length; i++)
      {
         RevoluteJoint revoluteJoint = allRevoluteJoints[i];
         PDController pdController = pdControllers.get(revoluteJoint);
         
         double q = revoluteJoint.getQ();
         double q_d = desiredJointPositions.get(revoluteJoint).getDoubleValue();
         double qd = revoluteJoint.getQd();
         double qd_d = 0.0;
         
         pdControllerOutputs.get(revoluteJoint).set(pdController.compute(q, q_d, qd, qd_d));
      }
   }

   @Override
   public void doTransitionIntoAction()
   {
      for (int i = 0; i < allRevoluteJoints.length; i++)
      {
         RevoluteJoint revoluteJoint = allRevoluteJoints[i];
         DoubleYoVariable q_d = desiredJointPositions.get(revoluteJoint);
         q_d.set(revoluteJoint.getQ());
      }
   }

   @Override
   public void doTransitionOutOfAction()
   {
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public static class GravityCompensationSliderBoard
   {
      private enum SliderBoardMode
      {
         Gains, LeftLegDesireds, RightLegDesireds, LeftArmDesireds, RightArmDesireds, SpineNeckDesireds;

         public static final SideDependentList<SliderBoardMode> legDesireds = new SideDependentList<>(LeftLegDesireds, RightLegDesireds);

         public static final SideDependentList<SliderBoardMode> armDesireds = new SideDependentList<>(LeftArmDesireds, RightArmDesireds);
      };

      public GravityCompensationSliderBoard(SimulationConstructionSet scs, FullRobotModel fullRobotModel, YoVariableRegistry registry)
      {
         this(scs, fullRobotModel, registry, null, 0.0, 0.0);
      }
      
      public GravityCompensationSliderBoard(SimulationConstructionSet scs, FullRobotModel fullRobotModel, YoVariableRegistry registry, String varNameForLastSlider, double varMinValue, double varMaxValue)
      {
         final EnumYoVariable<SliderBoardMode> sliderBoardMode = new EnumYoVariable<SliderBoardMode>("sliderBoardMode", registry, SliderBoardMode.class);
         final SliderBoardConfigurationManager sliderBoardConfigurationManager = new SliderBoardConfigurationManager(scs);

         sliderBoardConfigurationManager.setSlider(1, "percentOfGravityCompensation", registry, 0.0, 1.0);
         sliderBoardConfigurationManager.setSlider(2, "gravityComp_gainScaling", registry, 0.0, 1.0);
         int maxNumberOfDofs = 7;
         double maxGains = 3.0;
         double maxZetas = 0.3;
         if (USE_MASS_MATRIX)
         {
            maxGains = 100.0;
            maxZetas = 1.0;
         }
         sliderBoardConfigurationManager.setSlider(3, "gravityComp_allJointGains", registry, 0.0, maxGains);
         sliderBoardConfigurationManager.setSlider(4, "gravityComp_allJointZetas", registry, 0.0, maxZetas);
         
         if (STAND_ON_FEET)
         {
            sliderBoardConfigurationManager.setSlider(5, "gravityComp_footForceScaling", registry, 0.0, 1.0);
         }
         
         if (varNameForLastSlider != null)
            sliderBoardConfigurationManager.setSlider(8, varNameForLastSlider, registry, varMinValue, varMaxValue);
         sliderBoardConfigurationManager.setKnob  (8, "sliderBoardMode", registry, 0.0, SliderBoardMode.values().length);
         sliderBoardConfigurationManager.saveConfiguration(SliderBoardMode.Gains.toString());
         sliderBoardConfigurationManager.clearControls();

         for (RobotSide robotSide : RobotSide.values)
         {
            RevoluteJoint[] legRevoluteJoints = ScrewTools.filterJoints(ScrewTools.createJointPath(fullRobotModel.getPelvis(), fullRobotModel.getFoot(robotSide)), RevoluteJoint.class);

            setupJointSlidersAndKnobs(registry, sliderBoardConfigurationManager, maxNumberOfDofs, legRevoluteJoints);

            if (varNameForLastSlider != null)
               sliderBoardConfigurationManager.setSlider(8, varNameForLastSlider, registry, varMinValue, varMaxValue);
            sliderBoardConfigurationManager.setKnob  (8, "sliderBoardMode", registry, 0.0, SliderBoardMode.values().length);
            sliderBoardConfigurationManager.saveConfiguration(SliderBoardMode.legDesireds.get(robotSide).toString());
            sliderBoardConfigurationManager.clearControls();
         }

         for (RobotSide robotSide : RobotSide.values)
         {
            RevoluteJoint[] armRevoluteJoints = ScrewTools.filterJoints(ScrewTools.createJointPath(fullRobotModel.getChest(), fullRobotModel.getHand(robotSide)), RevoluteJoint.class);

            setupJointSlidersAndKnobs(registry, sliderBoardConfigurationManager, maxNumberOfDofs, armRevoluteJoints);

            if (varNameForLastSlider != null)
               sliderBoardConfigurationManager.setSlider(8, varNameForLastSlider, registry, varMinValue, varMaxValue);
            sliderBoardConfigurationManager.setKnob  (8, "sliderBoardMode", registry, 0.0, SliderBoardMode.values().length);
            sliderBoardConfigurationManager.saveConfiguration(SliderBoardMode.armDesireds.get(robotSide).toString());
            sliderBoardConfigurationManager.clearControls();
         }

         RevoluteJoint[] spineAndNeckRevoluteJoints = ScrewTools.filterJoints(ScrewTools.createJointPath(fullRobotModel.getPelvis(), fullRobotModel.getHead()), RevoluteJoint.class);

         setupJointSlidersAndKnobs(registry, sliderBoardConfigurationManager, maxNumberOfDofs, spineAndNeckRevoluteJoints);

         if (varNameForLastSlider != null)
            sliderBoardConfigurationManager.setSlider(8, varNameForLastSlider, registry, varMinValue, varMaxValue);
         sliderBoardConfigurationManager.setKnob  (8, "sliderBoardMode", registry, 0.0, SliderBoardMode.values().length);
         sliderBoardConfigurationManager.saveConfiguration(SliderBoardMode.SpineNeckDesireds.toString());
         sliderBoardConfigurationManager.clearControls();
         

         VariableChangedListener listener = new VariableChangedListener()
         {
            @Override
            public void variableChanged(YoVariable v)
            {
                  sliderBoardConfigurationManager.loadConfiguration(sliderBoardMode.getEnumValue().toString());
            }
         };

         sliderBoardMode.addVariableChangedListener(listener);
//         listener.variableChanged(null);
         sliderBoardMode.set(SliderBoardMode.Gains);
         sliderBoardConfigurationManager.loadConfiguration(SliderBoardMode.Gains.toString());
      }

      private void setupJointSlidersAndKnobs(YoVariableRegistry registry, final SliderBoardConfigurationManager sliderBoardConfigurationManager,
            int maxNumberOfDofs, RevoluteJoint[] revoluteJoints)
      {
         for (int i = 0; i < revoluteJoints.length; i++)
         {
            if (i > maxNumberOfDofs)
            {
               System.err.println("Too many joints for the slider board. Last joint configured: " + revoluteJoints[i-1].getName());
               break;
            }
            
            RevoluteJoint revoluteJoint = revoluteJoints[i];
            String q_d_varName = CONTROLLER_PREFIX + revoluteJoint.getName() + "_q_d";
            sliderBoardConfigurationManager.setSlider(i+1, q_d_varName, registry, revoluteJoint.getJointLimitLower(), revoluteJoint.getJointLimitUpper());
            
            String jointGainScalingVarName = CONTROLLER_PREFIX + revoluteJoint.getName() + "_gainScaling";
            sliderBoardConfigurationManager.setKnob(i+1, jointGainScalingVarName, registry, 0.0, 1.0);
         }
      }
   }
}
