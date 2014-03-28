package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import java.util.ArrayList;
import java.util.LinkedHashMap;

import us.ihmc.commonWalkingControlModules.dynamics.FullRobotModel;
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

import com.yobotics.simulationconstructionset.BooleanYoVariable;
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

   private final DoubleYoVariable allJointGains = new DoubleYoVariable(CONTROLLER_PREFIX + "allJointGains", registry);
   private final DoubleYoVariable allJointZetas = new DoubleYoVariable(CONTROLLER_PREFIX + "allJointZetas", registry);
   
   private final LinkedHashMap<RevoluteJoint, PDController> pdControllers = new LinkedHashMap<>();
   private final LinkedHashMap<RevoluteJoint, DoubleYoVariable> desiredJointPositions = new LinkedHashMap<>();
   private final LinkedHashMap<RevoluteJoint, DoubleYoVariable> tau_d_PDCtrlMap = new LinkedHashMap<>();
   private final LinkedHashMap<RevoluteJoint, DoubleYoVariable> qdd_dMap = new LinkedHashMap<>();
   private final LinkedHashMap<RevoluteJoint, DoubleYoVariable> tau_G_Map = new LinkedHashMap<>();

   private final LinkedHashMap<RevoluteJoint, DoubleYoVariable> individualJointGainScalingMap = new LinkedHashMap<>();

   private final LinkedHashMap<RevoluteJoint, DoubleYoVariable> zetasMap = new LinkedHashMap<>();
   
   private final DoubleYoVariable gainScaling = new DoubleYoVariable(CONTROLLER_PREFIX + "gainScaling", registry);
   private final DoubleYoVariable footForceScaling = new DoubleYoVariable(CONTROLLER_PREFIX + "footForceScaling", registry);
   
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
         final RevoluteJoint revoluteJoint = allRevoluteJoints[i];

         String prefix = CONTROLLER_PREFIX + revoluteJoint.getName();
         
         final PDController pdController = new PDController(prefix, registry);
         pdControllers.put(revoluteJoint, pdController);

         DoubleYoVariable desiredJointPosition = new DoubleYoVariable(prefix + "_q_d", registry);
         desiredJointPositions.put(revoluteJoint, desiredJointPosition);
         
         DoubleYoVariable tau_d_PDCtrl = new DoubleYoVariable(prefix + "_tau_d_PDCtrl", registry);
         tau_d_PDCtrlMap.put(revoluteJoint, tau_d_PDCtrl);
         
         DoubleYoVariable qdd_d = new DoubleYoVariable(prefix + "_qdd_d", registry);
         qdd_dMap.put(revoluteJoint, qdd_d);
         
         DoubleYoVariable tau_G = new DoubleYoVariable(prefix + "_tau_G", registry);
         tau_G_Map.put(revoluteJoint, tau_G);
         
         final DoubleYoVariable zeta = new DoubleYoVariable(prefix + "_zeta", registry);
         zeta.addVariableChangedListener(new VariableChangedListener()
         {
            @Override
            public void variableChanged(YoVariable v)
            {
               double kd;
               if (USE_MASS_MATRIX)
                  kd = GainCalculator.computeDerivativeGain(pdController.getProportionalGain(), zeta.getDoubleValue());
               else
               {
                  double mass = TotalMassCalculator.computeSubTreeMass(revoluteJoint.getSuccessor());
                  kd = GainCalculator.computeDampingForSecondOrderSystem(mass, pdController.getProportionalGain(), zeta.getDoubleValue());
               }
               pdController.setDerivativeGain(kd);
            }
         });
         zetasMap.put(revoluteJoint, zeta);
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
               
               zetasMap.get(revoluteJoint).set(allJointZetas.getDoubleValue(), false); // No need to call the changed listener
               
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

   private void initializeIDCalcultor()
   {
      inverseDynamicsCalculator.reset();
      for(InverseDynamicsJoint joint : allJoints)
         joint.setDesiredAccelerationToZero();
      
      desiredVerticalAccelerationVector.set(worldFrame, 0.0, 0.0, percentOfGravityCompensation.getDoubleValue() * gravityZ);
      desiredVerticalAccelerationVector.changeFrame(rootJoint.getFrameAfterJoint());
      desiredRootJointAcceleration.setLinearPart(desiredVerticalAccelerationVector.getVector());
      fullRobotModel.getRootJoint().setDesiredAcceleration(desiredRootJointAcceleration);
   }

   @Override
   public void doAction()
   {
      computeGravityTorquesForViz();

      initializeIDCalcultor();
      
      doPDControl();

      for (int i = 0; i < allRevoluteJoints.length; i++)
      {
         RevoluteJoint revoluteJoint = allRevoluteJoints[i];

         double qddDesired = qdd_dMap.get(revoluteJoint).getDoubleValue();
         revoluteJoint.setQddDesired(qddDesired);
      }
      
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

      for (int i = 0; i < allRevoluteJoints.length; i++)
      {
         RevoluteJoint revoluteJoint = allRevoluteJoints[i];
         double tauFromPDControl = tau_d_PDCtrlMap.get(revoluteJoint).getDoubleValue();
         revoluteJoint.setTau(revoluteJoint.getTau() + tauFromPDControl);
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
         
         if (USE_MASS_MATRIX)
         {
            qdd_dMap.get(revoluteJoint).set(pdController.compute(q, q_d, qd, qd_d));
            tau_d_PDCtrlMap.get(revoluteJoint).set(0.0);
         }
         else
         {
            qdd_dMap.get(revoluteJoint).set(0.0);
            tau_d_PDCtrlMap.get(revoluteJoint).set(pdController.compute(q, q_d, qd, qd_d));
         }
      }
   }
   
   private void computeGravityTorquesForViz()
   {
      initializeIDCalcultor();
      
      twistCalculator.compute();
      inverseDynamicsCalculator.compute();
      
      for (int i = 0; i < allRevoluteJoints.length; i++)
      {
         RevoluteJoint revoluteJoint = allRevoluteJoints[i];
         tau_G_Map.get(revoluteJoint).set(revoluteJoint.getTau());
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
         WholeBody, LeftLegDesireds, RightLegDesireds, LeftArmDesireds, RightArmDesireds, SpineNeckDesireds;

         public static final SideDependentList<SliderBoardMode> legDesireds = new SideDependentList<>(LeftLegDesireds, RightLegDesireds);

         public static final SideDependentList<SliderBoardMode> armDesireds = new SideDependentList<>(LeftArmDesireds, RightArmDesireds);
      };
      
      private enum SliderBoardSubMode {Gains, Desireds}

      private final EnumYoVariable<SliderBoardMode> sliderBoardMode;
      private final BooleanYoVariable sliderInGainsMode;
      
      private final SliderBoardConfigurationManager sliderBoardConfigurationManager;
      
      private final String lastSliderVarName;
      private final double lastSliderMinValue;
      private final double lastSliderMaxValue;
      
      private final int maxNumberOfDofs = 7;

      private final double maxGains = USE_MASS_MATRIX ? 100.0 : 3.0;
      private final double maxZetas = USE_MASS_MATRIX ? 1.0 : 0.3;
      
      private final YoVariableRegistry registry;
      
      public GravityCompensationSliderBoard(SimulationConstructionSet scs, FullRobotModel fullRobotModel, YoVariableRegistry registry)
      {
         this(scs, fullRobotModel, registry, null, 0.0, 0.0);
      }
      public GravityCompensationSliderBoard(SimulationConstructionSet scs, FullRobotModel fullRobotModel, YoVariableRegistry registry, String varNameForLastSlider, double varMinValue, double varMaxValue)
      {
         this.registry = registry;
         
         lastSliderVarName = varNameForLastSlider;
         lastSliderMinValue = varMinValue;
         lastSliderMaxValue = varMaxValue;
         
         sliderBoardMode = new EnumYoVariable<SliderBoardMode>("sliderBoardMode", registry, SliderBoardMode.class);
         final EnumYoVariable<SliderBoardSubMode> sliderBoardSubMode = new EnumYoVariable<SliderBoardSubMode>("sliderBoardSubMode", registry, SliderBoardSubMode.class);
         sliderInGainsMode = new BooleanYoVariable("sliderInGainsMode", registry);
         sliderBoardConfigurationManager = new SliderBoardConfigurationManager(scs);

         sliderBoardConfigurationManager.setSlider(1, "percentOfGravityCompensation", registry, 0.0, 1.0);
         sliderBoardConfigurationManager.setSlider(2, CONTROLLER_PREFIX + "gainScaling", registry, 0.0, 1.0);
         sliderBoardConfigurationManager.setSlider(3, CONTROLLER_PREFIX + "allJointGains", registry, 0.0, maxGains);
         sliderBoardConfigurationManager.setSlider(4, CONTROLLER_PREFIX + "allJointZetas", registry, 0.0, maxZetas);
         
         if (STAND_ON_FEET)
         {
            sliderBoardConfigurationManager.setSlider(5, CONTROLLER_PREFIX + "footForceScaling", registry, 0.0, 1.0);
         }

         SliderBoardMode currentMode = SliderBoardMode.WholeBody;
         finalizeConfiguration(currentMode, SliderBoardSubMode.Gains);

         RigidBody pelvis = fullRobotModel.getPelvis();
         for (RobotSide robotSide : RobotSide.values)
         {
            currentMode = SliderBoardMode.legDesireds.get(robotSide);
            RigidBody foot = fullRobotModel.getFoot(robotSide);
            RevoluteJoint[] legRevoluteJoints = ScrewTools.filterJoints(ScrewTools.createJointPath(pelvis, foot), RevoluteJoint.class);

            setupJointSlidersAndKnobsForDesireds(legRevoluteJoints);
            finalizeConfiguration(currentMode, SliderBoardSubMode.Desireds);
            setupJointSlidersAndKnobsForGains(legRevoluteJoints);
            finalizeConfiguration(currentMode, SliderBoardSubMode.Gains);
         }

         for (RobotSide robotSide : RobotSide.values)
         {
            currentMode = SliderBoardMode.armDesireds.get(robotSide);
            RigidBody hand = fullRobotModel.getHand(robotSide);
            RigidBody chest = fullRobotModel.getChest();
            RevoluteJoint[] armRevoluteJoints = ScrewTools.filterJoints(ScrewTools.createJointPath(chest, hand), RevoluteJoint.class);

            setupJointSlidersAndKnobsForDesireds(armRevoluteJoints);
            finalizeConfiguration(currentMode, SliderBoardSubMode.Desireds);

            setupJointSlidersAndKnobsForGains(armRevoluteJoints);
            finalizeConfiguration(currentMode, SliderBoardSubMode.Gains);
         }

         currentMode = SliderBoardMode.SpineNeckDesireds;
         RigidBody head = fullRobotModel.getHead();
         RevoluteJoint[] spineAndNeckRevoluteJoints = ScrewTools.filterJoints(ScrewTools.createJointPath(pelvis, head), RevoluteJoint.class);

         setupJointSlidersAndKnobsForDesireds(spineAndNeckRevoluteJoints);
         finalizeConfiguration(currentMode, SliderBoardSubMode.Desireds);

         setupJointSlidersAndKnobsForGains(spineAndNeckRevoluteJoints);
         finalizeConfiguration(currentMode, SliderBoardSubMode.Gains);
         

         sliderInGainsMode.addVariableChangedListener(new VariableChangedListener()
         {
            @Override
            public void variableChanged(YoVariable v)
            {
               if (sliderInGainsMode.getBooleanValue())
                  sliderBoardSubMode.set(SliderBoardSubMode.Gains);
               else
                  sliderBoardSubMode.set(SliderBoardSubMode.Desireds);
            }
         });
         

         VariableChangedListener variableChangedListener = new VariableChangedListener()
         {
            @Override
            public void variableChanged(YoVariable v)
            {
               sliderBoardConfigurationManager.loadConfiguration(sliderBoardMode.getEnumValue().toString() + sliderBoardSubMode.getEnumValue().toString());
               
               if (sliderBoardMode.getEnumValue() == SliderBoardMode.WholeBody && !sliderInGainsMode.getBooleanValue())
                  sliderInGainsMode.set(true);
            }
         };
         sliderBoardSubMode.addVariableChangedListener(variableChangedListener);
         sliderBoardMode.addVariableChangedListener(variableChangedListener);
         sliderBoardSubMode.set(SliderBoardSubMode.Gains);
         sliderBoardMode.set(SliderBoardMode.WholeBody);
         variableChangedListener.variableChanged(null);
      }

      private void setupJointSlidersAndKnobsForDesireds(RevoluteJoint[] revoluteJoints)
      {
         for (int i = 0; i < revoluteJoints.length; i++)
         {
            if (i > maxNumberOfDofs)
            {
               System.err.println("Too many joints for the slider board. Last joint configured: " + revoluteJoints[i - 1].getName());
               break;
            }

            RevoluteJoint revoluteJoint = revoluteJoints[i];
            double jointLimitLower = revoluteJoint.getJointLimitLower();
            double jointLimitUpper = revoluteJoint.getJointLimitUpper();
            
            String varPrefix = CONTROLLER_PREFIX + revoluteJoint.getName();
            String q_d_varName = varPrefix + "_q_d";
            String jointGainScalingVarName = varPrefix + "_gainScaling";

            sliderBoardConfigurationManager.setSlider(i + 1, q_d_varName, registry, jointLimitLower, jointLimitUpper);
            sliderBoardConfigurationManager.setKnob(i + 1, jointGainScalingVarName, registry, 0.0, 1.0);
         }
      }

      private void setupJointSlidersAndKnobsForGains(RevoluteJoint[] revoluteJoints)
      {
         for (int i = 0; i < revoluteJoints.length; i++)
         {
            if (i > maxNumberOfDofs)
            {
               System.err.println("Too many joints for the slider board. Last joint configured: " + revoluteJoints[i - 1].getName());
               break;
            }

            RevoluteJoint revoluteJoint = revoluteJoints[i];
            
            String varPrefix = CONTROLLER_PREFIX + revoluteJoint.getName();
            String kp_varName = "kp_" + varPrefix;
            String zeta_varName = varPrefix + "_zeta";

            double subtreeMass = TotalMassCalculator.computeSubTreeMass(revoluteJoint.getSuccessor());
            sliderBoardConfigurationManager.setSlider(i + 1, kp_varName, registry, 0.0, subtreeMass * maxGains);
            sliderBoardConfigurationManager.setKnob(i + 1, zeta_varName, registry, 0.0, maxZetas);
         }
      }

      private void finalizeConfiguration(SliderBoardMode currentMode, SliderBoardSubMode currentSubMode)
      {
         if (lastSliderVarName != null)
            sliderBoardConfigurationManager.setSlider(8, lastSliderVarName, registry, lastSliderMinValue, lastSliderMaxValue);
         sliderBoardConfigurationManager.setKnob(8, sliderBoardMode, 0.0, SliderBoardMode.values().length);
         sliderBoardConfigurationManager.setButton(1, sliderInGainsMode);
         sliderBoardConfigurationManager.saveConfiguration(currentMode.toString() + currentSubMode.toString());
         sliderBoardConfigurationManager.clearControls();
      }
   }
}
