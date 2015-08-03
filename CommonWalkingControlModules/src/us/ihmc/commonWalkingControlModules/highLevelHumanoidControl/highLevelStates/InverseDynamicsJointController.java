package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import org.apache.commons.lang3.mutable.MutableDouble;
import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controlModules.CenterOfPressureResolver;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.manipulation.individual.TrajectoryBasedNumericalInverseKinematicsCalculator;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.communication.packets.dataobjects.HighLevelState;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.plotting.Artifact;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.inputdevices.SliderBoardConfigurationManager;
import us.ihmc.robotics.humanoidRobot.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.robotics.humanoidRobot.frames.CommonHumanoidReferenceFrames;
import us.ihmc.robotics.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.MathTools;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.InverseDynamicsCalculator;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RevoluteJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.robotics.screwTheory.SpatialAccelerationVector;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.yoUtilities.controllers.GainCalculator;
import us.ihmc.yoUtilities.controllers.PDController;
import us.ihmc.yoUtilities.dataStructure.listener.VariableChangedListener;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition.GraphicType;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.math.frames.YoFrameOrientation;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;
import us.ihmc.yoUtilities.math.frames.YoFramePose;
import us.ihmc.yoUtilities.math.frames.YoFrameQuaternion;
import us.ihmc.yoUtilities.math.trajectories.ConstantPoseTrajectoryGenerator;

/**
 * Simple controller using an inverse dynamics calculator. Mainly used to check gravity compensation, and simple controls while having the robot hanging in the air.
 * @author Plenty of people :)
 */
public class InverseDynamicsJointController extends HighLevelBehavior
{
   private static final HighLevelState controllerState = HighLevelState.INGRESS_EGRESS;
   
   private static final String CONTROLLER_PREFIX = "gravityComp_";

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   /** Specify if the feet are controlled in taskspace or in jointspace */
   private static final boolean CONTROL_FEET_IN_TASKSPACE = true;
   
   /** Simply add an external wrench on each foot to compensate for the robot weight when true */
   private static final boolean STAND_ON_FEET = false;

   /** Specify if the controller uses the mass matrix for the PD controllers */
   private static final boolean USE_MASS_MATRIX = false;

   /** Compensate for Coriolis and centrifugal terms */
   private static final boolean COMPENSATE_BIAS = false;

   private final TwistCalculator twistCalculator;
   private final InverseDynamicsCalculator inverseDynamicsCalculator;

   private final FullRobotModel fullRobotModel;
   private final SixDoFJoint rootJoint;
   
   private final RigidBody pelvis;
   private final SideDependentList<RigidBody> feet = new SideDependentList<>();
   private final SideDependentList<RevoluteJoint[]> legsRevoluteJoints = new SideDependentList<>();

   private final InverseDynamicsJoint[] allJoints;
   private final RevoluteJoint[] allRevoluteJoints;
   private final SideDependentList<Wrench> footWrenches = new SideDependentList<>();

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final double totalMass, gravityZ;

   private final DoubleYoVariable percentOfGravityCompensation = new DoubleYoVariable("percentOfGravityCompensation", registry);

   private final SpatialAccelerationVector desiredRootJointAcceleration;
   private final FrameVector desiredVerticalAccelerationVector = new FrameVector();

   private final DoubleYoVariable allLowerBodyGains = new DoubleYoVariable(CONTROLLER_PREFIX + "allLowerBodyGains", registry);
   private final DoubleYoVariable allLowerBodyZetas = new DoubleYoVariable(CONTROLLER_PREFIX + "allLowerBodyZetas", registry);

   private final DoubleYoVariable allUpperBodyGains = new DoubleYoVariable(CONTROLLER_PREFIX + "allUpperBodyGains", registry);
   private final DoubleYoVariable allUpperBodyZetas = new DoubleYoVariable(CONTROLLER_PREFIX + "allUpperBodyZetas", registry);

   private final LinkedHashMap<RevoluteJoint, PDController> pdControllers = new LinkedHashMap<>();
   private final LinkedHashMap<RevoluteJoint, DoubleYoVariable> desiredJointPositions = new LinkedHashMap<>();
   private final LinkedHashMap<RevoluteJoint, DoubleYoVariable> kpMap = new LinkedHashMap<>();
   private final LinkedHashMap<RevoluteJoint, DoubleYoVariable> kpScaledMap = new LinkedHashMap<>();
   private final LinkedHashMap<RevoluteJoint, DoubleYoVariable> zetaMap = new LinkedHashMap<>();
   private final LinkedHashMap<RevoluteJoint, DoubleYoVariable> kdMap = new LinkedHashMap<>();
   private final LinkedHashMap<RevoluteJoint, DoubleYoVariable> tau_d_PDCtrlMap = new LinkedHashMap<>();
   private final LinkedHashMap<RevoluteJoint, DoubleYoVariable> qdd_dMap = new LinkedHashMap<>();
   private final LinkedHashMap<RevoluteJoint, DoubleYoVariable> tau_G_Map = new LinkedHashMap<>();
   
   private final LinkedHashMap<RevoluteJoint, DoubleYoVariable> kpLowerBodyMap = new LinkedHashMap<>();
   private final LinkedHashMap<RevoluteJoint, DoubleYoVariable> kpUpperBodyMap = new LinkedHashMap<>();
   private final LinkedHashMap<RevoluteJoint, DoubleYoVariable> zetaLowerBodyMap = new LinkedHashMap<>();
   private final LinkedHashMap<RevoluteJoint, DoubleYoVariable> zetaUpperBodyMap = new LinkedHashMap<>();

   private final SideDependentList<YoFramePose> desiredFootPoses = CONTROL_FEET_IN_TASKSPACE ? new SideDependentList<YoFramePose>() : null;
   private final SideDependentList<TrajectoryBasedNumericalInverseKinematicsCalculator> footIKCalculators = 
         CONTROL_FEET_IN_TASKSPACE ? new SideDependentList<TrajectoryBasedNumericalInverseKinematicsCalculator>() : null;
   
   private final BooleanYoVariable initializeMirrorDesireds;
   private final YoFramePose desiredMirroredFootPose;
   private final FramePose tempFramePose = new FramePose();
   
   private final Point3d mirroredXYZ = new Point3d();
   private final double[] mirroredYawPitchRoll = new double[]{0.0, 0.0, 0.0};

   private final LinkedHashMap<RevoluteJoint, DoubleYoVariable> individualJointGainScalingMap = new LinkedHashMap<>();

   private final DoubleYoVariable gainScaling = new DoubleYoVariable(CONTROLLER_PREFIX + "gainScaling", registry);
   private final DoubleYoVariable footForceScaling = new DoubleYoVariable(CONTROLLER_PREFIX + "footForceScaling", registry);

   private final DoubleYoVariable kpCoM = new DoubleYoVariable(CONTROLLER_PREFIX + "kpCoM", registry);
   private final DoubleYoVariable kdCoM = new DoubleYoVariable(CONTROLLER_PREFIX + "kdCoM", registry);

   private final FramePoint centerOfMass = new FramePoint();
   private final FrameVector centerOfMassVelocity = new FrameVector();

   private final CenterOfPressureResolver centerOfPressureResolver = new CenterOfPressureResolver();
   private final FramePoint2d tempCoP2d = new FramePoint2d();
   private final FramePoint tempCoP = new FramePoint();
   private final SideDependentList<YoFramePoint> cops = new SideDependentList<>();

   private final CommonHumanoidReferenceFrames referenceFrames;
   
   private final BipedSupportPolygons bipedSupportPolygons;
   private final SideDependentList<YoPlaneContactState> footContactStates = new SideDependentList<>();
   
   private final MomentumBasedController momentumBasedController;
   
   private final SideDependentList<BooleanYoVariable> mirroredLegGains = new SideDependentList<>();
   
   public InverseDynamicsJointController(MomentumBasedController momentumBasedController, BipedSupportPolygons bipedSupportPolygons)
   {
      super(controllerState);

      this.referenceFrames = momentumBasedController.getReferenceFrames();
      this.bipedSupportPolygons = bipedSupportPolygons;
      this.momentumBasedController = momentumBasedController;
      
      for (RobotSide robotSide : RobotSide.values)
      {
         ContactablePlaneBody contactableFoot = momentumBasedController.getContactableFeet().get(robotSide);
         footContactStates.put(robotSide, momentumBasedController.getContactState(contactableFoot));
      }
      
      gainScaling.set(0.0);
      gainScaling.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            gainScaling.set(MathTools.clipToMinMax(gainScaling.getDoubleValue(), 0.0, 1.0));
         }
      });

      this.fullRobotModel = momentumBasedController.getFullRobotModel();

      this.gravityZ = momentumBasedController.getGravityZ();
      this.percentOfGravityCompensation.set(0.0);

      this.footForceScaling.set(0.0);
      footForceScaling.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            footForceScaling.set(MathTools.clipToMinMax(footForceScaling.getDoubleValue(), 0.0, 1.0));
         }
      });

      rootJoint = fullRobotModel.getRootJoint();
      desiredRootJointAcceleration = new SpatialAccelerationVector(rootJoint.getFrameAfterJoint(), rootJoint.getFrameBeforeJoint(), rootJoint.getFrameAfterJoint());

      this.twistCalculator = momentumBasedController.getTwistCalculator();
      SpatialAccelerationVector rootAcceleration = ScrewTools.createGravitationalSpatialAcceleration(twistCalculator.getRootBody(), 0.0);
      LinkedHashMap<RigidBody, Wrench> externalWrenches = new LinkedHashMap<RigidBody, Wrench>();
      this.inverseDynamicsCalculator = new InverseDynamicsCalculator(worldFrame, rootAcceleration, externalWrenches, new ArrayList<InverseDynamicsJoint>(),
            COMPENSATE_BIAS, true, twistCalculator);

      this.totalMass = TotalMassCalculator.computeSubTreeMass(fullRobotModel.getElevator());

      allJoints = ScrewTools.computeSupportAndSubtreeJoints(rootJoint.getSuccessor());
      allRevoluteJoints = ScrewTools.filterJoints(allJoints, RevoluteJoint.class);

      pelvis = fullRobotModel.getPelvis();
      
      for (RobotSide robotSide : RobotSide.values)
      {
         RigidBody foot = fullRobotModel.getFoot(robotSide);
         feet.put(robotSide, foot);
         footWrenches.put(robotSide, new Wrench(foot.getBodyFixedFrame(), foot.getBodyFixedFrame()));

         RevoluteJoint[] legRevoluteJoints = ScrewTools.filterJoints(ScrewTools.createJointPath(pelvis, feet.get(robotSide)), RevoluteJoint.class);
         legsRevoluteJoints.put(robotSide, legRevoluteJoints);
         
         YoFramePoint cop = new YoFramePoint(CONTROLLER_PREFIX + robotSide.getCamelCaseNameForMiddleOfExpression() + "CoPDesired", worldFrame, registry);
         cops.put(robotSide, cop);
         
         YoGraphicsListRegistry yoGraphicsListRegistry = momentumBasedController.getDynamicGraphicObjectsListRegistry();
         if (yoGraphicsListRegistry != null)
         {
            Artifact artifact = new YoGraphicPosition(CONTROLLER_PREFIX + robotSide.getCamelCaseNameForMiddleOfExpression() + "CoP", cop, 0.005, YoAppearance.Brown(), GraphicType.BALL_WITH_CROSS).createArtifact();
            yoGraphicsListRegistry.registerArtifact(CONTROLLER_PREFIX + "CoP", artifact);
         }
         
         BooleanYoVariable copyGainsToOppositeSide = new BooleanYoVariable(CONTROLLER_PREFIX + "copyLegGainsTo" + robotSide.getOppositeSide().getCamelCaseNameForMiddleOfExpression() + "Side", registry);
         mirroredLegGains.put(robotSide, copyGainsToOppositeSide);
      }

      for (int i = 0; i < allRevoluteJoints.length; i++)
      {
         final RevoluteJoint revoluteJoint = allRevoluteJoints[i];

         String prefix = CONTROLLER_PREFIX + revoluteJoint.getName();

         DoubleYoVariable kp = new DoubleYoVariable(prefix + "_kp", registry);
         kpMap.put(revoluteJoint, kp);

         DoubleYoVariable kd = new DoubleYoVariable(prefix + "_kd", registry);
         kdMap.put(revoluteJoint, kd);

         DoubleYoVariable kpScaled = new DoubleYoVariable(prefix + "_kpScaled", registry);
         kpScaledMap.put(revoluteJoint, kpScaled);

         final PDController pdController = new PDController(kpScaled, kd, prefix, registry);
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
         zetaMap.put(revoluteJoint, zeta);

         DoubleYoVariable individualJointGainScaling = new DoubleYoVariable(prefix + "_gainScaling", registry);
         individualJointGainScalingMap.put(revoluteJoint, individualJointGainScaling);
      }
      
      for (RobotSide robotSide : RobotSide.values)
      {
         for (RevoluteJoint revoluteJoint : legsRevoluteJoints.get(robotSide))
         {
            kpLowerBodyMap.put(revoluteJoint, kpMap.get(revoluteJoint));
            zetaLowerBodyMap.put(revoluteJoint, zetaMap.get(revoluteJoint));
         }
      }
      
      ArrayList<RevoluteJoint> upperBodyRevoluteJoints = new ArrayList<>();
      upperBodyRevoluteJoints.addAll(Arrays.asList(allRevoluteJoints));
      for (RobotSide robotSide : RobotSide.values)
         upperBodyRevoluteJoints.removeAll(Arrays.asList(legsRevoluteJoints.get(robotSide)));
      
      for (RevoluteJoint revoluteJoint : upperBodyRevoluteJoints)
      {
         kpUpperBodyMap.put(revoluteJoint, kpMap.get(revoluteJoint));
         zetaUpperBodyMap.put(revoluteJoint, zetaMap.get(revoluteJoint));
      }
      
      if (USE_MASS_MATRIX)
      {
         allLowerBodyGains.set(2.0);
         allLowerBodyZetas.set(0.02);
         allUpperBodyGains.set(2.0);
         allUpperBodyZetas.set(0.02);
      }
      else
      {
         allLowerBodyGains.set(1.0);
         allLowerBodyZetas.set(0.02);
         allUpperBodyGains.set(1.0);
         allUpperBodyZetas.set(0.02);
      }

      kpCoM.set(200.0);
      kdCoM.set(100.0);
      
      if (CONTROL_FEET_IN_TASKSPACE)
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            String namePrefix = CONTROLLER_PREFIX + "desired" + robotSide.getCamelCaseNameForMiddleOfExpression() + "Foot";
            YoFramePoint desiredFootPosition = new YoFramePoint(namePrefix, pelvis.getBodyFixedFrame(), registry);
            final YoFrameQuaternion desiredFootQuaternion = new YoFrameQuaternion(namePrefix, pelvis.getBodyFixedFrame(), registry);
            final YoFrameOrientation desiredFootOrientation = new YoFrameOrientation(namePrefix, pelvis.getBodyFixedFrame(), registry);
            desiredFootOrientation.attachVariableChangedListener(new VariableChangedListener()
            {
               private final Quat4d localQuaternion = new Quat4d();
               @Override
               public void variableChanged(YoVariable<?> v)
               {
                  desiredFootOrientation.getQuaternion(localQuaternion);
                  desiredFootQuaternion.set(localQuaternion);
               }
            });
            YoFramePose desiredFootPose = new YoFramePose(desiredFootPosition, desiredFootOrientation);
            desiredFootPoses.put(robotSide, desiredFootPose);

            RigidBody foot = feet.get(robotSide);

            ConstantPoseTrajectoryGenerator constantPoseTrajectoryGenerator = new ConstantPoseTrajectoryGenerator(desiredFootPosition, desiredFootQuaternion);
            
            TrajectoryBasedNumericalInverseKinematicsCalculator footIKCalculator = new TrajectoryBasedNumericalInverseKinematicsCalculator(robotSide.getCamelCaseNameForStartOfExpression() + "Foot", pelvis, foot, momentumBasedController.getControlDT(),
                  twistCalculator, new YoVariableRegistry("dummy"), null);
            footIKCalculator.setTrajectory(constantPoseTrajectoryGenerator, constantPoseTrajectoryGenerator, foot.getBodyFixedFrame());

            footIKCalculators.put(robotSide, footIKCalculator);
         }
         
         initializeMirrorDesireds = new BooleanYoVariable(CONTROLLER_PREFIX + "initializeMirrorDesireds", registry);
         desiredMirroredFootPose = new YoFramePose(CONTROLLER_PREFIX + "desriedMirroredFoot", "", pelvis.getBodyFixedFrame(), registry);
      }
      else
      {
         initializeMirrorDesireds = null;
         desiredMirroredFootPose = null;
      }
      
      setupVariableChangedListeners();
   }

   private void setupVariableChangedListeners()
   {
      allLowerBodyGains.addVariableChangedListener(createMapUpdater(kpLowerBodyMap, allLowerBodyGains));
      allLowerBodyZetas.addVariableChangedListener(createMapUpdater(zetaLowerBodyMap, allLowerBodyZetas));
      allUpperBodyGains.addVariableChangedListener(createMapUpdater(kpUpperBodyMap, allUpperBodyGains));
      allUpperBodyZetas.addVariableChangedListener(createMapUpdater(zetaUpperBodyMap, allUpperBodyZetas));
      gainScaling.addVariableChangedListener(createMapUpdater(individualJointGainScalingMap, gainScaling));

      allLowerBodyGains.notifyVariableChangedListeners();
      allLowerBodyZetas.notifyVariableChangedListeners();
      allUpperBodyGains.notifyVariableChangedListeners();
      allUpperBodyZetas.notifyVariableChangedListeners();
      gainScaling.notifyVariableChangedListeners();

      for (int i = 0; i < allRevoluteJoints.length; i++)
      {
         final RevoluteJoint revoluteJoint = allRevoluteJoints[i];

         VariableChangedListener kpScaledUpdater = new VariableChangedListener()
         {
            @Override
            public void variableChanged(YoVariable<?> v)
            {
               DoubleYoVariable kpScaled = kpScaledMap.get(revoluteJoint);
               DoubleYoVariable kp = kpMap.get(revoluteJoint);
               DoubleYoVariable individualJointGainScaling = individualJointGainScalingMap.get(revoluteJoint);
               
               kpScaled.set(kp.getDoubleValue() * individualJointGainScaling.getDoubleValue());

               if (!USE_MASS_MATRIX)
                  kpScaled.mul(TotalMassCalculator.computeSubTreeMass(revoluteJoint.getSuccessor()));
            }
         };
         
         VariableChangedListener kdUpdater = new VariableChangedListener()
         {
            @Override
            public void variableChanged(YoVariable<?> v)
            {
               DoubleYoVariable kd = kdMap.get(revoluteJoint);
               DoubleYoVariable kpScaled = kpScaledMap.get(revoluteJoint);
               DoubleYoVariable zeta = zetaMap.get(revoluteJoint);

               if (USE_MASS_MATRIX)
               {
                  kd.set(GainCalculator.computeDerivativeGain(kpScaled.getDoubleValue(), zeta.getDoubleValue()));
               }
               else
               {
                  double mass = TotalMassCalculator.computeSubTreeMass(revoluteJoint.getSuccessor());
                  kd.set(GainCalculator.computeDampingForSecondOrderSystem(mass, kpScaled.getDoubleValue(), zeta.getDoubleValue()));
               }
            }
         };
         
         individualJointGainScalingMap.get(revoluteJoint).addVariableChangedListener(kpScaledUpdater);
         kpMap.get(revoluteJoint).addVariableChangedListener(kpScaledUpdater);
         kpScaledMap.get(revoluteJoint).addVariableChangedListener(kdUpdater);
         zetaMap.get(revoluteJoint).addVariableChangedListener(kdUpdater);
         
         kpScaledUpdater.variableChanged(null);
         kdUpdater.variableChanged(null);
      }
      
      if (CONTROL_FEET_IN_TASKSPACE)
      {
         initializeMirrorDesireds.addVariableChangedListener(new VariableChangedListener()
         {
            @Override
            public void variableChanged(YoVariable<?> v)
            {
               if (initializeMirrorDesireds.getBooleanValue())
                  initializeMirrorDesireds.set(false, false);
               else
                  return;

               desiredFootPoses.get(RobotSide.LEFT).getFramePoseIncludingFrame(tempFramePose);
               desiredMirroredFootPose.set(tempFramePose, false);
            }
         });
         
         desiredMirroredFootPose.attachVariableChangedListener(new VariableChangedListener()
         {
            @Override
            public void variableChanged(YoVariable<?> v)
            {
               for (RobotSide robotSide : RobotSide.values)
               {
                  desiredMirroredFootPose.getPosition().get(mirroredXYZ);
                  desiredFootPoses.get(robotSide).setXYZ(mirroredXYZ.x, robotSide.negateIfRightSide(mirroredXYZ.y), mirroredXYZ.z);
                  
                  desiredMirroredFootPose.getOrientation().getYawPitchRoll(mirroredYawPitchRoll);
                  desiredFootPoses.get(robotSide).setYawPitchRoll(robotSide.negateIfRightSide(mirroredYawPitchRoll[0]), mirroredYawPitchRoll[1], robotSide.negateIfRightSide(mirroredYawPitchRoll[2]));
               }
            }
         });
      }
   }

   private <T> VariableChangedListener createMapUpdater(final Map<T, DoubleYoVariable> mapToUpdate, final DoubleYoVariable refVariable)
   {
      VariableChangedListener variableChangedListener = new VariableChangedListener()
      {
         private final List<T> keyList = new ArrayList<T>(mapToUpdate.keySet());

         @Override
         public void variableChanged(YoVariable<?> v)
         {
            for (int i = 0; i < keyList.size(); i++)
               mapToUpdate.get(keyList.get(i)).set(refVariable.getDoubleValue());
         }
      };
      variableChangedListener.variableChanged(null);
      return variableChangedListener;
   }

   public void setPDControllerGains(Map<OneDoFJoint, Double> positionControlKpGains, Map<OneDoFJoint, Double> positionControlKdGains)
   {
      for (int i = 0; i < allRevoluteJoints.length; i++)
      {
         RevoluteJoint joint = allRevoluteJoints[i];
         Double kp = positionControlKpGains == null ? 0.0 : positionControlKpGains.get(joint);
         Double kd = positionControlKdGains == null ? 0.0 : positionControlKdGains.get(joint);
         
         if (kp != null)
            kpScaledMap.get(joint).set(kp);
         if (kd != null)
            kdMap.get(joint).set(kd);
      }
   }

   private void initializeIDCalculator()
   {
      inverseDynamicsCalculator.reset();
      for (InverseDynamicsJoint joint : allJoints)
         joint.setDesiredAccelerationToZero();

      desiredVerticalAccelerationVector.setIncludingFrame(worldFrame, 0.0, 0.0, percentOfGravityCompensation.getDoubleValue() * gravityZ);
      desiredVerticalAccelerationVector.changeFrame(rootJoint.getFrameAfterJoint());
      desiredRootJointAcceleration.setLinearPart(desiredVerticalAccelerationVector.getVector());
      fullRobotModel.getRootJoint().setDesiredAcceleration(desiredRootJointAcceleration);
   }
   
   private final SideDependentList<MutableDouble> comDistanceFromFeet = new SideDependentList<>(new MutableDouble(), new MutableDouble());
   private final FramePoint tempPoint = new FramePoint();
   
   @Override
   public void doAction()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         if (!mirroredLegGains.get(robotSide).getBooleanValue())
            break;

         for (int i = 0; i < legsRevoluteJoints.get(robotSide).length; i++)
         {
            RevoluteJoint jointThisSide = legsRevoluteJoints.get(robotSide)[i];
            RevoluteJoint jointOppSide = legsRevoluteJoints.get(robotSide.getOppositeSide())[i];

            double zetaThisSide = zetaMap.get(jointThisSide).getDoubleValue();
            zetaMap.get(jointOppSide).set(zetaThisSide);

            double kpThisSide = kpMap.get(jointThisSide).getDoubleValue();
            kpMap.get(jointOppSide).set(kpThisSide);
         }
      }
      
      bipedSupportPolygons.updateUsingContactStates(footContactStates);
      momentumBasedController.callUpdatables();
      
      computeGravityTorquesForViz();

      initializeIDCalculator();

      if (CONTROL_FEET_IN_TASKSPACE)
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            TrajectoryBasedNumericalInverseKinematicsCalculator footIKCalculator = footIKCalculators.get(robotSide);
            footIKCalculator.compute(0.0);
            DenseMatrix64F desiredJointAngles = footIKCalculator.getDesiredJointAngles();

            RevoluteJoint[] legRevoluteJoints = legsRevoluteJoints.get(robotSide);
            for (int i = 0; i < legRevoluteJoints.length; i++)
            {
               RevoluteJoint legJoint = legRevoluteJoints[i];
               desiredJointPositions.get(legJoint).set(desiredJointAngles.get(i, 0));
            }
         }
      }
      
      doPDControl();

      for (int i = 0; i < allRevoluteJoints.length; i++)
      {
         RevoluteJoint revoluteJoint = allRevoluteJoints[i];

         double qddDesired = qdd_dMap.get(revoluteJoint).getDoubleValue();
         revoluteJoint.setQddDesired(qddDesired);
      }

      if (STAND_ON_FEET)
      {
         ReferenceFrame centerOfMassFrame = referenceFrames.getCenterOfMassFrame();
         for (RobotSide robotSide : RobotSide.values)
         {
            centerOfMass.setToZero(centerOfMassFrame);
            ReferenceFrame footFrame = feet.get(robotSide).getBodyFixedFrame();
            centerOfMass.changeFrame(footFrame);
            tempPoint.setToZero(footFrame);
            
            comDistanceFromFeet.get(robotSide).setValue(centerOfMass.getXYPlaneDistance(tempPoint));
         }
         
         for (RobotSide robotSide : RobotSide.values)
         {
            RigidBody foot = feet.get(robotSide);
            ReferenceFrame footFrame = foot.getBodyFixedFrame();
            ReferenceFrame midFeetZUpFrame = referenceFrames.getMidFeetZUpFrame();

            centerOfMass.setToZero(centerOfMassFrame);
            centerOfMass.changeFrame(midFeetZUpFrame);

            Wrench footWrench = footWrenches.get(robotSide);
            double percentOfLoad = comDistanceFromFeet.get(robotSide).doubleValue() / (comDistanceFromFeet.get(robotSide).doubleValue() + comDistanceFromFeet.get(robotSide.getOppositeSide()).doubleValue());
            percentOfLoad = 1.0 - MathTools.clipToMinMax(percentOfLoad, 0.0, 1.0);
            percentOfLoad = MathTools.clipToMinMax(2.0 * (percentOfLoad - 0.5) + 0.5, 0.0, 1.0);
            double fz = totalMass * gravityZ * percentOfLoad * footForceScaling.getDoubleValue();
            
            footWrench.setToZero(footFrame, centerOfMassFrame);
            footWrench.setLinearPartZ(fz);
            footWrench.changeFrame(footFrame);

            momentumBasedController.getCenterOfMassJacobian().packCenterOfMassVelocity(centerOfMassVelocity);
            centerOfMass.changeFrame(midFeetZUpFrame);
            centerOfMassVelocity.changeFrame(midFeetZUpFrame);
            
            footWrench.setAngularPartX((kpCoM.getDoubleValue() * centerOfMass.getY() + kdCoM.getDoubleValue() * centerOfMassVelocity.getY()) * footForceScaling.getDoubleValue());
            footWrench.setAngularPartY(footWrench.getAngularPartY() - (kpCoM.getDoubleValue() * centerOfMass.getX() + kdCoM.getDoubleValue() * centerOfMassVelocity.getX()) * footForceScaling.getDoubleValue());
            footWrench.setAngularPartZ(0.0);
            
            centerOfPressureResolver.resolveCenterOfPressureAndNormalTorque(tempCoP2d, footWrench, footFrame);
            tempCoP.changeFrame(tempCoP2d.getReferenceFrame());
            tempCoP.setXY(tempCoP2d);
            tempCoP.setZ(0.0);
            tempCoP.changeFrame(worldFrame);
            tempCoP.setZ(0.0);
            cops.get(robotSide).set(tempCoP);
            
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
      initializeIDCalculator();

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
      
      inverseDynamicsCalculator.reset();
      inverseDynamicsCalculator.compute();

      if (!CONTROL_FEET_IN_TASKSPACE)
         return;
      
      for (RobotSide robotSide : RobotSide.values)
      {
         FramePose currentFootPose = new FramePose(feet.get(robotSide).getBodyFixedFrame());
         currentFootPose.changeFrame(pelvis.getBodyFixedFrame());
         
         desiredFootPoses.get(robotSide).set(currentFootPose);
         footIKCalculators.get(robotSide).initialize();
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
         WholeBody, LeftLeg, RightLeg, LeftArm, RightArm, SpineNeck;

         public static final SideDependentList<SliderBoardMode> legs = new SideDependentList<>(LeftLeg, RightLeg);

         public static final SideDependentList<SliderBoardMode> arms = new SideDependentList<>(LeftArm, RightArm);
      };

      private enum SliderBoardSubMode
      {
         Gains, Desireds
      }
      
      private enum ControlMode {Normal, Mirrored}

      private final EnumYoVariable<SliderBoardMode> sliderBoardMode;
      private final EnumYoVariable<SliderBoardSubMode> sliderBoardSubMode;
      private final BooleanYoVariable sliderInGainsMode;
      private final EnumYoVariable<ControlMode> controlMode;
      private final BooleanYoVariable inMirroredMode;

      private final SliderBoardConfigurationManager sliderBoardConfigurationManager;

      private final String lastSliderVarName;
      private final double lastSliderMinValue;
      private final double lastSliderMaxValue;

      private final int maxNumberOfDofs = 7;

      private final double maxGains = USE_MASS_MATRIX ? 100.0 : 10.0;
      private final double maxZetas = USE_MASS_MATRIX ? 1.0 : 0.5;

      private final YoVariableRegistry registry;

      public GravityCompensationSliderBoard(SimulationConstructionSet scs, FullRobotModel fullRobotModel, YoVariableRegistry registry)
      {
         this(scs, fullRobotModel, registry, null, 0.0, 0.0);
      }

      public GravityCompensationSliderBoard(SimulationConstructionSet scs, FullRobotModel fullRobotModel, final YoVariableRegistry registry,
            String varNameForLastSlider, double varMinValue, double varMaxValue)
      {
         this.registry = registry;

         lastSliderVarName = varNameForLastSlider;
         lastSliderMinValue = varMinValue;
         lastSliderMaxValue = varMaxValue;

         sliderBoardMode = new EnumYoVariable<SliderBoardMode>("GravityCompensationSliderBoard", registry, SliderBoardMode.class);
         sliderBoardSubMode = new EnumYoVariable<SliderBoardSubMode>("sliderBoardSubMode", registry, SliderBoardSubMode.class);
         sliderInGainsMode = new BooleanYoVariable("sliderInGainsMode", registry);
         if (CONTROL_FEET_IN_TASKSPACE)
         {
            controlMode = new EnumYoVariable<ControlMode>("controlMode", "", registry, ControlMode.class, true);
            inMirroredMode = new BooleanYoVariable("sliderInMirroredMode", registry);
         }
         else
         {
            controlMode = null;
            inMirroredMode = null;
         }
         sliderBoardConfigurationManager = new SliderBoardConfigurationManager(scs);

         sliderBoardConfigurationManager.setSlider(1, "percentOfGravityCompensation", registry, 0.0, 1.0);
         sliderBoardConfigurationManager.setSlider(2, CONTROLLER_PREFIX + "gainScaling", registry, 0.0, 1.0);
         sliderBoardConfigurationManager.setSlider(3, CONTROLLER_PREFIX + "allLowerBodyGains", registry, 0.0, maxGains);
         sliderBoardConfigurationManager.setSlider(4, CONTROLLER_PREFIX + "allLowerBodyZetas", registry, 0.0, maxZetas);
         sliderBoardConfigurationManager.setSlider(5, CONTROLLER_PREFIX + "allUpperBodyGains", registry, 0.0, maxGains);
         sliderBoardConfigurationManager.setSlider(6, CONTROLLER_PREFIX + "allUpperBodyZetas", registry, 0.0, maxZetas);

         if (STAND_ON_FEET)
         {
            sliderBoardConfigurationManager.setSlider(7, CONTROLLER_PREFIX + "footForceScaling", registry, 0.0, 1.0);
         }

         SliderBoardMode currentMode = SliderBoardMode.WholeBody;
         finalizeConfiguration(currentMode, SliderBoardSubMode.Gains);

         RigidBody pelvis = fullRobotModel.getPelvis();
         for (RobotSide robotSide : RobotSide.values)
         {
            currentMode = SliderBoardMode.legs.get(robotSide);
            RigidBody foot = fullRobotModel.getFoot(robotSide);
            RevoluteJoint[] legRevoluteJoints = ScrewTools.filterJoints(ScrewTools.createJointPath(pelvis, foot), RevoluteJoint.class);

            if (CONTROL_FEET_IN_TASKSPACE)
            {
               setupTaskpaceDesiredsJointGainScaling(robotSide, legRevoluteJoints);
               finalizeConfiguration(currentMode, SliderBoardSubMode.Desireds, ControlMode.Normal);
               setupTaskpaceMirroredDesiredsJointGainScaling(legRevoluteJoints);
               finalizeConfiguration(currentMode, SliderBoardSubMode.Desireds, ControlMode.Mirrored);
            }
            else
            {
               setupJointSlidersAndKnobsForDesireds(legRevoluteJoints);
               finalizeConfiguration(currentMode, SliderBoardSubMode.Desireds);
            }
            setupJointSlidersAndKnobsForGains(robotSide, legRevoluteJoints);
            finalizeConfiguration(currentMode, SliderBoardSubMode.Gains);
         }

         for (RobotSide robotSide : RobotSide.values)
         {
            currentMode = SliderBoardMode.arms.get(robotSide);
            RigidBody hand = fullRobotModel.getHand(robotSide);
            RigidBody chest = fullRobotModel.getChest();
            RevoluteJoint[] armRevoluteJoints = ScrewTools.filterJoints(ScrewTools.createJointPath(chest, hand), RevoluteJoint.class);

            setupJointSlidersAndKnobsForDesireds(armRevoluteJoints);
            finalizeConfiguration(currentMode, SliderBoardSubMode.Desireds);

            setupJointSlidersAndKnobsForGains(null, armRevoluteJoints);
            finalizeConfiguration(currentMode, SliderBoardSubMode.Gains);
         }

         currentMode = SliderBoardMode.SpineNeck;
         RigidBody head = fullRobotModel.getHead();
         RevoluteJoint[] spineAndNeckRevoluteJoints = ScrewTools.filterJoints(ScrewTools.createJointPath(pelvis, head), RevoluteJoint.class);

         setupJointSlidersAndKnobsForDesireds(spineAndNeckRevoluteJoints);
         finalizeConfiguration(currentMode, SliderBoardSubMode.Desireds);

         setupJointSlidersAndKnobsForGains(null, spineAndNeckRevoluteJoints);
         finalizeConfiguration(currentMode, SliderBoardSubMode.Gains);

         sliderInGainsMode.addVariableChangedListener(new VariableChangedListener()
         {
            @Override
            public void variableChanged(YoVariable<?> v)
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
            public void variableChanged(YoVariable<?> v)
            {
               if (sliderBoardMode.valueEquals(SliderBoardMode.WholeBody))
               {
                  sliderInGainsMode.set(true, false);
                  sliderBoardSubMode.set(SliderBoardSubMode.Gains, false);
               }
               
               if (CONTROL_FEET_IN_TASKSPACE)
               {
                  if ((!sliderBoardMode.valueEquals(SliderBoardMode.LeftLeg) && !sliderBoardMode.valueEquals(SliderBoardMode.RightLeg)) || sliderBoardSubMode.valueEquals(SliderBoardSubMode.Gains))
                  {
                     inMirroredMode.set(false, false);
                     controlMode.set(null, false);
                  }
                  else if (sliderBoardSubMode.valueEquals(SliderBoardSubMode.Desireds) && controlMode.getEnumValue() == null)
                  {
                     if (sliderBoardMode.valueEquals(SliderBoardMode.LeftLeg) || sliderBoardMode.valueEquals(SliderBoardMode.RightLeg))
                     {
                        inMirroredMode.set(false, false);
                        controlMode.set(ControlMode.Normal, false);
                     }
                  }
               }

               loadConfiguration(sliderBoardMode, sliderBoardSubMode, controlMode);
            }
         };
         sliderBoardSubMode.addVariableChangedListener(variableChangedListener);
         sliderBoardSubMode.set(SliderBoardSubMode.Gains, false);

         sliderBoardMode.addVariableChangedListener(variableChangedListener);
         sliderBoardMode.set(SliderBoardMode.WholeBody, false);
         
         if (CONTROL_FEET_IN_TASKSPACE)
         {
            inMirroredMode.addVariableChangedListener(new VariableChangedListener()
            {
               @Override
               public void variableChanged(YoVariable<?> v)
               {
                  if (inMirroredMode.getBooleanValue())
                     controlMode.set(ControlMode.Mirrored);
                  else
                     controlMode.set(ControlMode.Normal);
               }
            });
            controlMode.addVariableChangedListener(variableChangedListener);
            controlMode.addVariableChangedListener(new VariableChangedListener()
            {
               @SuppressWarnings("deprecation")
               @Override
               public void variableChanged(YoVariable<?> v)
               {
                  registry.getVariable(CONTROLLER_PREFIX + "initializeMirrorDesireds").setValueFromDouble(1.0);
               }
            });
            controlMode.set(null, false);
         }
         variableChangedListener.variableChanged(null);
      }

      private void setupTaskpaceDesiredsJointGainScaling(RobotSide robotSide, RevoluteJoint[] revoluteJoints)
      {
         String varPrefix = CONTROLLER_PREFIX + "desired" + robotSide.getCamelCaseNameForMiddleOfExpression() + "Foot";
         String[] axisSuffix = new String[]{"X", "Y", "Z", "Yaw", "Pitch", "Roll"};
         
         int i = 0;
         double quarterPi = Math.PI / 4.0;
         sliderBoardConfigurationManager.setSlider(++i, varPrefix + axisSuffix[i-1], registry, -0.5, 0.5);
         sliderBoardConfigurationManager.setSlider(++i, varPrefix + axisSuffix[i-1], registry, -0.5, 0.5);
         sliderBoardConfigurationManager.setSlider(++i, varPrefix + axisSuffix[i-1], registry, -1.5, -0.5);
         sliderBoardConfigurationManager.setSlider(++i, varPrefix + axisSuffix[i-1], registry, -quarterPi, quarterPi);
         sliderBoardConfigurationManager.setSlider(++i, varPrefix + axisSuffix[i-1], registry, -quarterPi, quarterPi);
         sliderBoardConfigurationManager.setSlider(++i, varPrefix + axisSuffix[i-1], registry, -quarterPi, quarterPi);

         sliderBoardConfigurationManager.setButton(2, inMirroredMode);
         
         for (int j = 0; j < revoluteJoints.length; j++)
         {
            if (j > maxNumberOfDofs)
            {
               System.err.println("Too many joints for the slider board. Last joint configured: " + revoluteJoints[j - 1].getName());
               break;
            }

            String jointGainScalingVarName = CONTROLLER_PREFIX + revoluteJoints[j].getName() + "_gainScaling";
            sliderBoardConfigurationManager.setKnob(j + 1, jointGainScalingVarName, registry, 0.0, 1.0);
         }
      }

      private void setupTaskpaceMirroredDesiredsJointGainScaling(RevoluteJoint[] revoluteJoints)
      {
         String varPrefix = CONTROLLER_PREFIX + "desriedMirroredFoot";
         String[] axisSuffix = new String[]{"X", "Y", "Z", "Yaw", "Pitch", "Roll"};
         
         int i = 0;
         double quarterPi = Math.PI / 4.0;
         sliderBoardConfigurationManager.setSlider(++i, varPrefix + axisSuffix[i-1], registry, -0.5, 0.5);
         sliderBoardConfigurationManager.setSlider(++i, varPrefix + axisSuffix[i-1], registry, -0.5, 0.5);
         sliderBoardConfigurationManager.setSlider(++i, varPrefix + axisSuffix[i-1], registry, -1.5, -0.5);
         sliderBoardConfigurationManager.setSlider(++i, varPrefix + axisSuffix[i-1], registry, -quarterPi, quarterPi);
         sliderBoardConfigurationManager.setSlider(++i, varPrefix + axisSuffix[i-1], registry, -quarterPi, quarterPi);
         sliderBoardConfigurationManager.setSlider(++i, varPrefix + axisSuffix[i-1], registry, -quarterPi, quarterPi);

         sliderBoardConfigurationManager.setButton(2, inMirroredMode);
         
         for (int j = 0; j < revoluteJoints.length; j++)
         {
            if (j > maxNumberOfDofs)
            {
               System.err.println("Too many joints for the slider board. Last joint configured: " + revoluteJoints[j - 1].getName());
               break;
            }

            String jointGainScalingVarName = CONTROLLER_PREFIX + revoluteJoints[j].getName() + "_gainScaling";
            sliderBoardConfigurationManager.setKnob(j + 1, jointGainScalingVarName, registry, 0.0, 1.0);
         }
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

      private void setupJointSlidersAndKnobsForGains(RobotSide robotSide, RevoluteJoint[] revoluteJoints)
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
            String kp_varName = varPrefix + "_kp";
            String zeta_varName = varPrefix + "_zeta";

            double subtreeMass = TotalMassCalculator.computeSubTreeMass(revoluteJoint.getSuccessor());
            sliderBoardConfigurationManager.setSlider(i + 1, kp_varName, registry, 0.0, subtreeMass * maxGains);
            sliderBoardConfigurationManager.setKnob(i + 1, zeta_varName, registry, 0.0, maxZetas);

            if (robotSide != null)
               sliderBoardConfigurationManager.setButton(3, CONTROLLER_PREFIX + "copyLegGainsTo" + robotSide.getOppositeSide().getCamelCaseNameForMiddleOfExpression() + "Side", registry);
         }
      }
      
      @SafeVarargs
      private final <T> void finalizeConfiguration(T... modes)
      {
         if (lastSliderVarName != null)
            sliderBoardConfigurationManager.setSlider(8, lastSliderVarName, registry, lastSliderMinValue, lastSliderMaxValue);
         sliderBoardConfigurationManager.setKnob(8, sliderBoardMode, 0.0, SliderBoardMode.values().length);
         sliderBoardConfigurationManager.setButton(1, sliderInGainsMode);

         StringBuilder configurationName = new StringBuilder();
         for (T mode : modes)
            configurationName.append(mode.toString());
         sliderBoardConfigurationManager.saveConfiguration(configurationName.toString());
         sliderBoardConfigurationManager.clearControls();
      }
      
      private void loadConfiguration(EnumYoVariable<?>... modes)
      {
         StringBuilder configurationName = new StringBuilder();
         for (EnumYoVariable<?> mode : modes)
         {
            if (mode != null && mode.getEnumValue() != null)
               configurationName.append(mode.getEnumValue().toString());
         }
         sliderBoardConfigurationManager.loadConfiguration(configurationName.toString());
      }
   }
}
