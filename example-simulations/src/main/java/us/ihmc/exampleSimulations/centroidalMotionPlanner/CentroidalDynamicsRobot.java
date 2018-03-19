package us.ihmc.exampleSimulations.centroidalMotionPlanner;

import java.awt.Color;
import java.util.EnumSet;
import java.util.Set;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.CentroidalMotionNode;
import us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.CentroidalMotionPlanner;
import us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.CentroidalMotionPlannerParameters;
import us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.ForceTrajectory;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotModels.FullRobotModelFactory;
import us.ihmc.robotModels.FullRobotModelFromDescription;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.JointNameMap;
import us.ihmc.robotics.partNames.JointRole;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.robotDescription.FloatingJointDescription;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.LinkGraphicsDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotSide.RobotSegment;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.GenericStateMachine;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.State;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransitionCondition;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.RobotFromDescription;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

public class CentroidalDynamicsRobot implements FullRobotModelFactory
{
   private final double robotMass;
   private final double Ixx;
   private final double Iyy;
   private final double Izz;
   private final double xRadius;
   private final double yRadius;
   private final double zRadius;
   private final DenseMatrix64F momentOfInertia;
   private final double robotHeight = 0.75;
   private final String robotName;

   private final RobotDescription robotDescription;
   private final JointNameMap<RobotCentroidal> jointMap;
   private RobotFromDescription scsRobot;
   private FloatingJoint rootJoint;

   private final Vector3D tempVector = new Vector3D();

   public CentroidalDynamicsRobot(String robotName)
   {
      this.robotName = robotName;
      this.robotMass = 18.0;
      this.Ixx = 0.1;
      this.Iyy = 0.1;
      this.Izz = 0.1;
      this.xRadius = 0.1;
      this.yRadius = 0.2;
      this.zRadius = 0.3;
      this.momentOfInertia = new DenseMatrix64F(3, 3);
      momentOfInertia.set(0, 0, Ixx);
      momentOfInertia.set(1, 1, Iyy);
      momentOfInertia.set(2, 2, Izz);
      this.robotDescription = new CentroidalRobotDescription(robotName);
      this.jointMap = new CentroidalRobotJointMap();
   }

   public Robot getSCSRobot()
   {
      if (scsRobot == null)
         createSCSRobot(null);
      return scsRobot;
   }

   public Robot getSCSRobot(YoGraphicsListRegistry graphicsListRegistry)
   {
      if (scsRobot == null)
         createSCSRobot(graphicsListRegistry);
      return scsRobot;
   }

   private void createSCSRobot(YoGraphicsListRegistry graphicsListRegistry)
   {
      scsRobot = new RobotFromDescription(getRobotDescription());
      ExternalForcePoint forcePoint = createExternalForcePointForControl(scsRobot);
      rootJoint = (FloatingJoint) scsRobot.getRootJoints().get(0);
      rootJoint.addExternalForcePoint(forcePoint);
      scsRobot.getGravity(tempVector);
      FullRobotModel controllerFullRobotModel = createFullRobotModel();
      scsRobot.setController(new CentroidalRobotEstimator(rootJoint, controllerFullRobotModel));
      scsRobot.setController(new CentroidalRobotController(forcePoint, tempVector, controllerFullRobotModel, scsRobot.getYoTime(), graphicsListRegistry));
      CentroidalRobotInitialSetup initialSetup = new CentroidalRobotInitialSetup();
      initialSetup.initializeRobot(scsRobot);
      return;
   }

   private ExternalForcePoint createExternalForcePointForControl(Robot robot)
   {
      ExternalForcePoint forcePoint = new ExternalForcePoint(robotName + "ForcePoint", robot);
      return forcePoint;
   }

   @Override
   public RobotDescription getRobotDescription()
   {
      return robotDescription;
   }

   @Override
   public FullRobotModel createFullRobotModel()
   {
      RobotDescription robotDescription = new CentroidalRobotDescription(robotName);
      FullRobotModelFromDescription fullRobotModel = new FullRobotModelFromDescription(robotDescription, getJointMap(), null);
      return fullRobotModel;
   }

   public JointNameMap<RobotCentroidal> getJointMap()
   {
      return jointMap;
   }

   public enum RobotCentroidal implements RobotSegment<RobotCentroidal>
   {
      BODY;

      public static final RobotCentroidal[] values = values();
      public static final EnumSet<RobotCentroidal> set = EnumSet.allOf(RobotCentroidal.class);

      @Override
      public RobotCentroidal[] getValues()
      {
         return values;
      }

      @Override
      public Class<RobotCentroidal> getClassType()
      {
         return RobotCentroidal.class;
      }

      @Override
      public EnumSet<RobotCentroidal> getEnumSet()
      {
         return set;
      }
   }

   public class CentroidalRobotInitialSetup
   {
      Vector3D initialPosition = new Vector3D();
      Quaternion orientation = new Quaternion();

      public CentroidalRobotInitialSetup()
      {
         initialPosition.set(0.0, 0.0, robotHeight);
      }

      public void initializeRobot(Robot robot)
      {
         FloatingJoint rootJoint = (FloatingJoint) robot.getRootJoints().get(0);
         rootJoint.setPosition(initialPosition);
         rootJoint.setQuaternion(orientation);
      }

      public void setInitialYaw(double yaw)
      {
         orientation.setToYawQuaternion(yaw);
      }

      public double getInitialYaw()
      {
         return orientation.getYaw();
      }

      public void setInitialGroundHeight(double groundHeight)
      {
         initialPosition.setZ(groundHeight);
      }

      public double getInitialGroundHeight()
      {
         return initialPosition.getZ();
      }

      public void setOffset(Vector3D additionalOffset)
      {
         initialPosition.add(additionalOffset);
      }

      public void getOffset(Vector3D offsetToPack)
      {
         offsetToPack.set(initialPosition);
      }
   }

   public class CentroidalRobotController implements RobotController
   {
      private static final String namePrefix = "CentroidaDynamicsRobotController";
      private final YoVariableRegistry registry = new YoVariableRegistry(namePrefix);
      private final ExternalForcePoint forcePoint;
      private final Vector3D gravity;
      private boolean initialized = false;

      private final CentroidalMotionPlannerParameters parameters;

      private final Vector3D tempVector = new Vector3D();
      private CentroidalMotionPlanner motionPlanner;
      private final FullRobotModel robotModel;
      private final GenericStateMachine<CentroidalRobotStateEnum, CentroidalRobotState> stateMachine;

      public CentroidalRobotController(ExternalForcePoint forcePoint, Vector3D gravity, FullRobotModel controllerFullRobotModel, YoDouble yoTime,
                                       YoGraphicsListRegistry graphicsListRegistry)
      {
         this.gravity = gravity;
         this.forcePoint = forcePoint;
         this.robotModel = controllerFullRobotModel;
         this.parameters = new CentroidalMotionPlannerParameters();
         parameters.setRobotMass(robotMass);
         parameters.setDeltaTMin(0.01);
         parameters.setGravity(gravity);
         parameters.setNominalIxx(Ixx);
         parameters.setNominalIyy(Iyy);
         parameters.setNominalIzz(Izz);
         parameters.setMaxForce(new Vector3D(100.0, 100.0, 2.0 * robotMass * 9.81));
         parameters.setMinForce(new Vector3D(-100.0, -100.0, 0.0));
         parameters.setMaxForceRate(new Vector3D(0.2, 0.2, 0.2));
         parameters.setMinForceRate(new Vector3D(-0.2, -0.2, -0.2));
         this.stateMachine = new GenericStateMachine<>(namePrefix + "State", namePrefix + "TimeInState", CentroidalRobotStateEnum.class, yoTime, registry);
         if (graphicsListRegistry != null)
            createExternalForcePointGraphic(graphicsListRegistry, forcePoint);
      }

      @Override
      public void initialize()
      {
         this.motionPlanner = new CentroidalMotionPlanner(parameters);
         setupStateMachine();
      }

      private void setupStateMachine()
      {
         GroundState groundState = new GroundState(forcePoint, gravity, robotModel, motionPlanner, registry);
         FlightState flightState = new FlightState(forcePoint, gravity, robotModel, motionPlanner, registry);
         GroundToFlightState groundToFlightTransition = new GroundToFlightState(groundState);
         groundState.addStateTransition(CentroidalRobotStateEnum.FLIGHT, groundToFlightTransition);
         FlightToGroundState flightToGroundTransition = new FlightToGroundState(flightState);
         flightState.addStateTransition(CentroidalRobotStateEnum.GROUND, flightToGroundTransition);
         stateMachine.addState(groundState);
         stateMachine.addState(flightState);
         stateMachine.setCurrentState(CentroidalRobotStateEnum.GROUND);
      }

      private void createExternalForcePointGraphic(YoGraphicsListRegistry graphicsListRegistry, ExternalForcePoint externalForcePoint)
      {
         YoFrameVector yoForceVector = externalForcePoint.getYoForce();
         YoFramePoint yoForcePoint = externalForcePoint.getYoPosition();
         YoFramePoint yoGroundPoint = new YoFramePoint(robotName + "ProjectedFocePoint", yoForcePoint.getReferenceFrame(), registry);
         yoForcePoint.attachVariableChangedListener(new VariableChangedListener()
         {
            @Override
            public void notifyOfVariableChange(YoVariable<?> v)
            {
               yoGroundPoint.set(yoForcePoint.getX(), yoForcePoint.getY(), 0.0);
            }
         });
         YoGraphicVector forceVisualization = new YoGraphicVector(robotName + "ForceVisualization", yoGroundPoint, yoForceVector, 0.005,
                                                                  new YoAppearanceRGBColor(Color.RED, 0.0), true);
         graphicsListRegistry.registerYoGraphic(robotName, forceVisualization);
      }

      @Override
      public YoVariableRegistry getYoVariableRegistry()
      {
         return registry;
      }

      @Override
      public String getName()
      {
         return "CentroidalDynamicsController";
      }

      @Override
      public String getDescription()
      {
         return "CentroidalRobotController";
      }

      @Override
      public void doControl()
      {
         if (!initialized)
         {
            initialize();
            initialized = true;
         }
         stateMachine.checkTransitionConditions();
         stateMachine.doAction();
      }
   }

   public class CentroidalRobotEstimator implements RobotController
   {
      private final FloatingJoint estimatedRootJoint;
      private final FloatingInverseDynamicsJoint controllerRootJoint;
      private final Point3D position = new Point3D();
      private final Vector3D velocity = new Vector3D();
      private final Vector3D acceleration = new Vector3D();
      private final Quaternion orientation = new Quaternion();
      private final Vector3D angularVelocity = new Vector3D();
      private final Vector3D angularAcceleration = new Vector3D();
      private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
      private final FullRobotModel robotModel;

      DenseMatrix64F tempMatrix = new DenseMatrix64F(3, 1);

      public CentroidalRobotEstimator(FloatingJoint rootJoint, FullRobotModel controllerRobotModel)
      {
         this.estimatedRootJoint = rootJoint;
         this.robotModel = controllerRobotModel;
         this.controllerRootJoint = robotModel.getRootJoint();
      }

      @Override
      public void initialize()
      {

      }

      @Override
      public YoVariableRegistry getYoVariableRegistry()
      {
         return registry;
      }

      @Override
      public String getName()
      {
         return getClass().getSimpleName();
      }

      @Override
      public String getDescription()
      {
         return "Estimator for the centroidal dynamics robot";
      }

      @Override
      public void doControl()
      {
         estimatedRootJoint.getPositionAndVelocity(position, velocity);
         estimatedRootJoint.getLinearAccelerationInWorld(acceleration);
         estimatedRootJoint.getRotationToWorld(orientation);
         estimatedRootJoint.getAngularVelocityInBody(angularVelocity);
         estimatedRootJoint.getAngularAccelerationInBody(angularAcceleration);

         controllerRootJoint.setPosition(position);
         controllerRootJoint.setRotation(orientation);
         tempMatrix.reshape(6, 1);
         angularVelocity.get(0, tempMatrix);
         velocity.get(3, tempMatrix);
         controllerRootJoint.setVelocity(tempMatrix, 0);
      }
   }

   public enum CentroidalRobotStateEnum
   {
      FLIGHT, GROUND
   };

   public abstract class CentroidalRobotState extends State<CentroidalRobotStateEnum>
   {
      protected final ExternalForcePoint forcePoint;
      protected final Vector3D gravity;
      protected final Point3D position = new Point3D();
      protected final Vector3D linearVelocity = new Vector3D();
      protected final Vector3D linearAcceleration = new Vector3D();
      protected final Quaternion orientation = new Quaternion();
      protected final Vector3D angularVelocity = new Vector3D();
      protected final Vector3D angularAcceleratrion = new Vector3D();

      protected boolean transitionToNextState = false;

      protected final FullRobotModel robotModel;
      protected final CentroidalMotionPlanner motionPlanner;

      public CentroidalRobotState(CentroidalRobotStateEnum stateEnum, ExternalForcePoint forcePoint, Vector3D gravity, FullRobotModel robotModel,
                                  CentroidalMotionPlanner motionPlanner)
      {
         super(stateEnum);
         this.forcePoint = forcePoint;
         this.gravity = gravity;
         this.robotModel = robotModel;
         this.motionPlanner = motionPlanner;
      }

      @Override
      public void doTransitionIntoAction()
      {
         transitionToNextState = false;
      }

      public void doControl()
      {
         FloatingInverseDynamicsJoint controllerRootJoint = robotModel.getRootJoint();
         controllerRootJoint.getTranslation(position);
         controllerRootJoint.getRotation(orientation);
         controllerRootJoint.getLinearVelocity(linearVelocity);
         controllerRootJoint.getAngularVelocity(angularVelocity);
         controllerRootJoint.getLinearAcceleration(linearAcceleration);
      }

      public boolean transitionToNextState()
      {
         return transitionToNextState;
      }
   }

   public class GroundState extends CentroidalRobotState
   {
      private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      private final CentroidalRobotStateEnum stateEnum;
      private final String namePrefix;

      private final FrameVector3D zeroForceConstraint = new FrameVector3D();
      private final FrameVector3D initialForceConstraint = new FrameVector3D();
      private final FrameVector3D finalForceConstraint = new FrameVector3D();
      private final FrameVector3D initialForceRateConstraint = new FrameVector3D();
      private final FrameVector3D finalForceRateConstraint = new FrameVector3D();
      private final FramePoint3D maxPosition = new FramePoint3D();
      private final FramePoint3D minPosition = new FramePoint3D();
      private final FramePoint3D initialPosition = new FramePoint3D();
      private final FrameVector3D initialVelocity = new FrameVector3D();
      private final FramePoint3D intermediatePosition = new FramePoint3D();
      private final FrameVector3D intermediateVelocity = new FrameVector3D();
      private final FramePoint3D finalPosition = new FramePoint3D();
      private final FrameVector3D finalVelocity = new FrameVector3D();
      private final double defaultPlanningTime = 2.0;
      private final FrameVector3D forceWeight = new FrameVector3D(worldFrame, 0.00001, 0.00001, 0.00001);
      private final FrameVector3D forceRateWeight = new FrameVector3D(worldFrame, 0.001, 0.001, 0.001);
      private final FrameVector3D positionWeight = new FrameVector3D(worldFrame, 1.0, 1.0, 1.0);
      private final FrameVector3D linearVelocityWeight = new FrameVector3D(worldFrame, 1.0, 1.0, 1.0);
      private final FrameVector3D intermediatePositionWeight = new FrameVector3D(worldFrame, 1.0, 1.0, 1.0);
      private final FrameVector3D intermediateLinearVelocityWeight = new FrameVector3D(worldFrame, 1.0, 1.0, 1.0);

      private FrameVector3D forceToExert = new FrameVector3D();
      private ForceTrajectory forceProfile;
      private final YoDouble trajectoryStartTime;
      private final YoDouble trajectoryTime;

      private RecyclingArrayList<CentroidalMotionNode> motionPlannerNode = new RecyclingArrayList<>(CentroidalMotionNode.class);

      public GroundState(ExternalForcePoint forcePoint, Vector3D gravity, FullRobotModel robotModel, CentroidalMotionPlanner motionPlanner,
                         YoVariableRegistry registry)
      {
         super(CentroidalRobotStateEnum.GROUND, forcePoint, gravity, robotModel, motionPlanner);
         namePrefix = getClass().getSimpleName();
         this.stateEnum = getStateEnum();
         this.trajectoryTime = new YoDouble(namePrefix + "TrajectoryTime", registry);
         this.trajectoryStartTime = new YoDouble(namePrefix + "TrajectoryStartTime", registry);
      }

      @Override
      public void doAction()
      {
         super.doControl();
         double time = getTimeInCurrentState() - trajectoryStartTime.getDoubleValue();
         trajectoryTime.set(time);
         if (time >= forceProfile.getFinalTime())
         {
            trajectoryStartTime.set(getTimeInCurrentState());
            trajectoryTime.set(0.0);
            planMotion();
            time = 0.0;
         }
         forceProfile.update(time, forceToExert);
         //forceToExert.setX(0.0);
         forceToExert.setY(0.0);
         if (forceToExert.containsNaN())
            forceToExert.setZ(0.0);
         forcePoint.setForce(forceToExert);
      }

      @Override
      public void doTransitionIntoAction()
      {
         super.doTransitionIntoAction();
         double timeInCurrentState = getTimeInCurrentState();
         trajectoryStartTime.set(timeInCurrentState);
         trajectoryTime.set(0.0);
         planMotion();
      }

      @Override
      public void doTransitionOutOfAction()
      {
      }

      public void planMotion()
      {
         PrintTools.debug("Replanning");
         zeroForceConstraint.set(worldFrame, 0.0, 0.0, 0.0);
         minPosition.set(worldFrame, -0.1, -0.1, -0.75);
         maxPosition.set(worldFrame, 0.1, 0.1, 0.10);
         initialForceConstraint.set(worldFrame, 0.0, 0.0, -robotMass * gravity.getZ());
         initialForceRateConstraint.set(worldFrame, 0.0, 0.0, 0.0);
         finalForceConstraint.set(worldFrame, 0.0, 0.0, -robotMass * gravity.getZ());
         finalForceRateConstraint.set(worldFrame, 0.0, 0.0, 0.0);

         initialPosition.set(worldFrame, 0.0, 0.0, 0.0);
         //initialPosition.subZ(robotHeight);
         intermediatePosition.set(worldFrame, 0.0, 0.0, 0.05);
         finalPosition.set(worldFrame, 0.0, 0.0, 0.0);

         initialVelocity.set(worldFrame, linearVelocity);
         intermediateVelocity.set(worldFrame, 0.0, 0.0, 0.0);
         finalVelocity.set(worldFrame, 0.0, 0.0, 0.0);

         motionPlannerNode.clear();
         motionPlanner.reset();
         CentroidalMotionNode node = motionPlannerNode.add();
         node.reset();
         node.setTime(0.0);
         node.setForceConstraint(initialForceConstraint);
         node.setForceRateConstraint(initialForceRateConstraint);
         node.setPositionConstraint(initialPosition);
         node.setLinearVelocityConstraint(initialVelocity);
         motionPlanner.submitNode(node);

         node.reset();
         node.setTime(defaultPlanningTime * 0.08);
         node.setForceObjective(initialForceConstraint, forceWeight);
         node.setForceRateObjective(initialForceRateConstraint, forceRateWeight);
         node.setPositionInequalities(maxPosition, minPosition);
         motionPlanner.submitNode(node);

         node.reset();
         node.setTime(defaultPlanningTime * 0.16);
         node.setForceObjective(initialForceConstraint, forceWeight);
         node.setForceRateObjective(initialForceRateConstraint, forceRateWeight);
         node.setPositionInequalities(maxPosition, minPosition);
         motionPlanner.submitNode(node);

         node.reset();
         node.setTime(defaultPlanningTime * 0.24);
         node.setForceObjective(initialForceConstraint, forceWeight);
         node.setForceRateObjective(initialForceRateConstraint, forceRateWeight);
         node.setPositionInequalities(maxPosition, minPosition);
         motionPlanner.submitNode(node);

         node.reset();
         node.setTime(defaultPlanningTime * 0.32);
         node.setForceObjective(initialForceConstraint, forceWeight);
         node.setForceRateObjective(initialForceRateConstraint, forceRateWeight);
         node.setPositionInequalities(maxPosition, minPosition);
         motionPlanner.submitNode(node);

         node.reset();
         node.setTime(defaultPlanningTime * 0.45);
         node.setForceConstraint(zeroForceConstraint);
         node.setForceRateConstraint(initialForceRateConstraint);
         node.setPositionInequalities(maxPosition, minPosition);
         motionPlanner.submitNode(node);

         node.reset();
         node.setTime(defaultPlanningTime * 0.5);
         node.setForceConstraint(zeroForceConstraint);
         node.setForceRateObjective(finalForceRateConstraint, forceRateWeight);
         //node.setPositionObjective(intermediatePosition, positionWeight);
         motionPlanner.submitNode(node);

         node.reset();
         node.setTime(defaultPlanningTime * 0.55);
         node.setForceConstraint(zeroForceConstraint);
         node.setForceRateConstraint(finalForceRateConstraint);
         node.setPositionInequalities(maxPosition, minPosition);
         motionPlanner.submitNode(node);

         node.reset();
         node.setTime(defaultPlanningTime * 0.68);
         node.setForceObjective(initialForceConstraint, forceWeight);
         node.setForceRateObjective(initialForceRateConstraint, forceRateWeight);
         node.setPositionInequalities(maxPosition, minPosition);
         motionPlanner.submitNode(node);

         node.reset();
         node.setTime(defaultPlanningTime * 0.76);
         node.setForceObjective(initialForceConstraint, forceWeight);
         node.setForceRateObjective(initialForceRateConstraint, forceRateWeight);
         node.setPositionInequalities(maxPosition, minPosition);
         motionPlanner.submitNode(node);

         node.reset();
         node.setTime(defaultPlanningTime * 0.84);
         node.setForceObjective(initialForceConstraint, forceWeight);
         node.setForceRateObjective(initialForceRateConstraint, forceRateWeight);
         node.setPositionInequalities(maxPosition, minPosition);
         motionPlanner.submitNode(node);

         node.reset();
         node.setTime(defaultPlanningTime * 0.92);
         node.setForceObjective(initialForceConstraint, forceWeight);
         node.setForceRateObjective(initialForceRateConstraint, forceRateWeight);
         node.setPositionInequalities(maxPosition, minPosition);
         motionPlanner.submitNode(node);

         node.reset();
         node.setTime(defaultPlanningTime);
         node.setForceConstraint(finalForceConstraint);
         node.setForceRateConstraint(finalForceRateConstraint);
         node.setPositionConstraint(finalPosition);
         node.setLinearVelocityConstraint(finalVelocity);
         motionPlanner.submitNode(node);

         motionPlanner.compute();
         forceProfile = motionPlanner.getForceProfile();
      }

      public boolean transitionToFlight()
      {
         return super.transitionToNextState();
      }
   }

   public class FlightState extends CentroidalRobotState
   {
      private final CentroidalRobotStateEnum stateEnum;

      public FlightState(ExternalForcePoint forcePoint, Vector3D gravity, FullRobotModel robotModel, CentroidalMotionPlanner motionPlanner,
                         YoVariableRegistry registry)
      {
         super(CentroidalRobotStateEnum.FLIGHT, forcePoint, gravity, robotModel, motionPlanner);
         stateEnum = getStateEnum();
      }

      @Override
      public void doAction()
      {
         super.doControl();
      }

      @Override
      public void doTransitionIntoAction()
      {
         super.doTransitionIntoAction();
      }

      @Override
      public void doTransitionOutOfAction()
      {

      }

      public boolean transitionToGround()
      {
         return super.transitionToNextState();
      }
   }

   public class GroundToFlightState implements StateTransitionCondition
   {
      private final GroundState groundState;

      public GroundToFlightState(GroundState groundState)
      {
         this.groundState = groundState;
      }

      @Override
      public boolean checkCondition()
      {
         return groundState.transitionToFlight();
      }
   }

   public class FlightToGroundState implements StateTransitionCondition
   {
      private final FlightState flightState;

      public FlightToGroundState(FlightState flightState)
      {
         this.flightState = flightState;
      }

      @Override
      public boolean checkCondition()
      {
         return flightState.transitionToGround();
      }
   }

   public class CentroidalRobotJointMap implements JointNameMap<RobotCentroidal>
   {

      @Override
      public LegJointName[] getLegJointNames()
      {
         return null;
      }

      @Override
      public ArmJointName[] getArmJointNames()
      {
         // TODO Auto-generated method stub
         return null;
      }

      @Override
      public SpineJointName[] getSpineJointNames()
      {
         // TODO Auto-generated method stub
         return null;
      }

      @Override
      public NeckJointName[] getNeckJointNames()
      {
         // TODO Auto-generated method stub
         return null;
      }

      @Override
      public String getModelName()
      {
         return robotName;
      }

      @Override
      public JointRole getJointRole(String jointName)
      {
         return null;
      }

      @Override
      public NeckJointName getNeckJointName(String jointName)
      {
         // TODO Auto-generated method stub
         return null;
      }

      @Override
      public SpineJointName getSpineJointName(String jointName)
      {
         // TODO Auto-generated method stub
         return null;
      }

      @Override
      public String getRootBodyName()
      {
         // TODO Auto-generated method stub
         return null;
      }

      @Override
      public String getUnsanitizedRootJointInSdf()
      {
         // TODO Auto-generated method stub
         return null;
      }

      @Override
      public String getHeadName()
      {
         // TODO Auto-generated method stub
         return null;
      }

      @Override
      public boolean isTorqueVelocityLimitsEnabled()
      {
         // TODO Auto-generated method stub
         return false;
      }

      @Override
      public Set<String> getLastSimulatedJoints()
      {
         // TODO Auto-generated method stub
         return null;
      }

      @Override
      public String[] getJointNamesBeforeFeet()
      {
         // TODO Auto-generated method stub
         return null;
      }

      @Override
      public RobotCentroidal[] getRobotSegments()
      {
         // TODO Auto-generated method stub
         return null;
      }

      @Override
      public RobotCentroidal getEndEffectorsRobotSegment(String jointNameBeforeEndEffector)
      {
         // TODO Auto-generated method stub
         return null;
      }
   }

   public class CentroidalRobotDescription extends RobotDescription
   {
      public CentroidalRobotDescription(String namePrefix)
      {
         super(namePrefix);
         FloatingJointDescription rootJoint = new FloatingJointDescription(namePrefix + "RootJoint");
         LinkDescription rootLink = new LinkDescription(namePrefix + "RootLink");
         LinkGraphicsDescription rootLinkGraphics = new LinkGraphicsDescription();
         rootLinkGraphics.addEllipsoid(xRadius, yRadius, zRadius, new YoAppearanceRGBColor(Color.BLUE, 0.5));
         rootLink.setLinkGraphics(rootLinkGraphics);
         rootLink.setMass(robotMass);
         rootLink.setMomentOfInertia(momentOfInertia);
         addRootJoint(rootJoint);
         rootJoint.setLink(rootLink);
      }
   }
}
