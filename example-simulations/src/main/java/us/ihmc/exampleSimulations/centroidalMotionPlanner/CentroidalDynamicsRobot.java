package us.ihmc.exampleSimulations.centroidalMotionPlanner;

import java.awt.Color;
import java.util.EnumSet;
import java.util.Set;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.CentroidalMotionPlanner;
import us.ihmc.commonWalkingControlModules.centroidalMotionPlanner.CentroidalMotionPlannerParameters;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotModels.FullRobotModelFactory;
import us.ihmc.robotModels.FullRobotModelFromDescription;
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
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

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
         createSCSRobot();
      return scsRobot;
   }

   private void createSCSRobot()
   {
      scsRobot = new RobotFromDescription(getRobotDescription());
      ExternalForcePoint forcePoint = createExternalForcePointForControl(scsRobot);
      rootJoint = (FloatingJoint) scsRobot.getRootJoints().get(0);
      rootJoint.addExternalForcePoint(forcePoint);
      scsRobot.getGravity(tempVector);
      FullRobotModel controllerFullRobotModel = createFullRobotModel();
      scsRobot.setController(new CentroidalRobotEstimator(rootJoint, controllerFullRobotModel));
      scsRobot.setController(new CentroidalRobotController(forcePoint, tempVector, controllerFullRobotModel, scsRobot.getYoTime()));
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

      public CentroidalRobotController(ExternalForcePoint forcePoint, Vector3D gravity, FullRobotModel controllerFullRobotModel, YoDouble yoTime)
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
         this.stateMachine = new GenericStateMachine<>(namePrefix + "State", namePrefix + "TimeInState", CentroidalRobotStateEnum.class, yoTime, registry);
      }

      @Override
      public void initialize()
      {
         this.motionPlanner = new CentroidalMotionPlanner(parameters);
         setupStateMachine();
      }

      private void setupStateMachine()
      {
         PrintTools.debug("State machine is setup");
         GroundState groundState = new GroundState(forcePoint, gravity, robotModel);
         FlightState flightState = new FlightState(forcePoint, gravity, robotModel);
         GroundToFlightState groundToFlightTransition = new GroundToFlightState(groundState);
         groundState.addStateTransition(CentroidalRobotStateEnum.FLIGHT, groundToFlightTransition);
         FlightToGroundState flightToGroundTransition = new FlightToGroundState(flightState);
         flightState.addStateTransition(CentroidalRobotStateEnum.GROUND, flightToGroundTransition);
         stateMachine.addState(groundState);
         stateMachine.addState(flightState);
         stateMachine.setCurrentState(CentroidalRobotStateEnum.GROUND);
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

      private final FullRobotModel robotModel;

      public CentroidalRobotState(CentroidalRobotStateEnum stateEnum, ExternalForcePoint forcePoint, Vector3D gravity, FullRobotModel robotModel)
      {
         super(stateEnum);
         this.forcePoint = forcePoint;
         this.gravity = gravity;
         this.robotModel = robotModel;
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
      private final CentroidalRobotStateEnum stateEnum;

      private Vector3D forceToExert = new Vector3D();
      
      public GroundState(ExternalForcePoint forcePoint, Vector3D gravity, FullRobotModel robotModel)
      {
         super(CentroidalRobotStateEnum.GROUND, forcePoint, gravity, robotModel);
         this.stateEnum = getStateEnum();
      }

      @Override
      public void doAction()
      {
         super.doControl();
         forceToExert.set(gravity);
         forceToExert.scale(-robotMass);
         forcePoint.setForce(forceToExert);
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

      public boolean transitionToFlight()
      {
         return super.transitionToNextState();
      }
   }

   public class FlightState extends CentroidalRobotState
   {
      private final CentroidalRobotStateEnum stateEnum;

      public FlightState(ExternalForcePoint forcePoint, Vector3D gravity, FullRobotModel robotModel)
      {
         super(CentroidalRobotStateEnum.FLIGHT, forcePoint, gravity, robotModel);
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
         rootLinkGraphics.addEllipsoid(xRadius, yRadius, zRadius, new YoAppearanceRGBColor(Color.BLUE, 0.0));
         rootLink.setLinkGraphics(rootLinkGraphics);
         rootLink.setMass(robotMass);
         rootLink.setMomentOfInertia(momentOfInertia);
         addRootJoint(rootJoint);
         rootJoint.setLink(rootLink);
      }
   }
}
