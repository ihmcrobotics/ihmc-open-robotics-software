package us.ihmc.simulationToolkit.physicsEngine;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.stream.Stream;

import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.PrismaticJoint;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.MultiBodySystemReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.SixDoFJointBasics;
import us.ihmc.mecano.spatial.interfaces.FixedFrameTwistBasics;
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotDataLogger.util.JVMStatisticsGenerator;
import us.ihmc.robotModels.FullRobotModelFactory;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.physics.CollidableHelper;
import us.ihmc.robotics.physics.ExperimentalPhysicsEngine;
import us.ihmc.robotics.physics.MultiBodySystemStateReader;
import us.ihmc.robotics.physics.MultiBodySystemStateWriter;
import us.ihmc.robotics.physics.PhysicsEngineTools;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.robotics.robotDescription.FloatingJointDescription;
import us.ihmc.robotics.robotDescription.JointDescription;
import us.ihmc.robotics.robotDescription.LinkDescription;
import us.ihmc.robotics.robotDescription.PinJointDescription;
import us.ihmc.robotics.robotDescription.RobotDescription;
import us.ihmc.robotics.robotDescription.SliderJointDescription;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.RobotFromDescription;
import us.ihmc.simulationconstructionset.Simulation;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;
import us.ihmc.simulationconstructionset.physics.ScsPhysics;
import us.ihmc.simulationconstructionset.physics.collision.simple.CollisionManager;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;
import us.ihmc.robotics.partNames.HumanoidJointNameMap;
import us.ihmc.wholeBodyController.SimulatedFullHumanoidRobotModelFactory;
import us.ihmc.yoVariables.registry.YoRegistry;

/**
 * SCS wrapper around {@link ExperimentalPhysicsEngine}.
 * <p>
 * This physics engine is <b>experimental</b>, meaning that it is not bug-free, not extensively
 * tested, nor fully compatible with our simulation framework.
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public class ExperimentalSimulation extends Simulation
{
   private static final long serialVersionUID = -3940628684026932009L;

   private final ExperimentalPhysicsEngine physicsEngine = new ExperimentalPhysicsEngine();
   private final SCSRobotExternalWrenchReader externalWrenchReader = new SCSRobotExternalWrenchReader();
   private final SCSRobotExternalForcePointWrapper externalForcePointWrapper = new SCSRobotExternalForcePointWrapper();
   private final SCSRobotIMUSensorReader imuSensorReader = new SCSRobotIMUSensorReader();
   private final SCSRobotTransformUpdater robotTransformUpdater = new SCSRobotTransformUpdater();

   private Vector3DReadOnly gravity;

   private final List<RigidBodyBasics> rootBodies = new ArrayList<>();

   /** List of tasks to be executed right before executing the physics engine's tick. */
   private final List<Runnable> preProcessors = new ArrayList<>();
   /** List of tasks to be executed right after executing the physics engine's tick. */
   private final List<Runnable> postProcessors = new ArrayList<>();

   private boolean initialize = true;

   public ExperimentalSimulation(int dataBufferSize)
   {
      this(null, dataBufferSize);
   }

   public ExperimentalSimulation(Robot[] robotArray, int dataBufferSize)
   {
      super(robotArray, dataBufferSize);
      physicsEngine.addExternalWrenchReader(externalWrenchReader);
      physicsEngine.addExternalWrenchProvider(externalForcePointWrapper);
      physicsEngine.addInertialMeasurementReader(imuSensorReader);
   }

   public void setGravity(Vector3DReadOnly gravity)
   {
      this.gravity = gravity;
   }

   public void addEnvironmentCollidables(CollidableHelper helper, String robotCollisionMask, String environmentCollisionMask,
                                         CommonAvatarEnvironmentInterface environment)
   {
      addEnvironmentCollidables(toCollidables(helper.getCollisionMask(environmentCollisionMask), helper.createCollisionGroup(robotCollisionMask), environment));
   }

   public void addEnvironmentCollidable(Collidable environmentCollidable)
   {
      physicsEngine.addEnvironmentCollidable(environmentCollidable);
   }

   public void addEnvironmentCollidables(Collection<? extends Collidable> environmentCollidables)
   {
      physicsEngine.addEnvironmentCollidables(environmentCollidables);
   }

   /**
    * Adds and configures a new robot to the simulation.
    */
   public void addRobot(RobotDescription robotDescription, RobotCollisionModel robotCollisionModel, MultiBodySystemStateWriter robotInitialStateWriter)
   {
      RobotFromDescription scsRobot = new RobotFromDescription(robotDescription);
      RigidBodyBasics rootBody = toInverseDynamicsRobot(robotDescription);

      MultiBodySystemStateWriter controllerOutputWriter = createControllerOutputWriter(scsRobot);
      MultiBodySystemStateWriter physicsInputStateWriter = toMultiBodySystemStateWriter(scsRobot);
      MultiBodySystemStateReader physicsOutputStateReader = createPhysicsOutputStateReader(scsRobot);

      rootBodies.add(rootBody);
      physicsEngine.addRobot(robotDescription.getName(),
                             rootBody,
                             controllerOutputWriter,
                             robotInitialStateWriter,
                             robotCollisionModel,
                             physicsInputStateWriter,
                             physicsOutputStateReader);
      externalWrenchReader.addRobot(rootBody, scsRobot);
      externalForcePointWrapper.addRobot(rootBody, scsRobot);
      imuSensorReader.addRobot(rootBody, scsRobot);
      robotTransformUpdater.addRobot(rootBody, scsRobot);
      addRobot(scsRobot);
   }

   /**
    * Adds and configures a new robot to the simulation.
    */
   public void addRobot(RobotDescription robotDescription, RobotCollisionModel robotCollisionModel, MultiBodySystemStateWriter robotInitialStateWriter,
                        MultiBodySystemStateWriter controllerOutputWriter)
   {
      RobotFromDescription scsRobot = new RobotFromDescription(robotDescription);
      RigidBodyBasics rootBody = toInverseDynamicsRobot(robotDescription);

      MultiBodySystemStateWriter physicsInputStateWriter = toMultiBodySystemStateWriter(scsRobot);
      MultiBodySystemStateReader physicsOutputStateReader = createPhysicsOutputStateReader(scsRobot);

      rootBodies.add(rootBody);
      physicsEngine.addRobot(robotDescription.getName(),
                             rootBody,
                             controllerOutputWriter,
                             robotInitialStateWriter,
                             robotCollisionModel,
                             physicsInputStateWriter,
                             physicsOutputStateReader);
      externalWrenchReader.addRobot(rootBody, scsRobot);
      externalForcePointWrapper.addRobot(rootBody, scsRobot);
      imuSensorReader.addRobot(rootBody, scsRobot);
      robotTransformUpdater.addRobot(rootBody, scsRobot);
      addRobot(scsRobot);
   }

   /**
    * Configures the physics for a robot that was already added via the constructor or
    * {@link #addRobot(Robot)}.
    */
   public void configureRobot(FullRobotModelFactory robotFactory, RobotCollisionModel robotCollisionModel, MultiBodySystemStateWriter robotInitialStateWriter)
   {
      String robotName = robotFactory.getRobotDescription().getName();
      RigidBodyBasics rootBody = robotFactory.createFullRobotModel().getElevator();
      configureRobot(robotName, rootBody, robotCollisionModel, robotInitialStateWriter);
   }

   /**
    * Configures the physics for a robot that was already added via the constructor or
    * {@link #addRobot(Robot)}.
    */
   public void configureRobot(String robotName, RigidBodyBasics rootBody, RobotCollisionModel robotCollisionModel,
                              MultiBodySystemStateWriter robotInitialStateWriter)
   {
      Robot scsRobot = Stream.of(getRobots()).filter(candidate -> candidate.getName().toLowerCase().equals(robotName.toLowerCase())).findFirst().get();

      MultiBodySystemStateWriter controllerOutputWriter = createControllerOutputWriter(scsRobot);
      MultiBodySystemStateWriter physicsInputStateWriter = toMultiBodySystemStateWriter(scsRobot);
      MultiBodySystemStateReader physicsOutputStateReader = createPhysicsOutputStateReader(scsRobot);
      rootBodies.add(rootBody);
      physicsEngine.addRobot(robotName,
                             rootBody,
                             controllerOutputWriter,
                             robotInitialStateWriter,
                             robotCollisionModel,
                             physicsInputStateWriter,
                             physicsOutputStateReader);
      externalWrenchReader.addRobot(rootBody, scsRobot);
      externalForcePointWrapper.addRobot(rootBody, scsRobot);
      imuSensorReader.addRobot(rootBody, scsRobot);
      robotTransformUpdater.addRobot(rootBody, scsRobot);
   }

   public void addPreProcessor(Runnable preProcessor)
   {
      preProcessors.add(preProcessor);
   }

   public void addPostProcessor(Runnable postProcessor)
   {
      postProcessors.add(postProcessor);
   }

   public void addJVMStatistics()
   {
      JVMStatisticsGenerator jvmStatisticsGenerator = new JVMStatisticsGenerator(getPhysicsEngineRegistry());
      postProcessors.add(() -> jvmStatisticsGenerator.runManual());
   }

   public void addSimulationEnergyStatistics()
   {
      SimulationEnergyStatistics.setupSimulationEnergyStatistics(gravity, physicsEngine);
   }

   public void initialize()
   {
      initialize = false;

      synchronized (getSimulationSynchronizer())
      {
         robotTransformUpdater.update();

         externalWrenchReader.initialize();
         physicsEngine.initialize();
         externalWrenchReader.updateSCSGroundContactPoints();

         Robot[] robots = getRobots();

         for (int i = 0; i < robots.length; i++)
         {
            Robot robot = robots[i];
            updateGroundContactPointsVelocity(rootBodies.get(i), robot);
         }

         getDataBuffer().fillBuffer();
      }
   }

   @Override
   public void simulate()
   {
      if (initialize)
         initialize();

      preProcessors.forEach(Runnable::run);

      synchronized (getSimulationSynchronizer())
      {
         Robot[] robots = getRobots();
         robotTransformUpdater.update();

         for (int i = 0; i < robots.length; i++)
         {
            Robot robot = robots[i];
            robot.doControllers();
         }

         externalWrenchReader.initialize();
         physicsEngine.simulate(getDT(), gravity);
         externalWrenchReader.updateSCSGroundContactPoints();

         for (int i = 0; i < robots.length; i++)
         {
            Robot robot = robots[i];
            updateGroundContactPointsVelocity(rootBodies.get(i), robot);
            robot.getYoTime().add(getDT());
         }
      }

      postProcessors.forEach(Runnable::run);
   }

   @Override
   public synchronized void simulate(int numTicks)
   {
      while (numTicks > 0)
      {
         for (int i = 0; i < getRecordFreq(); i++)
         {
            simulate();

            if (checkSimulateDoneCriterion())
               numTicks = -1;
         }

         getDataBuffer().tickAndWriteIntoBuffer();
         numTicks -= getRecordFreq();
      }

      // Notify all the listeners that the simulation stopped...
      notifySimulateDoneListeners();
   }

   @Override
   public void closeAndDispose()
   {
      super.closeAndDispose();
   }

   private MultiBodySystemStateReader createPhysicsOutputStateReader(Robot scsRobot)
   {
      return new MultiBodySystemStateReader()
      {
         private final List<Runnable> stateCopiers = new ArrayList<>();

         @Override
         public void read()
         {
            stateCopiers.forEach(Runnable::run);
         }

         @Override
         public void setMultiBodySystem(MultiBodySystemReadOnly multiBodySystem)
         {
            for (JointReadOnly joint : multiBodySystem.getAllJoints())
            {
               if (joint instanceof OneDoFJointReadOnly)
               {
                  OneDoFJointBasics idJoint = (OneDoFJointBasics) joint;
                  OneDegreeOfFreedomJoint scsJoint = (OneDegreeOfFreedomJoint) scsRobot.getJoint(idJoint.getName());

                  stateCopiers.add(new Runnable()
                  {
                     @Override
                     public void run()
                     {
                        scsJoint.setQ(idJoint.getQ());
                        scsJoint.setQd(idJoint.getQd());
                        scsJoint.setQdd(idJoint.getQdd());
                        scsJoint.setTau(idJoint.getTau());
                     }
                  });
               }
               else if (joint instanceof FloatingJointReadOnly)
               {
                  FloatingJointReadOnly idJoint = (FloatingJointReadOnly) joint;
                  FloatingJoint scsJoint = (FloatingJoint) scsRobot.getJoint(joint.getName());

                  stateCopiers.add(new Runnable()
                  {
                     private final FrameVector3D linearVelocity = new FrameVector3D();
                     private final FrameVector3D linearAcceleration = new FrameVector3D();

                     @Override
                     public void run()
                     {
                        scsJoint.setPosition(idJoint.getJointPose().getPosition());
                        scsJoint.setOrientation(idJoint.getJointPose().getOrientation());

                        linearVelocity.setMatchingFrame(idJoint.getJointTwist().getLinearPart());

                        scsJoint.setAngularVelocityInBody(idJoint.getJointTwist().getAngularPart());
                        scsJoint.setVelocity(linearVelocity);

                        idJoint.getJointAcceleration().getLinearAccelerationAtBodyOrigin(idJoint.getJointTwist(), linearAcceleration);
                        linearAcceleration.changeFrame(ReferenceFrame.getWorldFrame());

                        scsJoint.setAngularAccelerationInBody(idJoint.getJointAcceleration().getAngularPart());
                        scsJoint.setAcceleration(linearAcceleration);
                     }
                  });
               }
            }
         }
      };
   }

   private MultiBodySystemStateWriter createControllerOutputWriter(Robot scsRobot)
   {
      return new MultiBodySystemStateWriter()
      {
         private final List<Runnable> stateCopiers = new ArrayList<>();

         @Override
         public boolean write()
         {
            stateCopiers.forEach(Runnable::run);
            return true;
         }

         @Override
         public void setMultiBodySystem(MultiBodySystemBasics multiBodySystem)
         {
            for (JointBasics joint : multiBodySystem.getJointsToConsider())
            {
               if (joint instanceof OneDoFJointBasics)
               {
                  OneDoFJointBasics idJoint = (OneDoFJointBasics) joint;
                  OneDegreeOfFreedomJoint scsJoint = (OneDegreeOfFreedomJoint) scsRobot.getJoint(joint.getName());
                  stateCopiers.add(() -> idJoint.setTau(scsJoint.getTau()));
               }
            }
         }
      };
   }

   @Override
   protected void doControl()
   {
      throw new UnsupportedOperationException();
   }

   @Override
   protected void doDynamicsAndIntegrate() throws UnreasonableAccelerationException
   {
      throw new UnsupportedOperationException();
   }

   @Override
   public void initializeShapeCollision(CollisionManager collisionManager)
   {
      throw new UnsupportedOperationException();
   }

   @Override
   public void initPhysics(ScsPhysics physics)
   {
      throw new UnsupportedOperationException();
   }

   public YoRegistry getPhysicsEngineRegistry()
   {
      return physicsEngine.getPhysicsEngineRegistry();
   }

   public YoGraphicsListRegistry getPhysicsEngineGraphicsRegistry()
   {
      return physicsEngine.getPhysicsEngineGraphicsRegistry();
   }

   public static List<Collidable> toCollidables(long collisionMask, long collisionGroup, CommonAvatarEnvironmentInterface environment)
   {
      return toCollidables(collisionMask, collisionGroup, environment.getTerrainObject3D());
   }

   public static List<Collidable> toCollidables(long collisionMask, long collisionGroup, TerrainObject3D terrainObject3D)
   {
      List<Collidable> collidables = new ArrayList<>();

      for (Shape3DReadOnly terrainShape : terrainObject3D.getTerrainCollisionShapes())
      {
         collidables.add(new Collidable(null,
                                        collisionMask,
                                        collisionGroup,
                                        PhysicsEngineTools.toFrameShape3DBasics(ReferenceFrame.getWorldFrame(), terrainShape)));
      }

      return collidables;
   }

   public static MultiBodySystemStateWriter toRobotInitialStateWriter(BiConsumer<HumanoidFloatingRootJointRobot, HumanoidJointNameMap> initialSetup,
                                                                      SimulatedFullHumanoidRobotModelFactory robotFactory, HumanoidJointNameMap jointMap)
   {
      return toRobotInitialStateWriter(initialSetup, robotFactory.createHumanoidFloatingRootJointRobot(false), jointMap);
   }

   public static <T extends Robot> MultiBodySystemStateWriter toRobotInitialStateWriter(BiConsumer<T, HumanoidJointNameMap> initialSetup, T robot,
                                                                                        HumanoidJointNameMap jointMap)
   {
      initialSetup.accept(robot, jointMap);
      return toMultiBodySystemStateWriter(robot);
   }

   private static MultiBodySystemStateWriter toMultiBodySystemStateWriter(Robot robot)
   {
      return new MultiBodySystemStateWriter()
      {
         private List<? extends JointBasics> allIDJoints;
         private final FrameVector3D linearVelocity = new FrameVector3D();
         private final double epsilon = 1.0e-10;

         @Override
         public boolean write()
         {
            boolean stateChanged = false;

            for (JointBasics idJoint : allIDJoints)
            {
               if (idJoint instanceof OneDoFJointBasics)
               {
                  OneDegreeOfFreedomJoint scsOneDoFJoint = (OneDegreeOfFreedomJoint) robot.getJoint(idJoint.getName());
                  OneDoFJointBasics idOneDoFJoint = (OneDoFJointBasics) idJoint;

                  if (!stateChanged)
                     stateChanged = !EuclidCoreTools.epsilonEquals(idOneDoFJoint.getQ(), scsOneDoFJoint.getQ(), epsilon)
                           || !EuclidCoreTools.epsilonEquals(idOneDoFJoint.getQd(), scsOneDoFJoint.getQD(), epsilon);

                  idOneDoFJoint.setQ(scsOneDoFJoint.getQ());
                  idOneDoFJoint.setQd(scsOneDoFJoint.getQD());
               }
               else if (idJoint instanceof SixDoFJointBasics)
               {
                  FloatingJoint scsSixDoFJoint = (FloatingJoint) robot.getJoint(idJoint.getName());
                  SixDoFJointBasics idSixDoFJoint = (SixDoFJointBasics) idJoint;

                  Pose3DBasics jointPose = idSixDoFJoint.getJointPose();
                  FixedFrameTwistBasics jointTwist = idSixDoFJoint.getJointTwist();

                  if (!stateChanged)
                     stateChanged = !jointPose.getPosition().epsilonEquals(scsSixDoFJoint.getPosition(), epsilon);
                  if (!stateChanged)
                     stateChanged = !jointPose.getOrientation().epsilonEquals(scsSixDoFJoint.getQuaternion(), epsilon);

                  jointPose.getOrientation().set(scsSixDoFJoint.getQuaternion());
                  scsSixDoFJoint.getPosition(jointPose.getPosition());

                  scsSixDoFJoint.getVelocity(linearVelocity);
                  linearVelocity.changeFrame(idSixDoFJoint.getFrameAfterJoint());

                  if (!stateChanged)
                     stateChanged = !jointTwist.getLinearPart().epsilonEquals(linearVelocity, epsilon);
                  if (!stateChanged)
                     stateChanged = !jointTwist.getAngularPart().epsilonEquals(scsSixDoFJoint.getAngularVelocityInBody(), epsilon);

                  jointTwist.getLinearPart().set(linearVelocity);
                  jointTwist.getAngularPart().set(scsSixDoFJoint.getAngularVelocityInBody());
               }
               else
               {
                  throw new UnsupportedOperationException("Unsupported joint type: " + idJoint);
               }
            }
            return stateChanged;
         }

         @Override
         public void setMultiBodySystem(MultiBodySystemBasics multiBodySystem)
         {
            allIDJoints = multiBodySystem.getAllJoints();
         }
      };
   }

   private static void updateGroundContactPointsVelocity(RigidBodyReadOnly rootBody, Robot scsRobot)
   {
      List<GroundContactPoint> scsGroundContactPoints = scsRobot.getAllGroundContactPoints();
      JointReadOnly[] allJoint = MultiBodySystemTools.collectSubtreeJoints(rootBody);
      FramePoint3D position = new FramePoint3D();
      FrameVector3D linearVelocity = new FrameVector3D();
      FrameVector3D angularVelocity = new FrameVector3D();

      for (GroundContactPoint scsGroundContactPoint : scsGroundContactPoints)
      {
         Joint scsJoint = scsGroundContactPoint.getParentJoint();
         scsGroundContactPoint.getOffset(position);
         JointReadOnly joint = Stream.of(allJoint).filter(candidate -> candidate.getName().equals(scsJoint.getName())).findFirst().get();
         position.setReferenceFrame(joint.getFrameAfterJoint());
         TwistReadOnly twistOfFrame = joint.getFrameAfterJoint().getTwistOfFrame();
         twistOfFrame.getLinearVelocityAt(position, linearVelocity);
         linearVelocity.changeFrame(ReferenceFrame.getWorldFrame());
         angularVelocity.setMatchingFrame(twistOfFrame.getAngularPart());
         scsGroundContactPoint.setVelocity(linearVelocity);
         scsGroundContactPoint.setAngularVelocity(angularVelocity);
      }
   }

   static RigidBodyBasics toInverseDynamicsRobot(RobotDescription description)
   {
      RigidBody rootBody = new RigidBody("elevator", ReferenceFrame.getWorldFrame());
      for (JointDescription rootJoint : description.getRootJoints())
         addJointRecursive(rootJoint, rootBody);
      return rootBody;
   }

   static void addJointRecursive(JointDescription jointDescription, RigidBodyBasics parentBody)
   {
      JointBasics joint;
      String name = jointDescription.getName();
      Vector3D jointOffset = new Vector3D();
      jointDescription.getOffsetFromParentJoint(jointOffset);

      if (jointDescription instanceof PinJointDescription)
      {
         Vector3D jointAxis = new Vector3D();
         ((PinJointDescription) jointDescription).getJointAxis(jointAxis);
         joint = new RevoluteJoint(name, parentBody, jointOffset, jointAxis);
      }
      else if (jointDescription instanceof SliderJointDescription)
      {
         Vector3D jointAxis = new Vector3D();
         ((SliderJointDescription) jointDescription).getJointAxis(jointAxis);
         joint = new PrismaticJoint(name, parentBody, jointOffset, jointAxis);
      }
      else if (jointDescription instanceof FloatingJointDescription)
      {
         RigidBodyTransform transformToParent = new RigidBodyTransform();
         transformToParent.getTranslation().set(jointOffset);
         joint = new SixDoFJoint(name, parentBody, transformToParent);
      }
      else
      {
         throw new IllegalStateException("Joint type not handled.");
      }

      LinkDescription linkDescription = jointDescription.getLink();

      String bodyName = linkDescription.getName();
      Matrix3DReadOnly momentOfInertia = linkDescription.getMomentOfInertiaCopy();
      double mass = linkDescription.getMass();
      Tuple3DReadOnly centerOfMassOffset = linkDescription.getCenterOfMassOffset();
      RigidBody successor = new RigidBody(bodyName, joint, momentOfInertia, mass, centerOfMassOffset);
      joint.setSuccessor(successor);

      for (JointDescription childJoint : jointDescription.getChildrenJoints())
         addJointRecursive(childJoint, successor);
   }

   public ExperimentalPhysicsEngine getPhysicsEngine()
   {
      return physicsEngine;
   }
}
