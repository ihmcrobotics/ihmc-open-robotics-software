package us.ihmc.simulationToolkit.physicsEngine;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.stream.Stream;

import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
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
import us.ihmc.mecano.spatial.interfaces.TwistReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotics.physics.Collidable;
import us.ihmc.robotics.physics.CollidableHelper;
import us.ihmc.robotics.physics.ExperimentalPhysicsEngine;
import us.ihmc.robotics.physics.MultiBodySystemStateReader;
import us.ihmc.robotics.physics.MultiBodySystemStateWriter;
import us.ihmc.robotics.physics.PhysicsEngineTools;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.Simulation;
import us.ihmc.simulationconstructionset.UnreasonableAccelerationException;
import us.ihmc.simulationconstructionset.physics.ScsPhysics;
import us.ihmc.simulationconstructionset.physics.collision.simple.CollisionManager;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;
import us.ihmc.wholeBodyController.DRCRobotJointMap;
import us.ihmc.wholeBodyController.SimulatedFullHumanoidRobotModelFactory;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

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

   private Vector3DReadOnly gravity;

   private final List<RigidBodyBasics> rootBodies = new ArrayList<>();

   public ExperimentalSimulation(Robot[] robotArray, int dataBufferSize)
   {
      super(robotArray, dataBufferSize);
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

   public void addEnvironmentCollidables(Collection<? extends Collidable> environmentCollidable)
   {
      physicsEngine.addEnvironmentCollidables(environmentCollidable);
      physicsEngine.addExternalWrenchReader(externalWrenchReader);
   }

   public void addRobot(FullHumanoidRobotModelFactory robotFactory, RobotCollisionModel robotCollisionModel, MultiBodySystemStateWriter robotInitialStateWriter)
   {
      String robotName = robotFactory.getRobotDescription().getName();
      RigidBodyBasics rootBody = robotFactory.createFullRobotModel().getElevator();
      addRobot(robotName, rootBody, robotCollisionModel, robotInitialStateWriter);
   }

   public void addRobot(String robotName, RigidBodyBasics rootBody, RobotCollisionModel robotCollisionModel, MultiBodySystemStateWriter robotInitialStateWriter)
   {
      Robot scsRobot = Stream.of(getRobots()).filter(candidate -> candidate.getName().toLowerCase().equals(robotName.toLowerCase())).findFirst().get();

      MultiBodySystemStateWriter controllerOutputWriter = createControllerOutputWriter(scsRobot);
      MultiBodySystemStateReader physicsOutputWriter = createPhysicsOutputWriter(scsRobot);
      rootBodies.add(rootBody);
      physicsEngine.addRobot(robotName, rootBody, controllerOutputWriter, robotInitialStateWriter, robotCollisionModel, physicsOutputWriter);
      externalWrenchReader.addRobot(rootBody, scsRobot);
   }

   @Override
   public void simulate()
   {
      synchronized (getSimulationSynchronizer())
      {
         for (Robot robot : getRobots())
         {
            robot.update();
            robot.doControllers();
         }

         externalWrenchReader.initialize();
         physicsEngine.simulate(getDT(), gravity);
         externalWrenchReader.updateSCSGroundContactPoints();

         Robot[] robots = getRobots();

         for (int i = 0; i < robots.length; i++)
         {
            Robot robot = robots[i];
            updateGroundContactPointsVelocity(rootBodies.get(i), robot);
            robot.getYoTime().add(getDT());
         }
      }
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

         getDataBuffer().tickAndUpdate();
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

   private MultiBodySystemStateReader createPhysicsOutputWriter(Robot scsRobot)
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
                     @Override
                     public void run()
                     {
                        scsJoint.setPosition(idJoint.getJointPose().getPosition());
                        scsJoint.setQuaternion(idJoint.getJointPose().getOrientation());
                        scsJoint.setAngularVelocityInBody(idJoint.getJointTwist().getAngularPart());
                        FrameVector3D linearVelocity = new FrameVector3D(idJoint.getJointTwist().getLinearPart());
                        linearVelocity.changeFrame(ReferenceFrame.getWorldFrame());
                        scsJoint.setVelocity(linearVelocity);
                        scsJoint.setAngularAccelerationInBody(idJoint.getJointAcceleration().getAngularPart());
                        FrameVector3D linearAcceleration = new FrameVector3D(idJoint.getJointAcceleration().getLinearPart());
                        linearAcceleration.changeFrame(ReferenceFrame.getWorldFrame());
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
         public void write()
         {
            stateCopiers.forEach(Runnable::run);
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

   public YoVariableRegistry getPhysicsEngineRegistry()
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

   public static MultiBodySystemStateWriter toRobotInitialStateWriter(BiConsumer<HumanoidFloatingRootJointRobot, DRCRobotJointMap> initialSetup,
                                                                      SimulatedFullHumanoidRobotModelFactory robotFactory, DRCRobotJointMap jointMap)
   {
      return toRobotInitialStateWriter(initialSetup, robotFactory.createHumanoidFloatingRootJointRobot(false), jointMap);
   }

   public static <T extends Robot> MultiBodySystemStateWriter toRobotInitialStateWriter(BiConsumer<T, DRCRobotJointMap> initialSetup, T robot,
                                                                                        DRCRobotJointMap jointMap)
   {
      initialSetup.accept(robot, jointMap);

      return new MultiBodySystemStateWriter()
      {
         private List<? extends JointBasics> allIDJoints;

         @Override
         public void write()
         {
            for (JointBasics idJoint : allIDJoints)
            {
               if (idJoint instanceof OneDoFJointBasics)
               {
                  OneDegreeOfFreedomJoint scsJoint = (OneDegreeOfFreedomJoint) robot.getJoint(idJoint.getName());
                  ((OneDoFJointBasics) idJoint).setQ(scsJoint.getQ());
                  ((OneDoFJointBasics) idJoint).setQd(scsJoint.getQD());
               }
               else if (idJoint instanceof SixDoFJointBasics)
               {
                  FloatingJoint scsJoint = (FloatingJoint) robot.getJoint(idJoint.getName());
                  SixDoFJointBasics idSixDoFJoint = (SixDoFJointBasics) idJoint;

                  Pose3DBasics jointPose = idSixDoFJoint.getJointPose();
                  jointPose.getOrientation().set(scsJoint.getQuaternion());
                  scsJoint.getPosition(jointPose.getPosition());

                  FrameVector3D linearVelocity = new FrameVector3D(ReferenceFrame.getWorldFrame());
                  scsJoint.getVelocity(linearVelocity);
                  linearVelocity.changeFrame(idSixDoFJoint.getFrameAfterJoint());
                  idSixDoFJoint.getJointTwist().getLinearPart().set(linearVelocity);
                  idSixDoFJoint.getJointTwist().getAngularPart().set(scsJoint.getAngularVelocityInBody());
               }
               else
               {
                  throw new UnsupportedOperationException("Unsupported joint type: " + idJoint);
               }
            }
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
      ArrayList<GroundContactPoint> scsGroundContactPoints = scsRobot.getAllGroundContactPoints();
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

   public ExperimentalPhysicsEngine getPhysicsEngine()
   {
      return physicsEngine;
   }
}
