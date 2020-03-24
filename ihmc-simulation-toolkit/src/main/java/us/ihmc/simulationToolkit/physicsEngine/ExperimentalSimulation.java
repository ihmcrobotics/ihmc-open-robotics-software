package us.ihmc.simulationToolkit.physicsEngine;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.function.BiConsumer;
import java.util.stream.Stream;

import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.multiBodySystem.interfaces.*;
import us.ihmc.robotics.physics.*;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationconstructionset.*;
import us.ihmc.simulationconstructionset.physics.ScsPhysics;
import us.ihmc.simulationconstructionset.physics.collision.simple.CollisionManager;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;
import us.ihmc.wholeBodyController.DRCRobotJointMap;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class ExperimentalSimulation extends Simulation
{
   private static final long serialVersionUID = -3940628684026932009L;

   private final PhysicsEngine physicsEngine = new PhysicsEngine();

   private Vector3DReadOnly gravity;

   public ExperimentalSimulation(Robot[] robotArray, int dataBufferSize)
   {
      super(robotArray, dataBufferSize);
   }

   public void setGravity(Vector3DReadOnly gravity)
   {
      this.gravity = gravity;
   }

   public void addEnvironmentCollidables(Collection<? extends Collidable> environmentCollidable)
   {
      physicsEngine.addEnvironmentCollidables(environmentCollidable);
   }

   public void addRobot(String robotName, RigidBodyBasics rootBody, RobotCollisionModel robotCollisionModel, MultiBodySystemStateWriter robotInitialStateWriter)
   {
      Robot scsRobot = Stream.of(getRobots()).filter(candidate -> candidate.getName().toLowerCase().equals(robotName.toLowerCase())).findFirst().get();

      MultiBodySystemStateWriter controllerOutputWriter = createControllerOutputWriter(scsRobot);
      MultiBodySystemStateReader physicsOutputWriter = createPhysicsOutputWriter(scsRobot);
      physicsEngine.addRobot(robotName, rootBody, controllerOutputWriter, robotInitialStateWriter, robotCollisionModel, physicsOutputWriter);
   }

   @Override
   protected void simulate() throws UnreasonableAccelerationException
   {
      synchronized (getSimulationSynchronizer())
      {
         for (Robot robot : getRobots())
         {
            robot.updateVelocities();
            robot.doControllers();
         }

         physicsEngine.simulate(getDT(), gravity);

         for (Robot robot : getRobots())
            robot.getYoTime().add(getDT());
      }
   }

   @Override
   public synchronized void simulate(int numTicks) throws UnreasonableAccelerationException
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

   public static List<Collidable> toCollidables(int collisionMask, int collisionGroup, CommonAvatarEnvironmentInterface environment)
   {
      return toCollidables(collisionMask, collisionGroup, environment.getTerrainObject3D());
   }

   public static List<Collidable> toCollidables(int collisionMask, int collisionGroup, TerrainObject3D terrainObject3D)
   {
      List<Collidable> collidables = new ArrayList<>();

      for (Shape3DReadOnly terrainShape : terrainObject3D.getTerrainCollisionShapes())
      {
         collidables.add(new Collidable(null, collisionMask, collisionGroup, terrainShape, ReferenceFrame.getWorldFrame()));
      }

      return collidables;
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
               }
               else if (idJoint instanceof SixDoFJointBasics)
               {
                  FloatingJoint scsJoint = (FloatingJoint) robot.getJoint(idJoint.getName());
                  Pose3DBasics jointPose = ((SixDoFJointBasics) idJoint).getJointPose();
                  jointPose.getOrientation().set(scsJoint.getQuaternion());
                  scsJoint.getPosition(jointPose.getPosition());
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
}
