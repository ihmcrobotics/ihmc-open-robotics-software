package us.ihmc.exampleSimulations.planarWalker;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.Axis3D;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimRigidBody;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.exampleSimulations.planarWalker.BWCPlanarWalkingRobotDefinition;
import us.ihmc.mecano.frames.FixedMovingReferenceFrame;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.MovingZUpFrame;
import us.ihmc.robotics.screwTheory.TotalMassCalculator;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimPrismaticJoint;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimRevoluteJoint;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimFloatingJointBasics;
import us.ihmc.scs2.simulation.robot.trackers.GroundContactPoint;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimRigidBodyBasics;

import static us.ihmc.scs2.definition.visual.ColorDefinitions.*;
import static us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.*;

public class BWCPlanarWalkingRobot implements SCS2YoGraphicHolder
{
  private final SimFloatingJointBasics floatingJoint;
  private final SideDependentList<SimPrismaticJoint> kneeJoints = new SideDependentList<>();
  private final SideDependentList<SimRevoluteJoint> hipJoints = new SideDependentList<>();
  private final SideDependentList<YoDouble> legLengths = new SideDependentList<>();
  private final SideDependentList<YoFrameVector3D> footVelocity = new SideDependentList<>();
  private final SideDependentList<SimRevoluteJoint> armJoints = new SideDependentList<>();
  private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
  private final SideDependentList<MovingReferenceFrame> footFrames = new SideDependentList<>();
  private final double mass;
  private double footMass;
  private final DoubleProvider time;
  private final ReferenceFrame worldFrame;
  private final MovingReferenceFrame centerOfMassFrame;
  private final MovingZUpFrame centerOfMassZUpFrame;
  private final YoFramePoint3D centerOfMassPosition;
  private final YoFrameVector3D centerOfMassVelocity;
  private SimRigidBodyBasics torsoBodyDefinition;

  private RigidBodyDefinition createThigh(String name)
  {
    double thighMass = BWCPlanarWalkingRobotDefinition.THIGH_MASS;
    Matrix3D thighInertia = new Matrix3D();
    thighInertia.setIdentity();
    thighInertia.scale(BWCPlanarWalkingRobotDefinition.THIGH_INERTIA_SCALE);
    Vector3D comOffset = new Vector3D(0, 0, -0.25);  // Center of mass offset

    RigidBodyDefinition thigh = new RigidBodyDefinition(name);
    thigh.setMass(thighMass);
    thigh.setMomentOfInertia(thighInertia);
    thigh.setCenterOfMassOffset(comOffset);
    return thigh;
  }

  private RigidBodyDefinition createShin(String name)
  {
    double shinMass = BWCPlanarWalkingRobotDefinition.SHIN_MASS;
    Matrix3D shinInertia = new Matrix3D();
    shinInertia.setIdentity();
    shinInertia.scale(BWCPlanarWalkingRobotDefinition.SHIN_INERTIA_SCALE);
    Vector3D comOffset = new Vector3D(0, 0, -0.25);  // Center of mass offset

    RigidBodyDefinition shin = new RigidBodyDefinition(name);
    shin.setMass(shinMass);
    shin.setMomentOfInertia(shinInertia);
    shin.setCenterOfMassOffset(comOffset);
    return shin;
  }

  public BWCPlanarWalkingRobot(Robot robot, DoubleProvider time)
  {
    this.time = time;
    this.floatingJoint = robot.getFloatingRootJoint();
    this.torsoBodyDefinition = robot.getRootBody();

    if (this.torsoBodyDefinition == null)
    {
      throw new IllegalStateException("Root body must not be null");
    }

    this.floatingJoint.setJointPosition(new Vector3D(0.0, 0.0, 0.75));
    this.mass = TotalMassCalculator.computeSubTreeMass(this.torsoBodyDefinition);

    this.worldFrame = robot.getInertialFrame();
    this.centerOfMassFrame = this.floatingJoint.getFrameAfterJoint();
    this.centerOfMassZUpFrame = new MovingZUpFrame(centerOfMassFrame, "CenterOfMassZUpFrame");

    this.centerOfMassPosition = new YoFramePoint3D("centerOfMassPosition", worldFrame, registry);
    this.centerOfMassVelocity = new YoFrameVector3D("centerOfMassVelocity", worldFrame, registry);

    initializeJoints();
  }

  private void initializeJoints()
  {
    double footMassLocal = 0.0;

    for (RobotSide robotSide : RobotSide.values())
    {
      SimRevoluteJoint hipJoint = hipJoints.get(robotSide);

      if (hipJoint == null)
      {
        // If not, create and initialize the hip joint
        hipJoint = new SimRevoluteJoint(robotSide.getCamelCaseName() + "Hip", this.torsoBodyDefinition, new Vector3D(0, 1, 0));
        Matrix3D hipInertia = new Matrix3D();
        hipInertia.setIdentity();
        hipInertia.scale(0.01);
        SimRigidBodyBasics hipSuccessor = new SimRigidBody("HipBody", hipJoint, hipInertia, 1.0, new Vector3D(0, 0, -0.1));
        hipJoint.setSuccessor(hipSuccessor);
        hipJoints.put(robotSide, hipJoint);
      }

      if (hipJoint.getSuccessor() == null)
      {
        // Ensure there is a successor body
        RigidBodyDefinition thigh = BWCPlanarWalkingRobotDefinition.createThigh(robotSide.getCamelCaseName() + "Thigh");
        //        SimRigidBodyBasics thighRigidBody = new SimRigidBody(thigh);
        SimRigidBodyBasics thighRigidBody = new SimRigidBody(thigh, hipJoint);

        hipJoint.setSuccessor(thighRigidBody);
      }

      // Create the knee joint
      Vector3D jointOffset = new Vector3D(0, 0, -0.1);
      Vector3D jointAxis = new Vector3D(0, 0, 1);
      SimPrismaticJoint kneeJoint = new SimPrismaticJoint(robotSide.getCamelCaseName() + "Knee", hipJoint.getSuccessor(), jointOffset, jointAxis);
      kneeJoints.put(robotSide, kneeJoint);

      // Create the successor for the knee joint
      RigidBodyDefinition shin = BWCPlanarWalkingRobotDefinition.createShin(robotSide.getCamelCaseName() + "Shin");
      //      SimRigidBodyBasics shinRigidBody = new SimRigidBody(shin);
      SimRigidBodyBasics shinRigidBody = new SimRigidBody(shin, kneeJoint);
      kneeJoint.setSuccessor(shinRigidBody);

      footMassLocal = Math.max(footMassLocal, shinRigidBody.getInertia().getMass());
    }

    this.footMass = footMassLocal;
  }

  private void initializeArmJoints()
  {
    for (RobotSide side : RobotSide.values())
    {
      SimRevoluteJoint armJoint =
          new SimRevoluteJoint(side.getCamelCaseName() + "Arm", this.torsoBodyDefinition, new Vector3D(0, 1, 0));
      armJoints.put(side, armJoint);
    }
  }

  public ReferenceFrame getWorldFrame()
  {
    return worldFrame;
  }

  public YoRegistry getYoRegistry()
  {
    return registry;
  }

  public SimFloatingJointBasics getFloatingJoint()
  {
    return floatingJoint;
  }

  public GroundContactPoint getGroundContactPoint(RobotSide robotSide)
  {
    return kneeJoints.get(robotSide).getAuxiliaryData().getGroundContactPoints().get(0);
  }

  public DoubleProvider getTime()
  {
    return time;
  }

  public double getMass()
  {
    return mass;
  }

  public double getFootMass()
  {
    return footMass;
  }

  public double getLegLength(RobotSide robotSide)
  {
    return legLengths.get(robotSide).getDoubleValue();
  }

  public MovingReferenceFrame getFootFrame(RobotSide robotSide)
  {
    return footFrames.get(robotSide);
  }

  public SimPrismaticJoint getKneeJoint(RobotSide robotSide)
  {
    return kneeJoints.get(robotSide);
  }

  public SimRevoluteJoint getHipJoint(RobotSide robotSide)
  {
    return hipJoints.get(robotSide);
  }

  public MovingReferenceFrame getCenterOfMassFrame()
  {
    return centerOfMassZUpFrame;
  }

  public FramePoint3DReadOnly getCenterOfMassPosition()
  {
    return centerOfMassPosition;
  }

  public FrameVector3DReadOnly getCenterOfMassVelocity()
  {
    return centerOfMassVelocity;
  }

  public SimRevoluteJoint getArmJoint(RobotSide side)
  {
    return armJoints.get(side);
  }

  public FrameVector3DReadOnly getVelocityOfFootRelativeToCoM(RobotSide robotSide)
  {
    return footVelocity.get(robotSide);
  }

//  public void update()
//  {
//    centerOfMassZUpFrame.update();
//    for (RobotSide robotSide : RobotSide.values())
//    {
//      // update the current leg length
//      double restingLegLength = (BWCPlanarWalkingRobotDefinition.thighLength + BWCPlanarWalkingRobotDefinition.shinLength) / 2.0;
//      double currentLegLength = restingLegLength - kneeJoints.get(robotSide).getQ();
//      legLengths.get(robotSide).set(currentLegLength);
//      footFrames.get(robotSide).update();
//      Twist footTwist = new Twist();
//      footFrames.get(robotSide).getTwistRelativeToOther(centerOfMassFrame, footTwist);
//      footVelocity.get(robotSide).setMatchingFrame(footTwist.getLinearPart());
//    }
//
//    centerOfMassPosition.setFromReferenceFrame(centerOfMassFrame);
//    centerOfMassVelocity.setMatchingFrame(centerOfMassFrame.getTwistOfFrame().getLinearPart());
//  }

  public void update() {
    centerOfMassZUpFrame.update();
    for (RobotSide robotSide : RobotSide.values()) {
      // Ensure legLengths and other lists are initialized and have entries for this robotSide
      if (legLengths.get(robotSide) == null) {
        System.err.println("Warning: legLengths not initialized for " + robotSide);
        continue; // Skip this iteration if data is missing
      }

      // Current leg length calculation
      double restingLegLength = (BWCPlanarWalkingRobotDefinition.thighLength + BWCPlanarWalkingRobotDefinition.shinLength) / 2.0;
      double currentLegLength = restingLegLength - kneeJoints.get(robotSide).getQ();
      legLengths.get(robotSide).set(currentLegLength);

      // Update footFrames and calculate foot velocity
      if (footFrames.get(robotSide) != null) {
        footFrames.get(robotSide).update();
        Twist footTwist = new Twist();
        footFrames.get(robotSide).getTwistRelativeToOther(centerOfMassFrame, footTwist);
        footVelocity.get(robotSide).setMatchingFrame(footTwist.getLinearPart());
      } else {
        System.err.println("Warning: footFrames not initialized for " + robotSide);
      }
    }

    centerOfMassPosition.setFromReferenceFrame(centerOfMassFrame);
    centerOfMassVelocity.setMatchingFrame(centerOfMassFrame.getTwistOfFrame().getLinearPart());
  }


  @Override public YoGraphicDefinition getSCS2YoGraphics()
  {
    YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());

    for (RobotSide robotSide : RobotSide.values())
    {
      // Safe access to kinematic points and ground contact points
      if (!hipJoints.get(robotSide).getAuxiliaryData().getKinematicPoints().isEmpty() &&
          !kneeJoints.get(robotSide).getAuxiliaryData().getGroundContactPoints().isEmpty())
      {
        group.addChild(newYoGraphicPoint3D(robotSide.getLowerCaseName() + "GroundPoint",
            kneeJoints.get(robotSide).getAuxiliaryData().getGroundContactPoints().get(0).getPose().getPosition(), 0.01,
            DarkOrange()));
        group.addChild(newYoGraphicCoordinateSystem3D(robotSide.getLowerCaseName() + "KneeFrame",
            kneeJoints.get(robotSide).getAuxiliaryData().getKinematicPoints().get(0).getPose(), 0.075));
        group.addChild(newYoGraphicCoordinateSystem3D(robotSide.getLowerCaseName() + "HipFrame",
            hipJoints.get(robotSide).getAuxiliaryData().getKinematicPoints().get(0).getPose(), 0.075));
      }
      else
      {
        System.out.println("Warning: Kinematic or contact points data missing for " + robotSide);
      }
    }
    group.setVisible(true);
    return group;
  }
}
