package us.ihmc.exampleSimulations.planarWalker;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
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

import static us.ihmc.scs2.definition.visual.ColorDefinitions.*;
import static us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinitionFactory.*;

// Stand in for the full robot model
public class BWCPlanarWalkingRobot implements SCS2YoGraphicHolder
{
   private final SimFloatingJointBasics floatingJoint;
   private final SideDependentList<SimPrismaticJoint> kneeJoints;
   private final SideDependentList<SimRevoluteJoint> hipPitchJoints;
   private final SideDependentList<SimRevoluteJoint> hipRollJoints;

   private final SideDependentList<YoDouble> legLengths = new SideDependentList<YoDouble>();
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final SideDependentList<MovingReferenceFrame> footFrames = new SideDependentList<>();

   private final double mass;
   private final double footMass;
   private final DoubleProvider time;
   private final ReferenceFrame worldFrame;

   private final MovingReferenceFrame centerOfMassFrame;
   private final MovingZUpFrame centerOfMassZUpFrame;
   private final YoFramePoint3D centerOfMassPosition;
   private final YoFrameVector3D centerOfMassVelocity;
   private final SideDependentList<YoFrameVector3D> footVelocity = new SideDependentList<>();

   public BWCPlanarWalkingRobot(Robot robot, DoubleProvider time)
   {
      this.time = time;
      floatingJoint = robot.getFloatingRootJoint();
      floatingJoint.setJointPosition(new Vector3D(0.0, 0.0, 0.75));
      mass = TotalMassCalculator.computeSubTreeMass(robot.getRootBody());

      worldFrame = robot.getInertialFrame();
      kneeJoints = new SideDependentList<>();
      hipPitchJoints = new SideDependentList<>();
      hipRollJoints = new SideDependentList<>();

      centerOfMassPosition = new YoFramePoint3D("centerOfMassPosition", worldFrame, registry);
      centerOfMassVelocity = new YoFrameVector3D("centerOfMassVelocity", worldFrame, registry);

      // FIXME use the center of mass jacobian calculator for this.
      centerOfMassFrame = robot.getJoint(BWCPlanarWalkingRobotDefinition.baseJointName).getFrameAfterJoint();
      centerOfMassZUpFrame = new MovingZUpFrame(centerOfMassFrame, "CenterOfMassZUpFrame");

      double footMassLocal = 0.0;
      for (RobotSide robotSide : RobotSide.values)
      {
         SimPrismaticJoint kneeJoint = (SimPrismaticJoint) robot.getJoint(BWCPlanarWalkingRobotDefinition.kneeNames.get(robotSide));
         SimRevoluteJoint hipPitchJoint = (SimRevoluteJoint) robot.getJoint(BWCPlanarWalkingRobotDefinition.hipPitchNames.get(robotSide));
         SimRevoluteJoint hipRollJoint = (SimRevoluteJoint) robot.getJoint(BWCPlanarWalkingRobotDefinition.hipRollNames.get(robotSide));
         kneeJoints.put(robotSide, kneeJoint);
         hipPitchJoints.put(robotSide, hipPitchJoint);
         hipRollJoints.put(robotSide, hipRollJoint);

         footMassLocal = Math.max(footMassLocal, kneeJoint.getSuccessor().getInertia().getMass());

         footVelocity.put(robotSide,  new YoFrameVector3D(robotSide.getLowerCaseName() + "FootVelocity", centerOfMassFrame, registry));

         YoDouble legLength = new YoDouble(robotSide.getLowerCaseName() + "LegLength", registry);
         legLengths.put(robotSide, legLength);

         Vector3D footTranslationFromKnee = new Vector3D();
         footTranslationFromKnee.setZ(-BWCPlanarWalkingRobotDefinition.shinLength / 2.0);
         MovingReferenceFrame footFrame = new FixedMovingReferenceFrame(robotSide.getLowerCaseName() + "FootFrame", kneeJoint.getFrameAfterJoint(), footTranslationFromKnee);
         footFrames.put(robotSide, footFrame);
      }
      kneeJoints.get(RobotSide.LEFT).setQ(0.25);
      footMass = footMassLocal;
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

   public SimRevoluteJoint getHipPitchJoint(RobotSide robotSide)
   {
      return hipPitchJoints.get(robotSide);
   }

   public SimRevoluteJoint getHipRollJoint(RobotSide robotSide)
   {
      return hipRollJoints.get(robotSide);
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

   public FrameVector3DReadOnly getVelocityOfFootRelativeToCoM(RobotSide robotSide)
   {
      return footVelocity.get(robotSide);
   }

   public void update()
   {
      centerOfMassZUpFrame.update();
      for (RobotSide robotSide : RobotSide.values)
      {
         // update the current leg length
         double restingLegLength = (BWCPlanarWalkingRobotDefinition.thighLength + BWCPlanarWalkingRobotDefinition.shinLength) / 2.0;
         double currentLegLength = restingLegLength - kneeJoints.get(robotSide).getQ();

         legLengths.get(robotSide).set(currentLegLength);

         footFrames.get(robotSide).update();

         Twist footTwist = new Twist();
         footFrames.get(robotSide).getTwistRelativeToOther(centerOfMassFrame, footTwist);
         footVelocity.get(robotSide).setMatchingFrame(footTwist.getLinearPart()); //TODO: does the y direction need a sign change?
      }

      centerOfMassPosition.setFromReferenceFrame(centerOfMassFrame);
      centerOfMassVelocity.setMatchingFrame(centerOfMassFrame.getTwistOfFrame().getLinearPart());
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      group.addChild(newYoGraphicCoordinateSystem3D("BasePoint", floatingJoint.getAuxiliaryData().getKinematicPoints().get(0).getPose(), 0.25));
      for (RobotSide robotSide : RobotSide.values)
      {
         group.addChild(newYoGraphicPoint3D(robotSide.getLowerCaseName() + "GroundPoint", kneeJoints.get(robotSide).getAuxiliaryData().getGroundContactPoints().get(0).getPose().getPosition(), 0.01, DarkOrange()));
         group.addChild(newYoGraphicCoordinateSystem3D(robotSide.getLowerCaseName() + "KneeFrame", kneeJoints.get(robotSide).getAuxiliaryData().getKinematicPoints().get(0).getPose(), 0.075));
         group.addChild(newYoGraphicCoordinateSystem3D(robotSide.getLowerCaseName() + "HipPitchFrame", hipPitchJoints.get(robotSide).getAuxiliaryData().getKinematicPoints().get(0).getPose(), 0.075));
         group.addChild(newYoGraphicCoordinateSystem3D(robotSide.getLowerCaseName() + "HipRollFrame", hipRollJoints.get(robotSide).getAuxiliaryData().getKinematicPoints().get(0).getPose(), 0.075));
      }
      group.setVisible(true);
      return group;
   }
}
