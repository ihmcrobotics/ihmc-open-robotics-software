package us.ihmc.exampleSimulations.planarWalker.BWC;

import us.ihmc.euclid.referenceFrame.FixedReferenceFrame;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimPrismaticJoint;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimRevoluteJoint;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.sql.Ref;

public class BWCPlanarWalkingRobot
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final SideDependentList<SimPrismaticJoint> kneeJoints = new SideDependentList<>();

   private final SideDependentList<SimRevoluteJoint> hipJoints = new SideDependentList<>();

   private final SideDependentList<YoDouble>  legLengths = new SideDependentList<>();

   private final double restingLegLength;

   private final SideDependentList<ReferenceFrame> footFrames = new SideDependentList<>();

   private final DoubleProvider time;

   private final ReferenceFrame worldFrame;

   private final ReferenceFrame comFrame;


   public BWCPlanarWalkingRobot(Robot robot, DoubleProvider time)
   {
      robot.getFloatingRootJoint().setJointPosition(new Point3D(0.0, 0.0, 0.6));

      this.time = time;

      worldFrame = robot.getInertialFrame();

      //TODO
      comFrame = robot.getJoint(BWCPlanarWalkingRobotDefinition.baseJointName).getFrameAfterJoint();
      restingLegLength = (BWCPlanarWalkingRobotDefinition.shinLength + BWCPlanarWalkingRobotDefinition.thighLength) / 2;

      kneeJoints.put(RobotSide.LEFT, (SimPrismaticJoint) robot.getJoint(BWCPlanarWalkingRobotDefinition.leftKneeName));
      kneeJoints.put(RobotSide.RIGHT, (SimPrismaticJoint) robot.getJoint(BWCPlanarWalkingRobotDefinition.rightKneeName));

      hipJoints.put(RobotSide.LEFT, (SimRevoluteJoint) robot.getJoint(BWCPlanarWalkingRobotDefinition.leftHipPitchName));
      hipJoints.put(RobotSide.RIGHT, (SimRevoluteJoint) robot.getJoint(BWCPlanarWalkingRobotDefinition.rightHipPitchName));

      for (RobotSide robotSide : RobotSide.values())
      {
         legLengths.put(robotSide, new YoDouble(robotSide.getLowerCaseName() + "legLength", registry));

         Vector3D footTranslationFromKnee = new Vector3D();
         footTranslationFromKnee.setZ(-BWCPlanarWalkingRobotDefinition.shinLength / 2.0);

         footFrames.put(robotSide, new FixedReferenceFrame(robotSide.getLowerCaseName() + "FootFrame", kneeJoints.get(robotSide).getFrameAfterJoint(), footTranslationFromKnee));
      }
   }

   public void update()
   {
      for (RobotSide robotSide : RobotSide.values())
      {
         double currentLegLength = restingLegLength - kneeJoints.get(robotSide).getQ();
         legLengths.get(robotSide).set(currentLegLength);
         footFrames.get(robotSide).update();
      }
   }

   public ReferenceFrame getWorldFrame()
   {
      return worldFrame;
   }

   public ReferenceFrame getCenterOfMassFrame()
   {
      return comFrame;
   }

   public ReferenceFrame getFootFrame(RobotSide robotSide)
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

   public double getLegLength(RobotSide robotSide)
   {
      return legLengths.get(robotSide).getDoubleValue();
   }

   public DoubleProvider getTime()
   {
      return time;
   }

   public YoRegistry getYoRegistry()
   {
      return registry;
   }
}

