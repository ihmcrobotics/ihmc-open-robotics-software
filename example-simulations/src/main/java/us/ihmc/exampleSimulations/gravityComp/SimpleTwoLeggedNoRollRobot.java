package us.ihmc.exampleSimulations.gravityComp;

import static us.ihmc.exampleSimulations.gravityComp.SimpleLegRobotDefinition.anklePitchBodyName;
import static us.ihmc.exampleSimulations.gravityComp.SimpleLegRobotDefinition.anklePitchJointName;
import static us.ihmc.exampleSimulations.gravityComp.SimpleLegRobotDefinition.ankleRollJointName;
import static us.ihmc.exampleSimulations.gravityComp.SimpleLegRobotDefinition.footBodyName;
import static us.ihmc.exampleSimulations.gravityComp.SimpleLegRobotDefinition.footSize;
import static us.ihmc.exampleSimulations.gravityComp.SimpleLegRobotDefinition.hipPitchJointName;
import static us.ihmc.exampleSimulations.gravityComp.SimpleLegRobotDefinition.hipRollBodyName;
import static us.ihmc.exampleSimulations.gravityComp.SimpleLegRobotDefinition.hipRollJointName;
import static us.ihmc.exampleSimulations.gravityComp.SimpleLegRobotDefinition.kneePitchJointName;
import static us.ihmc.exampleSimulations.gravityComp.SimpleLegRobotDefinition.pelvisBodyName;
import static us.ihmc.exampleSimulations.gravityComp.SimpleLegRobotDefinition.rootJointName;
import static us.ihmc.exampleSimulations.gravityComp.SimpleLegRobotDefinition.shinBodyName;
import static us.ihmc.exampleSimulations.gravityComp.SimpleLegRobotDefinition.thighBodyName;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.mecano.frames.FixedMovingReferenceFrame;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.SixDoFJointBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.Robot;

public class SimpleTwoLeggedNoRollRobot
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final RigidBodyBasics rootBody;
   private final RigidBodyBasics pelvis;
   private final SideDependentList<RigidBodyBasics> thigh = new SideDependentList<>();
   private final SideDependentList<RigidBodyBasics> shin = new SideDependentList<>();
   private final SideDependentList<RigidBodyBasics> foot = new SideDependentList<>();

   private final SixDoFJointBasics rootJoint;
   private final SideDependentList<OneDoFJointBasics> hipPitchJoint = new SideDependentList<>();
   private final SideDependentList<OneDoFJointBasics> kneePitchJoint = new SideDependentList<>();
   private final SideDependentList<OneDoFJointBasics> anklePitchJoint = new SideDependentList<>();
   private final List<OneDoFJointBasics> oneDoFJoints = new ArrayList();

   private final FloatingJoint scsRootJoint;
   private final SideDependentList<OneDegreeOfFreedomJoint> scsHipPitchJoint = new SideDependentList<>();
   private final SideDependentList<OneDegreeOfFreedomJoint> scsKneePitchJoint = new SideDependentList<>();
   private final SideDependentList<OneDegreeOfFreedomJoint> scsAnklePitchJoint = new SideDependentList<>();

   private final SideDependentList<MovingReferenceFrame> soleFrame = new SideDependentList<>();

   public SimpleTwoLeggedNoRollRobot(SimpleTwoLeggedNoRollRobotDefinition definition, Robot simpleLegSCSRobot)
   {
      rootBody = definition.newInstance(worldFrame);
      pelvis = MultiBodySystemTools.findRigidBody(rootBody, pelvisBodyName);
      rootJoint = (SixDoFJointBasics) MultiBodySystemTools.findJoint(rootBody, rootJointName);
      scsRootJoint = (FloatingJoint) simpleLegSCSRobot.getJoint(rootJointName);

      for (RobotSide robotSide : RobotSide.values)
      {
         thigh.put(robotSide, MultiBodySystemTools.findRigidBody(rootBody, robotSide.getShortLowerCaseName() + "_" + thighBodyName));
         shin.put(robotSide, MultiBodySystemTools.findRigidBody(rootBody, robotSide.getShortLowerCaseName() + "_" + shinBodyName));
         foot.put(robotSide, MultiBodySystemTools.findRigidBody(rootBody, robotSide.getShortLowerCaseName() + "_" + footBodyName));

         hipPitchJoint.put(robotSide,
                           (OneDoFJointBasics) MultiBodySystemTools.findJoint(rootBody, robotSide.getShortLowerCaseName() + "_" + hipPitchJointName));
         kneePitchJoint.put(robotSide,
                            (OneDoFJointBasics) MultiBodySystemTools.findJoint(rootBody, robotSide.getShortLowerCaseName() + "_" + kneePitchJointName));
         anklePitchJoint.put(robotSide,
                             (OneDoFJointBasics) MultiBodySystemTools.findJoint(rootBody, robotSide.getShortLowerCaseName() + "_" + anklePitchJointName));

         oneDoFJoints.addAll(Arrays.asList(hipPitchJoint.get(robotSide), kneePitchJoint.get(robotSide), anklePitchJoint.get(robotSide)));

         scsHipPitchJoint.put(robotSide, (OneDegreeOfFreedomJoint) simpleLegSCSRobot.getJoint(robotSide.getShortLowerCaseName() + "_" + hipPitchJointName));
         scsKneePitchJoint.put(robotSide, (OneDegreeOfFreedomJoint) simpleLegSCSRobot.getJoint(robotSide.getShortLowerCaseName() + "_" + kneePitchJointName));
         scsAnklePitchJoint.put(robotSide, (OneDegreeOfFreedomJoint) simpleLegSCSRobot.getJoint(robotSide.getShortLowerCaseName() + "_" + anklePitchJointName));

         soleFrame.put(robotSide,
                       new FixedMovingReferenceFrame("soleFrame",
                                                     anklePitchJoint.get(robotSide).getFrameAfterJoint(),
                                                     new Vector3D(0.0, 0.0, -footSize.getZ())));
      }
   }

   public void readSimulationSensorData()
   {
      rootJoint.getJointPose().set(scsRootJoint.getPosition(), scsRootJoint.getOrientation());
      rootJoint.getJointTwist().getAngularPart().set(scsRootJoint.getAngularVelocity());
      rootJoint.getJointTwist().getLinearPart().setMatchingFrame(worldFrame, scsRootJoint.getLinearVelocity());

      for (RobotSide robotSide : RobotSide.values)
      {

         hipPitchJoint.get(robotSide).setQ(scsHipPitchJoint.get(robotSide).getQ());
         hipPitchJoint.get(robotSide).setQd(scsHipPitchJoint.get(robotSide).getQD());

         kneePitchJoint.get(robotSide).setQ(scsKneePitchJoint.get(robotSide).getQ());
         kneePitchJoint.get(robotSide).setQd(scsKneePitchJoint.get(robotSide).getQD());

         anklePitchJoint.get(robotSide).setQ(scsAnklePitchJoint.get(robotSide).getQ());
         anklePitchJoint.get(robotSide).setQd(scsAnklePitchJoint.get(robotSide).getQD());

      }
      rootBody.updateFramesRecursively();
   }

   public void writeControllerOutput()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         scsHipPitchJoint.get(robotSide).setTau(hipPitchJoint.get(robotSide).getTau());
         scsKneePitchJoint.get(robotSide).setTau(kneePitchJoint.get(robotSide).getTau());
         scsAnklePitchJoint.get(robotSide).setTau(anklePitchJoint.get(robotSide).getTau());
      }
   }

   public RigidBodyBasics getRootBody()
   {
      return rootBody;
   }

   public RigidBodyBasics getPelvis()
   {
      return pelvis;
   }


   public RigidBodyBasics getThigh(RobotSide robotSide)
   {
      return thigh.get(robotSide);
   }

   public RigidBodyBasics getShin(RobotSide robotSide)
   {
      return shin.get(robotSide);
   }


   public RigidBodyBasics getFoot(RobotSide robotSide)
   {
      return foot.get(robotSide);
   }

   public SixDoFJointBasics getRootJoint()
   {
      return rootJoint;
   }


   public OneDoFJointBasics getHipPitchJoint(RobotSide robotSide)
   {
      return hipPitchJoint.get(robotSide);
   }

   public OneDoFJointBasics getKneePitchJoint(RobotSide robotSide)
   {
      return kneePitchJoint.get(robotSide);
   }

   public OneDoFJointBasics getAnklePitchJoint(RobotSide robotSide)
   {
      return anklePitchJoint.get(robotSide);
   }



   public List<OneDoFJointBasics> getOneDoFJoints()
   {
      return oneDoFJoints;
   }

   public MovingReferenceFrame getSoleFrame(RobotSide robotSide)
   {
      return soleFrame.get(robotSide);
   }
}
