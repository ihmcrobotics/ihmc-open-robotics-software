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

public class SimpleTwoLeggedRobot
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final RigidBodyBasics rootBody;
   private final RigidBodyBasics pelvis;
   private final SideDependentList<RigidBodyBasics> hipRollBody = new SideDependentList<>();
   private final SideDependentList<RigidBodyBasics> thigh = new SideDependentList<>();
   private final SideDependentList<RigidBodyBasics> shin = new SideDependentList<>();
   private final SideDependentList<RigidBodyBasics> anklePitchBody = new SideDependentList<>();
   private final SideDependentList<RigidBodyBasics> foot = new SideDependentList<>();

   private final SixDoFJointBasics rootJoint;
   private final SideDependentList<OneDoFJointBasics> hipRollJoint = new SideDependentList<>();
   private final SideDependentList<OneDoFJointBasics> hipPitchJoint = new SideDependentList<>();
   private final SideDependentList<OneDoFJointBasics> kneePitchJoint = new SideDependentList<>();
   private final SideDependentList<OneDoFJointBasics> anklePitchJoint = new SideDependentList<>();
   private final SideDependentList<OneDoFJointBasics> ankleRollJoint = new SideDependentList<>();
   private final List<OneDoFJointBasics> oneDoFJoints = new ArrayList();

   private final FloatingJoint scsRootJoint;
   private final SideDependentList<OneDegreeOfFreedomJoint> scsHipRollJoint = new SideDependentList<>();
   private final SideDependentList<OneDegreeOfFreedomJoint> scsHipPitchJoint = new SideDependentList<>();
   private final SideDependentList<OneDegreeOfFreedomJoint> scsKneePitchJoint = new SideDependentList<>();
   private final SideDependentList<OneDegreeOfFreedomJoint> scsAnklePitchJoint = new SideDependentList<>();
   private final SideDependentList<OneDegreeOfFreedomJoint> scsAnkleRollJoint = new SideDependentList<>();

   private final SideDependentList<MovingReferenceFrame> soleFrame = new SideDependentList<>();

   public SimpleTwoLeggedRobot(SimpleTwoLeggedRobotDefinition definition, Robot simpleLegSCSRobot)
   {
      rootBody = definition.newInstance(worldFrame);
      pelvis = MultiBodySystemTools.findRigidBody(rootBody, pelvisBodyName);
      rootJoint = (SixDoFJointBasics) MultiBodySystemTools.findJoint(rootBody, rootJointName);
      scsRootJoint = (FloatingJoint) simpleLegSCSRobot.getJoint(rootJointName);

      for (RobotSide robotSide : RobotSide.values)
      {
         hipRollBody.put(robotSide, MultiBodySystemTools.findRigidBody(rootBody, robotSide.getShortLowerCaseName() + "_" + hipRollBodyName));
         thigh.put(robotSide, MultiBodySystemTools.findRigidBody(rootBody, robotSide.getShortLowerCaseName() + "_" + thighBodyName));
         shin.put(robotSide, MultiBodySystemTools.findRigidBody(rootBody, robotSide.getShortLowerCaseName() + "_" + shinBodyName));
         anklePitchBody.put(robotSide, MultiBodySystemTools.findRigidBody(rootBody, robotSide.getShortLowerCaseName() + "_" + anklePitchBodyName));
         foot.put(robotSide, MultiBodySystemTools.findRigidBody(rootBody, robotSide.getShortLowerCaseName() + "_" + footBodyName));

         hipRollJoint.put(robotSide, (OneDoFJointBasics) MultiBodySystemTools.findJoint(rootBody, robotSide.getShortLowerCaseName() + "_" + hipRollJointName));
         hipPitchJoint.put(robotSide,
                           (OneDoFJointBasics) MultiBodySystemTools.findJoint(rootBody, robotSide.getShortLowerCaseName() + "_" + hipPitchJointName));
         kneePitchJoint.put(robotSide,
                            (OneDoFJointBasics) MultiBodySystemTools.findJoint(rootBody, robotSide.getShortLowerCaseName() + "_" + kneePitchJointName));
         anklePitchJoint.put(robotSide,
                             (OneDoFJointBasics) MultiBodySystemTools.findJoint(rootBody, robotSide.getShortLowerCaseName() + "_" + anklePitchJointName));
         ankleRollJoint.put(robotSide,
                            (OneDoFJointBasics) MultiBodySystemTools.findJoint(rootBody, robotSide.getShortLowerCaseName() + "_" + ankleRollJointName));

         oneDoFJoints.addAll(Arrays.asList(hipRollJoint.get(robotSide),
                                           hipPitchJoint.get(robotSide),
                                           kneePitchJoint.get(robotSide),
                                           anklePitchJoint.get(robotSide),
                                           ankleRollJoint.get(robotSide)));

         scsHipRollJoint.put(robotSide, (OneDegreeOfFreedomJoint) simpleLegSCSRobot.getJoint(robotSide.getShortLowerCaseName() + "_" + hipRollJointName));
         scsHipPitchJoint.put(robotSide, (OneDegreeOfFreedomJoint) simpleLegSCSRobot.getJoint(robotSide.getShortLowerCaseName() + "_" + hipPitchJointName));
         scsKneePitchJoint.put(robotSide, (OneDegreeOfFreedomJoint) simpleLegSCSRobot.getJoint(robotSide.getShortLowerCaseName() + "_" + kneePitchJointName));
         scsAnklePitchJoint.put(robotSide, (OneDegreeOfFreedomJoint) simpleLegSCSRobot.getJoint(robotSide.getShortLowerCaseName() + "_" + anklePitchJointName));
         scsAnkleRollJoint.put(robotSide, (OneDegreeOfFreedomJoint) simpleLegSCSRobot.getJoint(robotSide.getShortLowerCaseName() + "_" + ankleRollJointName));

         soleFrame.put(robotSide,
                       new FixedMovingReferenceFrame("soleFrame",
                                                     ankleRollJoint.get(robotSide).getFrameAfterJoint(),
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
         hipRollJoint.get(robotSide).setQ(scsHipRollJoint.get(robotSide).getQ());
         hipRollJoint.get(robotSide).setQd(scsHipRollJoint.get(robotSide).getQD());

         hipPitchJoint.get(robotSide).setQ(scsHipPitchJoint.get(robotSide).getQ());
         hipPitchJoint.get(robotSide).setQd(scsHipPitchJoint.get(robotSide).getQD());

         kneePitchJoint.get(robotSide).setQ(scsKneePitchJoint.get(robotSide).getQ());
         kneePitchJoint.get(robotSide).setQd(scsKneePitchJoint.get(robotSide).getQD());

         anklePitchJoint.get(robotSide).setQ(scsAnklePitchJoint.get(robotSide).getQ());
         anklePitchJoint.get(robotSide).setQd(scsAnklePitchJoint.get(robotSide).getQD());

         ankleRollJoint.get(robotSide).setQ(scsAnkleRollJoint.get(robotSide).getQ());
         ankleRollJoint.get(robotSide).setQd(scsAnkleRollJoint.get(robotSide).getQD());

         
      }
      rootBody.updateFramesRecursively();
   }

   public void writeControllerOutput()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         scsHipRollJoint.get(robotSide).setTau(hipRollJoint.get(robotSide).getTau());
         scsHipPitchJoint.get(robotSide).setTau(hipPitchJoint.get(robotSide).getTau());
         scsKneePitchJoint.get(robotSide).setTau(kneePitchJoint.get(robotSide).getTau());
         scsAnklePitchJoint.get(robotSide).setTau(anklePitchJoint.get(robotSide).getTau());
         scsAnkleRollJoint.get(robotSide).setTau(ankleRollJoint.get(robotSide).getTau());
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

   public RigidBodyBasics getHipRollBody(RobotSide robotSide)
   {
      return hipRollBody.get(robotSide);
   }

   public RigidBodyBasics getThigh(RobotSide robotSide)
   {
      return thigh.get(robotSide);
   }

   public RigidBodyBasics getShin(RobotSide robotSide)
   {
      return shin.get(robotSide);
   }

   public RigidBodyBasics getAnklePitchBody(RobotSide robotSide)
   {
      return anklePitchBody.get(robotSide);
   }

   public RigidBodyBasics getFoot(RobotSide robotSide)
   {
      return foot.get(robotSide);
   }

   public SixDoFJointBasics getRootJoint()
   {
      return rootJoint;
   }

   public OneDoFJointBasics getHipRollJoint(RobotSide robotSide)
   {
      return hipRollJoint.get(robotSide);
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

   public OneDoFJointBasics getAnkleRollJoint(RobotSide robotSide)
   {
      return ankleRollJoint.get(robotSide);
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
