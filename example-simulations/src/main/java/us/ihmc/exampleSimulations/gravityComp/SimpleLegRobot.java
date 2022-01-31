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
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.Robot;

public class SimpleLegRobot
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final RigidBodyBasics rootBody;
   private final RigidBodyBasics pelvis;
   private final RigidBodyBasics hipRollBody;
   private final RigidBodyBasics thigh;
   private final RigidBodyBasics shin;
   private final RigidBodyBasics anklePitchBody;
   private final RigidBodyBasics foot;

   private final SixDoFJointBasics rootJoint;
   private final OneDoFJointBasics hipRollJoint;
   private final OneDoFJointBasics hipPitchJoint;
   private final OneDoFJointBasics kneePitchJoint;
   private final OneDoFJointBasics anklePitchJoint;
   private final OneDoFJointBasics ankleRollJoint;
   private final List<OneDoFJointBasics> oneDoFJoints;

   private final FloatingJoint scsRootJoint;
   private final OneDegreeOfFreedomJoint scsHipRollJoint;
   private final OneDegreeOfFreedomJoint scsHipPitchJoint;
   private final OneDegreeOfFreedomJoint scsKneePitchJoint;
   private final OneDegreeOfFreedomJoint scsAnklePitchJoint;
   private final OneDegreeOfFreedomJoint scsAnkleRollJoint;

   private final MovingReferenceFrame soleFrame;

   public SimpleLegRobot(SimpleLegRobotDefinition definition, Robot simpleLegSCSRobot)
   {
      rootBody = definition.newInstance(worldFrame);
      pelvis = MultiBodySystemTools.findRigidBody(rootBody, pelvisBodyName);
      hipRollBody = MultiBodySystemTools.findRigidBody(rootBody, hipRollBodyName);
      thigh = MultiBodySystemTools.findRigidBody(rootBody, thighBodyName);
      shin = MultiBodySystemTools.findRigidBody(rootBody, shinBodyName);
      anklePitchBody = MultiBodySystemTools.findRigidBody(rootBody, anklePitchBodyName);
      foot = MultiBodySystemTools.findRigidBody(rootBody, footBodyName);

      rootJoint = (SixDoFJointBasics) MultiBodySystemTools.findJoint(rootBody, rootJointName);
      hipRollJoint = (OneDoFJointBasics) MultiBodySystemTools.findJoint(rootBody, hipRollJointName);
      hipPitchJoint = (OneDoFJointBasics) MultiBodySystemTools.findJoint(rootBody, hipPitchJointName);
      kneePitchJoint = (OneDoFJointBasics) MultiBodySystemTools.findJoint(rootBody, kneePitchJointName);
      anklePitchJoint = (OneDoFJointBasics) MultiBodySystemTools.findJoint(rootBody, anklePitchJointName);
      ankleRollJoint = (OneDoFJointBasics) MultiBodySystemTools.findJoint(rootBody, hipRollJointName);
      oneDoFJoints = Arrays.asList(hipRollJoint, hipPitchJoint, kneePitchJoint, anklePitchJoint, ankleRollJoint);

      scsRootJoint = (FloatingJoint) simpleLegSCSRobot.getJoint(rootJointName);
      scsHipRollJoint = (OneDegreeOfFreedomJoint) simpleLegSCSRobot.getJoint(hipRollJointName);
      scsHipPitchJoint = (OneDegreeOfFreedomJoint) simpleLegSCSRobot.getJoint(hipPitchJointName);
      scsKneePitchJoint = (OneDegreeOfFreedomJoint) simpleLegSCSRobot.getJoint(kneePitchJointName);
      scsAnklePitchJoint = (OneDegreeOfFreedomJoint) simpleLegSCSRobot.getJoint(anklePitchJointName);
      scsAnkleRollJoint = (OneDegreeOfFreedomJoint) simpleLegSCSRobot.getJoint(ankleRollJointName);

      soleFrame = new FixedMovingReferenceFrame("soleFrame", ankleRollJoint.getFrameAfterJoint(), new Vector3D(0.0, 0.0, -footSize.getZ()));
   }

   public void readSimulationSensorData()
   {
      rootJoint.getJointPose().set(scsRootJoint.getPosition(), scsRootJoint.getOrientation());
      rootJoint.getJointTwist().getAngularPart().set(scsRootJoint.getAngularVelocity());
      rootJoint.getJointTwist().getLinearPart().setMatchingFrame(worldFrame, scsRootJoint.getLinearVelocity());

      hipRollJoint.setQ(scsHipRollJoint.getQ());
      hipRollJoint.setQd(scsHipRollJoint.getQD());

      hipPitchJoint.setQ(scsHipPitchJoint.getQ());
      hipPitchJoint.setQd(scsHipPitchJoint.getQD());

      kneePitchJoint.setQ(scsKneePitchJoint.getQ());
      kneePitchJoint.setQd(scsKneePitchJoint.getQD());

      anklePitchJoint.setQ(scsAnklePitchJoint.getQ());
      anklePitchJoint.setQd(scsAnklePitchJoint.getQD());

      ankleRollJoint.setQ(scsAnkleRollJoint.getQ());
      ankleRollJoint.setQd(scsAnkleRollJoint.getQD());

      rootBody.updateFramesRecursively();
   }

   public void writeControllerOutput()
   {
      scsHipRollJoint.setTau(hipRollJoint.getTau());
      scsHipPitchJoint.setTau(hipPitchJoint.getTau());
      scsKneePitchJoint.setTau(kneePitchJoint.getTau());
      scsAnklePitchJoint.setTau(anklePitchJoint.getTau());
      scsAnkleRollJoint.setTau(ankleRollJoint.getTau());
   }

   public RigidBodyBasics getRootBody()
   {
      return rootBody;
   }

   public RigidBodyBasics getPelvis()
   {
      return pelvis;
   }

   public RigidBodyBasics getHipRollBody()
   {
      return hipRollBody;
   }

   public RigidBodyBasics getThigh()
   {
      return thigh;
   }

   public RigidBodyBasics getShin()
   {
      return shin;
   }

   public RigidBodyBasics getAnklePitchBody()
   {
      return anklePitchBody;
   }

   public RigidBodyBasics getFoot()
   {
      return foot;
   }

   public SixDoFJointBasics getRootJoint()
   {
      return rootJoint;
   }

   public OneDoFJointBasics getHipRollJoint()
   {
      return hipRollJoint;
   }

   public OneDoFJointBasics getHipPitchJoint()
   {
      return hipPitchJoint;
   }

   public OneDoFJointBasics getKneePitchJoint()
   {
      return kneePitchJoint;
   }

   public OneDoFJointBasics getAnklePitchJoint()
   {
      return anklePitchJoint;
   }

   public OneDoFJointBasics getAnkleRollJoint()
   {
      return ankleRollJoint;
   }

   public List<OneDoFJointBasics> getOneDoFJoints()
   {
      return oneDoFJoints;
   }

   public MovingReferenceFrame getSoleFrame()
   {
      return soleFrame;
   }
}
