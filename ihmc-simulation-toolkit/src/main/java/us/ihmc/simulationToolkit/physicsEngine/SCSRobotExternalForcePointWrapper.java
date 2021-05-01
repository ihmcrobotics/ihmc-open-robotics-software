package us.ihmc.simulationToolkit.physicsEngine;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.FixedFrameSpatialVectorBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.physics.ExternalWrenchProvider;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector3D;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Function;
import java.util.stream.Stream;

public class SCSRobotExternalForcePointWrapper implements ExternalWrenchProvider
{
   private final Map<JointReadOnly, List<ExternalForcePoint>> externalForcePointMap = new HashMap<>();

   private final Wrench wrenchToApply = new Wrench();
   private final Pose3D externalForcePointPose = new Pose3D();
   private final PoseReferenceFrame scsExternalForcePointFrame = new PoseReferenceFrame("scsExternalForcePointFrame", ReferenceFrame.getWorldFrame());

   public void addRobot(RigidBodyReadOnly rootBody, Robot scsRobot)
   {
      JointReadOnly[] allJoints = MultiBodySystemTools.collectSubtreeJoints(rootBody);

      List<ExternalForcePoint> externalForcePoints = scsRobot.getAllExternalForcePoints();
      for (int i = 0; i < externalForcePoints.size(); i++)
      {
         ExternalForcePoint externalForcePoint = externalForcePoints.get(i);
         String jointName = externalForcePoint.getParentJoint().getName();
         JointReadOnly joint = Stream.of(allJoints).filter(candidate -> candidate.getName().equals(jointName)).findAny().get();

         externalForcePointMap.computeIfAbsent(joint, name -> new ArrayList<>()).add(externalForcePoint);
      }
   }

   @Override
   public void applyExternalWrenches(RigidBodyReadOnly rootBody, Function<RigidBodyReadOnly, FixedFrameSpatialVectorBasics> externalWrenches)
   {
      JointReadOnly[] joints = MultiBodySystemTools.collectSubtreeJoints(rootBody);

      for (int i = 0; i < joints.length; i++)
      {
         JointReadOnly joint = joints[i];
         RigidBodyReadOnly rigidBody = joint.getSuccessor();

         List<ExternalForcePoint> externalForcePoints = externalForcePointMap.get(joint);
         if (externalForcePoints == null)
            continue;

         for (int j = 0; j < externalForcePoints.size(); j++)
         {
            ExternalForcePoint externalForcePoint = externalForcePoints.get(j);
            YoFrameVector3D moment = externalForcePoint.getYoMoment();
            YoFrameVector3D force = externalForcePoint.getYoForce();

            externalForcePointPose.getOrientation().setToZero();
            externalForcePoint.getPosition(externalForcePointPose.getPosition());
            scsExternalForcePointFrame.setPoseAndUpdate(externalForcePointPose);

            FixedFrameSpatialVectorBasics externalWrench = externalWrenches.apply(rigidBody);
            wrenchToApply.setIncludingFrame(rigidBody.getBodyFixedFrame(), scsExternalForcePointFrame, moment, force);
            wrenchToApply.changeFrame(externalWrench.getReferenceFrame());
            externalWrench.add(wrenchToApply);
         }
      }
   }
}
