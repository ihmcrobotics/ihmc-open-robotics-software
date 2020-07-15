package us.ihmc.simulationToolkit.physicsEngine;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyReadOnly;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Robot;

public class SCSRobotTransformUpdater
{
   private final List<SingleRobotUpdater> robotUpdaters = new ArrayList<>();

   public SCSRobotTransformUpdater()
   {
   }

   public void addRobot(RigidBodyReadOnly rootBody, Robot scsRobot)
   {
      robotUpdaters.add(new SingleRobotUpdater(rootBody, scsRobot));
   }

   public void update()
   {
      for (SingleRobotUpdater robotUpdater : robotUpdaters)
      {
         robotUpdater.update();
      }
   }

   private static class SingleRobotUpdater
   {
      private final List<Joint> scsJoints = new ArrayList<>();
      private final List<ReferenceFrame> frameAfterJoints = new ArrayList<>();

      public SingleRobotUpdater(RigidBodyReadOnly rootBody, Robot scsRobot)
      {

         for (JointReadOnly joint : rootBody.childrenSubtreeIterable())
         {
            scsJoints.add(scsRobot.getJoint(joint.getName()));
            frameAfterJoints.add(joint.getFrameAfterJoint());
         }
      }

      public void update()
      {
         for (int i = 0; i < scsJoints.size(); i++)
         {
            Joint joint = scsJoints.get(i);
            ReferenceFrame frameAfterJoint = frameAfterJoints.get(i);
            joint.jointTransform3D.set(frameAfterJoint.getTransformToParent());
            joint.transformToNext.set(frameAfterJoint.getTransformToRoot());
         }
      }

   }
}
