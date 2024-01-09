package us.ihmc.rdx.simulation.scs2;

import imgui.ImGui;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.gizmo.RDXSelectablePose3DGizmo;
import us.ihmc.scs2.simulation.bullet.physicsEngine.BulletRobot;

/**
 * Allows a user to move robots around with gizmos in an SCS 2 session.
 * This can be used to reset the state of the environment when doing
 * mobile manipulation experiments.
 */
public class RDXSCS2BulletRobotMover
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final BulletRobot bulletRobot;
   private final RDXSelectablePose3DGizmo gizmo;
   private final String checkboxLabel;
   private final Pose3D poseBarrier = new Pose3D();

   public RDXSCS2BulletRobotMover(RDX3DPanel panel3D, BulletRobot bulletRobot)
   {
      this.bulletRobot = bulletRobot;

      gizmo = new RDXSelectablePose3DGizmo();
      gizmo.createAndSetupDefault(panel3D);
      checkboxLabel = labels.get("Move " + bulletRobot.getName());
   }

   public void update()
   {
      synchronized (poseBarrier)
      {
         if (gizmo.isSelected())
         {
            poseBarrier.set(gizmo.getPoseGizmo().getTransformToParent());
         }
         else
         {
            gizmo.getPoseGizmo().getTransformToParent().set(poseBarrier);
         }
      }
   }

   public void beforePhysics(double time)
   {
      synchronized (poseBarrier)
      {
         if (gizmo.isSelected())
         {
            bulletRobot.setIsKinematicObject(true);

            bulletRobot.getFloatingRootJoint().getJointPose().set(poseBarrier);
            bulletRobot.getFloatingRootJoint().updateFramesRecursively();
         }
         else
         {
            bulletRobot.setIsKinematicObject(false);

            poseBarrier.set(bulletRobot.getFloatingRootJoint().getJointPose());
         }
      }
   }

   public void renderMoveJointCheckbox()
   {
      ImGui.checkbox(checkboxLabel, gizmo.getSelected());
   }
}
