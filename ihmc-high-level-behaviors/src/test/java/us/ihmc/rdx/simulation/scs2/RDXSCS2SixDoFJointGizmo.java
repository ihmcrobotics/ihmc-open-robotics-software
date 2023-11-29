package us.ihmc.rdx.simulation.scs2;

import imgui.ImGui;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.gizmo.RDXSelectablePose3DGizmo;
import us.ihmc.scs2.sharedMemory.LinkedYoDouble;
import us.ihmc.scs2.sharedMemory.interfaces.LinkedYoVariableFactory;
import us.ihmc.scs2.simulation.robot.multiBodySystem.SimSixDoFJoint;
import us.ihmc.yoVariables.registry.YoRegistry;

public class RDXSCS2SixDoFJointGizmo
{
   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final SimSixDoFJoint rootJoint;
   private final RDXSelectablePose3DGizmo gizmo;
   private final String checkboxLabel;
   private LinkedYoDouble x;
   private LinkedYoDouble y;
   private LinkedYoDouble z;
   private LinkedYoDouble qx;
   private LinkedYoDouble qy;
   private LinkedYoDouble qz;
   private LinkedYoDouble qs;

   public RDXSCS2SixDoFJointGizmo(RDX3DPanel panel3D, SimSixDoFJoint rootJoint)
   {
      this.rootJoint = rootJoint;

      gizmo = new RDXSelectablePose3DGizmo();
      gizmo.createAndSetupDefault(panel3D);
      checkboxLabel = labels.get("Move " + rootJoint.getName());

   }

   public void linkVariables(YoRegistry rootRegistry, LinkedYoVariableFactory linkedYoVariableFactory)
   {
      x = (LinkedYoDouble) linkedYoVariableFactory.newLinkedYoVariable(rootRegistry.findVariable(rootJoint.getJointPose().getYoX().getFullNameString()));
      y = (LinkedYoDouble) linkedYoVariableFactory.newLinkedYoVariable(rootRegistry.findVariable(rootJoint.getJointPose().getYoY().getFullNameString()));
      z = (LinkedYoDouble) linkedYoVariableFactory.newLinkedYoVariable(rootRegistry.findVariable(rootJoint.getJointPose().getYoZ().getFullNameString()));
      qx = (LinkedYoDouble) linkedYoVariableFactory.newLinkedYoVariable(rootRegistry.findVariable(rootJoint.getJointPose().getYoQx().getFullNameString()));
      qy = (LinkedYoDouble) linkedYoVariableFactory.newLinkedYoVariable(rootRegistry.findVariable(rootJoint.getJointPose().getYoQy().getFullNameString()));
      qz = (LinkedYoDouble) linkedYoVariableFactory.newLinkedYoVariable(rootRegistry.findVariable(rootJoint.getJointPose().getYoQz().getFullNameString()));
      qs = (LinkedYoDouble) linkedYoVariableFactory.newLinkedYoVariable(rootRegistry.findVariable(rootJoint.getJointPose().getYoQs().getFullNameString()));
   }

   public void update()
   {
      if (x != null)
      {
         x.pull();
         y.pull();
         z.pull();
         qx.pull();
         qy.pull();
         qz.pull();
         qs.pull();

         if (gizmo.isSelected())
         {
            x.getLinkedYoVariable().set(gizmo.getPoseGizmo().getPose().getTranslation().getX());
            y.getLinkedYoVariable().set(gizmo.getPoseGizmo().getPose().getTranslation().getY());
            z.getLinkedYoVariable().set(gizmo.getPoseGizmo().getPose().getTranslation().getZ());
            qx.getLinkedYoVariable().set(gizmo.getPoseGizmo().getPose().getRotation().getX());
            qy.getLinkedYoVariable().set(gizmo.getPoseGizmo().getPose().getRotation().getY());
            qz.getLinkedYoVariable().set(gizmo.getPoseGizmo().getPose().getRotation().getZ());
            qs.getLinkedYoVariable().set(gizmo.getPoseGizmo().getPose().getRotation().getS());
         }
         else
         {
            gizmo.getPoseGizmo().getTransformToParent().getTranslation().setX(x.getLinkedYoVariable().getValue());
            gizmo.getPoseGizmo().getTransformToParent().getTranslation().setY(y.getLinkedYoVariable().getValue());
            gizmo.getPoseGizmo().getTransformToParent().getTranslation().setZ(z.getLinkedYoVariable().getValue());
            gizmo.getPoseGizmo().getTransformToParent().getRotation().setQuaternion(qx.getLinkedYoVariable().getValue(),
                                                                                    qy.getLinkedYoVariable().getValue(),
                                                                                    qz.getLinkedYoVariable().getValue(),
                                                                                    qs.getLinkedYoVariable().getValue());
         }

         x.push();
         y.push();
         z.push();
         qx.push();
         qy.push();
         qz.push();
         qs.push();
      }
   }

   public void renderMoveJointCheckbox()
   {
      ImGui.checkbox(checkboxLabel, gizmo.getSelected());
   }
}
