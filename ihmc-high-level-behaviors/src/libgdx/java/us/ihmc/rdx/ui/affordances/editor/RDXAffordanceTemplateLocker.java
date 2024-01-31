package us.ihmc.rdx.ui.affordances.editor;

import imgui.ImGui;
import imgui.flag.ImGuiCol;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

/**
 * The "Lock Hand To Object" and "Lock Both Hands To Object" functionality allows you to lock the hands to the object and translate and rotate the object with
 * the hands attached to it.
 * It works only for post grasp frames. This means you need to first click on a frame button in the Post Grasp Menu or add one frame if there are none.
 * While moving the object and attached hands, you can then "Set" or "Add" frames whenever you wish.
 */
public class RDXAffordanceTemplateLocker
{
   private final SideDependentList<FramePose3D> handPoses;
   private final SideDependentList<RigidBodyTransform> handTransformsToWorld;
   private final RDXAffordanceTemplateEditorStatus editorStatus;

   private final SideDependentList<Boolean> affordancePoseLocked = new SideDependentList<>();
   private final SideDependentList<Boolean> handsLocked = new SideDependentList<>();
   private final SideDependentList<PoseReferenceFrame> handLockedFrames = new SideDependentList<>();
   private boolean bothHandsLocked = false;

   private final PoseReferenceFrame objectFrame = new PoseReferenceFrame("objectFrame", ReferenceFrame.getWorldFrame());

   public RDXAffordanceTemplateLocker(SideDependentList<RigidBodyTransform> handTransformsToWorld,
                                      SideDependentList<FramePose3D> handPoses,
                                      RDXAffordanceTemplateEditorStatus editorStatus)
   {
      this.handPoses = handPoses;
      this.handTransformsToWorld = handTransformsToWorld;
      this.editorStatus = editorStatus;

      for (RobotSide side : RobotSide.values)
      {
         handsLocked.put(side, false);
         affordancePoseLocked.put(side, false);
      }
   }

   public void update(FramePose3DReadOnly objectPose)
   {
      for (RobotSide side : handPoses.keySet())
      {
         if (editorStatus.getActiveMenu() == RDXActiveAffordanceMenu.POST_GRASP && handsLocked.get(side))
         {
            // used to update the hand pose according to object pose in post-grasping once fixed contact with object
            if (!affordancePoseLocked.get(side))
            {
               objectFrame.setPoseAndUpdate(objectPose);
               handLockedFrames.put(side, new PoseReferenceFrame(side.getLowerCaseName() + "HandFrame", objectFrame));
               handPoses.get(side).changeFrame(objectFrame);
               handLockedFrames.get(side).setPoseAndUpdate(handPoses.get(side));
               handPoses.get(side).changeFrame(ReferenceFrame.getWorldFrame());
               affordancePoseLocked.replace(side, true);
            }
            objectFrame.setPoseAndUpdate(objectPose);
            FramePose3D pose = new FramePose3D(handLockedFrames.get(side));
            pose.changeFrame(ReferenceFrame.getWorldFrame());
            handTransformsToWorld.get(side).set(pose.getOrientation(), pose.getTranslation());
         }
         else
         {
            handsLocked.replace(side, false);
            affordancePoseLocked.replace(side, false);
         }
      }

      if (handsLocked.get(RobotSide.RIGHT) && handsLocked.get(RobotSide.LEFT))
         bothHandsLocked = true;
      else
         bothHandsLocked = false;
   }

   public void renderImGuiWidgets(ImGuiUniqueLabelMap labels)
   {
      boolean changedColorLockOneHand = false;
      if (handPoses.containsKey(editorStatus.getActiveSide()))
      {
         if (handsLocked.get(editorStatus.getActiveSide()))
         {
            ImGui.pushStyleColor(ImGuiCol.Button, 0.0f, 0.0f, 1.0f, 0.5f);
            changedColorLockOneHand = true;
         }
         if (!(handsLocked.get(RobotSide.LEFT) && handsLocked.get(RobotSide.RIGHT)))
         {
            if (ImGui.button(labels.get("Lock Hand To Object")) && editorStatus.getActiveMenu() == RDXActiveAffordanceMenu.POST_GRASP)
               handsLocked.replace(editorStatus.getActiveSide(), !handsLocked.get(editorStatus.getActiveSide()));
         }
         if (changedColorLockOneHand)
            ImGui.popStyleColor();

         boolean changedColorLockBothHands = false;
         if ((handsLocked.get(RobotSide.LEFT) && handsLocked.get(RobotSide.RIGHT)))
         {
            ImGui.pushStyleColor(ImGuiCol.Button, 0.0f, 0.0f, 1.0f, 0.5f);
            changedColorLockBothHands = true;
         }
         if (!((handsLocked.get(RobotSide.LEFT) && !handsLocked.get(RobotSide.RIGHT)) || (!handsLocked.get(RobotSide.LEFT)
                                                                                          && handsLocked.get(RobotSide.RIGHT))))
         { // not in alternate state, this means single hand lock is not activate
            if (ImGui.button(labels.get("Lock Both Hands To Object")) && editorStatus.getActiveMenu() == RDXActiveAffordanceMenu.POST_GRASP)
            {
               for (RobotSide side : RobotSide.values)
                  handsLocked.replace(side, !handsLocked.get(side));
            }
            if (changedColorLockBothHands)
               ImGui.popStyleColor();
         }
      }
   }

   public boolean areBothHandsLocked()
   {
      return bothHandsLocked;
   }
}
