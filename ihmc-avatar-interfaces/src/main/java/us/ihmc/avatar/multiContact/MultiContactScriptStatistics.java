package us.ihmc.avatar.multiContact;

import org.apache.commons.lang3.tuple.Pair;
import toolbox_msgs.msg.dds.KinematicsToolboxRigidBodyMessage;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.WorkspacePathTools;

import javax.swing.*;
import javax.swing.filechooser.FileNameExtensionFilter;
import java.io.File;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

public class MultiContactScriptStatistics
{
   public static void main(String[] args)
   {
      Path currentDirectory = WorkspacePathTools.handleWorkingDirectoryFuzziness("ihmc-open-robotics-software")
                                                .resolve("valkyrie/src/main/resources/multiContact/scripts")
                                                .toAbsolutePath()
                                                .normalize();
      LogTools.info(currentDirectory);

      JFileChooser fileChooser = new JFileChooser(currentDirectory.toFile());
      fileChooser.setFileFilter(new FileNameExtensionFilter("JSON log", "json"));

      int chooserState = fileChooser.showOpenDialog(null);
      if (chooserState != JFileChooser.APPROVE_OPTION)
         return;

      File selectedFile = fileChooser.getSelectedFile();
      MultiContactScriptReader scriptReader = new MultiContactScriptReader();
      if (!scriptReader.loadScript(selectedFile))
         return;

      List<KinematicsToolboxSnapshotDescription> script = scriptReader.getAllItems();
      List<KFStats> kfStatList = new ArrayList<>();

      for (int i = 0; i < script.size(); i++)
      {
         KinematicsToolboxSnapshotDescription snapshot = script.get(i);
         List<SixDoFMotionControlAnchorDescription> contactAnchors = snapshot.getSixDoFAnchors().stream().filter(SixDoFMotionControlAnchorDescription::isContactState).toList();
         List<SixDoFMotionControlAnchorDescription> taskspaceAnchors = snapshot.getSixDoFAnchors().stream().filter(a -> !a.isContactState()).toList();
         List<OneDoFMotionControlAnchorDescription> oneDoFAnchors = snapshot.getOneDoFAnchors();
         CenterOfMassMotionControlAnchorDescription centerOfMassAnchor = snapshot.getCenterOfMassAnchor();

         int numContacts = contactAnchors.size();
         int numTaskspace = taskspaceAnchors.size();
         int numOneDoF = oneDoFAnchors.size();
         boolean hasCom = isCoMEnabled(centerOfMassAnchor);
         KFStats kfStats = new KFStats(numContacts, numTaskspace, numOneDoF, hasCom);

         if (i > 0)
         {
            KinematicsToolboxSnapshotDescription prevSnapshot = script.get(i - 1);
            List<SixDoFMotionControlAnchorDescription> prevContactAnchors = prevSnapshot.getSixDoFAnchors().stream().filter(SixDoFMotionControlAnchorDescription::isContactState).toList();
            List<SixDoFMotionControlAnchorDescription> prevTaskspaceAnchors = prevSnapshot.getSixDoFAnchors().stream().filter(a -> !a.isContactState()).toList();
            List<OneDoFMotionControlAnchorDescription> prevOneDoFAnchors = prevSnapshot.getOneDoFAnchors();
            CenterOfMassMotionControlAnchorDescription prevCenterOfMassAnchor = prevSnapshot.getCenterOfMassAnchor();

            Pair<Integer, Integer> contactsRemovedAndModified = amountSixDofRemovedAndModified(prevContactAnchors, contactAnchors, true);
            int contactsAdded = numContacts - (kfStatList.get(i-1).nContactAnchors - contactsRemovedAndModified.getLeft());
            kfStats.setContactData(contactsAdded, contactsRemovedAndModified.getLeft(), contactsRemovedAndModified.getRight());

            Pair<Integer, Integer> taskspaceRemovedAndModified = amountSixDofRemovedAndModified(prevTaskspaceAnchors, taskspaceAnchors, false);
            int taskspaceAdded = numTaskspace - (kfStatList.get(i-1).nTaskspaceAnchors - taskspaceRemovedAndModified.getLeft());
            kfStats.setTaskspaceData(taskspaceAdded, taskspaceRemovedAndModified.getLeft(), taskspaceRemovedAndModified.getRight());

            Pair<Integer, Integer> oneDoFRemovedAndModified = amountOneDoFRemovedAndModified(prevOneDoFAnchors, oneDoFAnchors);
            int oneDoFAdded = numOneDoF - (kfStatList.get(i-1).nOneDoF - oneDoFRemovedAndModified.getLeft());
            kfStats.setOneDoFData(oneDoFAdded, oneDoFRemovedAndModified.getLeft(), oneDoFRemovedAndModified.getRight());

            kfStats.setComModified(isCoMModified(prevCenterOfMassAnchor, centerOfMassAnchor));
         }

         kfStatList.add(kfStats);
      }

      int startingIndex = 1;

      // print tabular start
      System.out.print("\\begin{tabular}{|");
      for (int i = startingIndex; i < kfStatList.size() + 2; i++)
      {
         System.out.print(" c |");
      }
      System.out.println("}");
      System.out.println("\\hline");

      // Label keyframes
      System.out.print(" & Keyframe & ");
      for (int i = startingIndex; i < kfStatList.size(); i++)
      {
         String suffix = i == kfStatList.size() - 1 ? " \\\\\n " : " & ";
         System.out.print(i + suffix);
      }

      System.out.println("\\hline");
      System.out.println("\\hline");

      // Contact anchor data
      System.out.print("\\rotatebox[origin=c]{90}{Contact} & ");
      System.out.print("\\makecell{Total \\\\ Modified} & ");
      for (int i = startingIndex; i < kfStatList.size(); i++)
      {
         String suffix = i == kfStatList.size() - 1 ? " \\\\\n " : " & ";
         int nContactAnchors = kfStatList.get(i).nContactAnchors;
         String contactsModified = formatModified(kfStatList.get(i).contactModified);
         String addRemoved = formatAddedRemoved(kfStatList.get(i).contactsAdded, kfStatList.get(i).contactsRemoved);
         System.out.print("\\makecell{" + nContactAnchors + "\\\\ " + contactsModified + "} "+ suffix);
      }

      System.out.println("\\hline");

      // One dof
      System.out.print("\\rotatebox[origin=c]{90}{Taskspace} & ");
      System.out.print("\\makecell{Total \\\\ Modified} & ");
      for (int i = startingIndex; i < kfStatList.size(); i++)
      {
         String suffix = i == kfStatList.size() - 1 ? " \\\\\n " : " & ";
         int nTaskspaceAnchors = kfStatList.get(i).nTaskspaceAnchors;
         String taskspaceModified = formatModified(kfStatList.get(i).taskspaceModified);
         String addRemoved = formatAddedRemoved(kfStatList.get(i).taskspaceAdded, kfStatList.get(i).taskspaceRemoved);
         System.out.print("\\makecell{" + nTaskspaceAnchors + "\\\\ " + taskspaceModified + "} "+ suffix);
      }

      System.out.println("\\hline");

      // One dof
      System.out.print("\\rotatebox[origin=c]{90}{Joint} & ");
      System.out.print("\\makecell{Total \\\\ Modified} & ");
      for (int i = startingIndex; i < kfStatList.size(); i++)
      {
         String suffix = i == kfStatList.size() - 1 ? " \\\\\n " : " & ";
         int nOneDoFAnchors = kfStatList.get(i).nOneDoF;
         String oneDoFModified = formatModified(kfStatList.get(i).oneDoFModified);
         String addRemoved = formatAddedRemoved(kfStatList.get(i).oneDoFAdded, kfStatList.get(i).oneDoFRemoved);
         System.out.print("\\makecell{" + nOneDoFAnchors + "\\\\ " + oneDoFModified + "\\\\" + "} "+ suffix);
      }

      System.out.println("\\hline");

      // CoM
      System.out.print("\\rotatebox[origin=c]{90}{CoM} & ");
      System.out.print("\\makecell{Present \\\\ Modified } & ");
      for (int i = startingIndex; i < kfStatList.size(); i++)
      {
         String suffix = i == kfStatList.size() - 1 ? " \\\\\n " : " & ";
         boolean hasCoM = kfStatList.get(i).hasCoM;
         boolean comModified = kfStatList.get(i).comModified;
         String hasCoMStr = hasCoM ? "\\checkmark" : " - ";
         String comModifiedStr = comModified ? "\\checkmark" : " - ";
         System.out.print("\\makecell{" + hasCoMStr + "\\\\ " + comModifiedStr + "} "+ suffix);
      }

      System.out.println("\\hline");

      //      int numKfs = script.size();
//      int numSixDof = script.get(0).getSixDoFAnchors().size();
//      int numSixDofContact = 0;
//      int numSixDofNonContact = 0;
//
//      for (int i = 0; i < script.get(0).getSixDoFAnchors().size(); i++)
//      {
//         if (script.get(0).getSixDoFAnchors().get(i).isContactState())
//            numSixDofContact++;
//         else
//            numSixDofNonContact++;
//      }
//
//      int numOneDof = script.get(0).getOneDoFAnchors().size();
//      int numComAnchors = (script.get(0).getCenterOfMassAnchor() != null && isCoMEnabled(script.get(0).getCenterOfMassAnchor())) ? 1 : 0;
//      System.out.println("frame \t new \t old");
//
//      for (int i = 1; i < script.size(); i++)
//      {
//         KinematicsToolboxSnapshotDescription thisKf = script.get(i);
//         KinematicsToolboxSnapshotDescription prevKf = script.get(i - 1);
//
//         int newSixDofs = 0;
//         int oldSixDofs = 0;
//
//         for (SixDoFMotionControlAnchorDescription thisSixDof : thisKf.getSixDoFAnchors())
//         {
//            boolean foundMatch = false;
//            for (SixDoFMotionControlAnchorDescription prevSixDof : prevKf.getSixDoFAnchors())
//            {
//               if (areSixDoFAnchorsEqual(thisSixDof.getInputMessage(), prevSixDof.getInputMessage()))
//               {
//                  foundMatch = true;
//                  break;
//               }
//            }
//
//            if (!foundMatch)
//            {
//               numSixDof++;
//               newSixDofs++;
//
//               if (thisSixDof.isContactState())
//                  numSixDofContact++;
//               else
//                  numSixDofNonContact++;
//            }
//            else
//            {
//               oldSixDofs++;
//            }
//         }
//
//         System.out.println(i + "\t\t" + newSixDofs + "\t\t" + oldSixDofs);
//
//         for (OneDoFMotionControlAnchorDescription thisOneDof : thisKf.getOneDoFAnchors())
//         {
//            boolean foundMatch = false;
//            for (OneDoFMotionControlAnchorDescription prevOneDof : prevKf.getOneDoFAnchors())
//            {
//               if (thisOneDof.getInputMessage().epsilonEquals(prevOneDof.getInputMessage(), 1e-2))
//               {
//                  foundMatch = true;
//                  break;
//               }
//            }
//
//            if (!foundMatch)
//               numOneDof++;
//         }
//
//         CenterOfMassMotionControlAnchorDescription thisCoM = thisKf.getCenterOfMassAnchor();
//         CenterOfMassMotionControlAnchorDescription prevCoM = prevKf.getCenterOfMassAnchor();
//         if (thisCoM != null && isCoMEnabled(thisCoM))
//         {
//            if (prevCoM == null || !thisCoM.getInputMessage().epsilonEquals(prevCoM.getInputMessage(), 1e-2))
//            {
//               numComAnchors++;
//            }
//         }
//      }
//
//      System.out.println("num kfs   :" + numKfs);
//      System.out.println("num 6 dof :" + numSixDof);
//      System.out.println("num 1 dof :" + numOneDof);
//      System.out.println("num com   :" + numComAnchors);
//
//      System.out.println("latex: " + numKfs + " & " + numSixDofNonContact + " & " + numSixDofContact + " & " + numOneDof + " & " + numComAnchors);
   }

   private static Pair<Integer, Integer> amountSixDofRemovedAndModified(List<SixDoFMotionControlAnchorDescription> prev, List<SixDoFMotionControlAnchorDescription> next, boolean contactAnchor)
   {
      int removalCount = 0;
      int modifiedCount = 0;

      outerLoop:
      for (SixDoFMotionControlAnchorDescription prevAnchor : prev)
      {
         for (SixDoFMotionControlAnchorDescription nextAnchor : next)
         {
            if (areSameSixDoFAnchor(prevAnchor, nextAnchor, contactAnchor))
            {
               if (wasSixDoFAnchorModified(prevAnchor, nextAnchor, contactAnchor))
                  modifiedCount++;
               continue outerLoop;
            }
         }

         removalCount++;
      }

      return Pair.of(removalCount, modifiedCount);
   }

   private static Pair<Integer, Integer> amountOneDoFRemovedAndModified(List<OneDoFMotionControlAnchorDescription> prev, List<OneDoFMotionControlAnchorDescription> next)
   {
      int removalCount = 0;
      int modifiedCount = 0;

      outerLoop:
      for (OneDoFMotionControlAnchorDescription prevAnchor : prev)
      {
         for (OneDoFMotionControlAnchorDescription nextAnchor : next)
         {
            if (prevAnchor.getJointName().equals(nextAnchor.getJointName()))
            {
               if (wasOneDoFAnchorModified(prevAnchor, nextAnchor))
                  modifiedCount++;
               continue outerLoop;
            }
         }

         removalCount++;
      }

      return Pair.of(removalCount, modifiedCount);
   }

   private static boolean isCoMModified(CenterOfMassMotionControlAnchorDescription prev, CenterOfMassMotionControlAnchorDescription next)
   {
      if (prev == null || next == null)
         return false;
      return !prev.getInputMessage().epsilonEquals(next.getInputMessage(), 1e-3);
   }

   private static class KFStats
   {
      int nContactAnchors;
      int nTaskspaceAnchors;
      int nOneDoF;
      boolean hasCoM;

      int contactsAdded;
      int contactsRemoved;
      int taskspaceAdded;
      int taskspaceRemoved;
      int oneDoFAdded;
      int oneDoFRemoved;
      boolean comModified;

      int contactModified;
      int taskspaceModified;
      int oneDoFModified;

      public KFStats(int nContactAnchors, int nTaskspaceAnchors, int nJointAnchors, boolean hasCoM)
      {
         this.nContactAnchors = nContactAnchors;
         this.nTaskspaceAnchors = nTaskspaceAnchors;
         this.nOneDoF = nJointAnchors;
         this.hasCoM = hasCoM;
      }

      public void setContactData(int contactsAdded, int contactsRemoved, int contactModified)
      {
         this.contactsAdded = contactsAdded;
         this.contactsRemoved = contactsRemoved;
         this.contactModified = contactModified;
      }

      public void setTaskspaceData(int taskspaceAdded, int taskspaceRemoved, int taskspaceModified)
      {
         this.taskspaceAdded = taskspaceAdded;
         this.taskspaceRemoved = taskspaceRemoved;
         this.taskspaceModified = taskspaceModified;
      }

      public void setOneDoFData(int jointsAdded, int jointsRemoved, int oneDoFModified)
      {
         this.oneDoFAdded = jointsAdded;
         this.oneDoFRemoved = jointsRemoved;
         this.oneDoFModified = oneDoFModified;
      }

      public void setComModified(boolean comModified)
      {
         this.comModified = comModified;
      }
   }

   private static String formatModified(int modified)
   {
      return modified == 0 ? " - " : Integer.toString(modified);
   }

   private static String formatAddedRemoved(int added, int removed)
   {
      if (added == 0 && removed == 0)
         return " - ";
      else if (added == 0)
         return "-" + removed;
      else if (removed == 0)
         return "+" + added;
      else
         return "+" + added + ",-" + removed;
   }

   private static boolean areSameSixDoFAnchor(SixDoFMotionControlAnchorDescription anchorA, SixDoFMotionControlAnchorDescription anchorB, boolean contactAnchor)
   {
      if (!anchorA.getRigidBodyName().equals(anchorB.getRigidBodyName()))
         return false;
      if (!anchorA.getInputMessage().getControlFramePositionInEndEffector().epsilonEquals(anchorB.getInputMessage().getControlFramePositionInEndEffector(), 1e-4))
         return false;
      if (!contactAnchor && !anchorA.getInputMessage().getControlFrameOrientationInEndEffector().epsilonEquals(anchorB.getInputMessage().getControlFrameOrientationInEndEffector(), 1e-4))
         return false;
      return true;
   }

   private static boolean wasSixDoFAnchorModified(SixDoFMotionControlAnchorDescription anchorA, SixDoFMotionControlAnchorDescription anchorB, boolean contactAnchor)
   {
      KinematicsToolboxRigidBodyMessage messageA = anchorA.getInputMessage();
      KinematicsToolboxRigidBodyMessage messageB = anchorB.getInputMessage();

      if (!messageA.getLinearSelectionMatrix().epsilonEquals(messageB.getLinearSelectionMatrix(), 0.0))
         return true;
      if (!messageA.getAngularSelectionMatrix().epsilonEquals(messageB.getAngularSelectionMatrix(), 0.0))
         return true;

      if (!messageA.getLinearWeightMatrix().epsilonEquals(messageB.getLinearWeightMatrix(), 1e-5))
         return true;
      if (!messageA.getLinearWeightMatrix().epsilonEquals(messageB.getLinearWeightMatrix(), 1e-5))
         return true;

      if (messageA.getLinearSelectionMatrix().getXSelected() && !EuclidCoreTools.epsilonEquals(messageA.getDesiredPositionInWorld().getX(), messageB.getDesiredPositionInWorld().getX(), 1e-3))
         return true;
      if (messageA.getLinearSelectionMatrix().getYSelected() && !EuclidCoreTools.epsilonEquals(messageA.getDesiredPositionInWorld().getY(), messageB.getDesiredPositionInWorld().getY(), 1e-3))
         return true;
      if (messageA.getLinearSelectionMatrix().getZSelected() && !EuclidCoreTools.epsilonEquals(messageA.getDesiredPositionInWorld().getZ(), messageB.getDesiredPositionInWorld().getZ(), 1e-3))
         return true;

      if (contactAnchor)
         return false;

      Vector3D rotationA = new Vector3D();
      Vector3D rotationB = new Vector3D();
      messageA.getDesiredOrientationInWorld().getRotationVector(rotationA);
      messageB.getDesiredOrientationInWorld().getRotationVector(rotationB);

      if (messageA.getAngularSelectionMatrix().getXSelected() && !EuclidCoreTools.epsilonEquals(rotationA.getX(), rotationB.getX(), 1e-3))
         return true;
      if (messageA.getAngularSelectionMatrix().getYSelected() && !EuclidCoreTools.epsilonEquals(rotationA.getY(), rotationB.getY(), 1e-3))
         return true;
      if (messageA.getAngularSelectionMatrix().getZSelected() && !EuclidCoreTools.epsilonEquals(rotationA.getZ(), rotationB.getZ(), 1e-3))
         return true;

      return false;
   }

   private static boolean wasOneDoFAnchorModified(OneDoFMotionControlAnchorDescription anchorA, OneDoFMotionControlAnchorDescription anchorB)
   {
      if (!EuclidCoreTools.epsilonEquals(anchorA.getInputMessage().getWeight(), anchorB.getInputMessage().getWeight(), 1e-3))
         return true;
      if (!EuclidCoreTools.epsilonEquals(anchorA.getInputMessage().getDesiredPosition(), anchorB.getInputMessage().getDesiredPosition(), 1e-3))
         return true;
      return false;
   }

   private static boolean areSixDoFAnchorsEqual(KinematicsToolboxRigidBodyMessage messageA, KinematicsToolboxRigidBodyMessage messageB)
   {
      if (!messageA.getLinearSelectionMatrix().epsilonEquals(messageB.getLinearSelectionMatrix(), 0.0))
         return false;
      if (!messageA.getAngularSelectionMatrix().epsilonEquals(messageB.getAngularSelectionMatrix(), 0.0))
         return false;

      if (!messageA.getLinearWeightMatrix().epsilonEquals(messageB.getLinearWeightMatrix(), 0.0))
         return false;
      if (!messageA.getLinearWeightMatrix().epsilonEquals(messageB.getLinearWeightMatrix(), 0.0))
         return false;

      if (messageA.getLinearSelectionMatrix().getXSelected() && !EuclidCoreTools.epsilonEquals(messageA.getDesiredPositionInWorld().getX(), messageB.getDesiredPositionInWorld().getX(), 1e-3))
         return false;
      if (messageA.getLinearSelectionMatrix().getYSelected() && !EuclidCoreTools.epsilonEquals(messageA.getDesiredPositionInWorld().getY(), messageB.getDesiredPositionInWorld().getY(), 1e-3))
         return false;
      if (messageA.getLinearSelectionMatrix().getZSelected() && !EuclidCoreTools.epsilonEquals(messageA.getDesiredPositionInWorld().getZ(), messageB.getDesiredPositionInWorld().getZ(), 1e-3))
         return false;

      Vector3D rotationA = new Vector3D();
      Vector3D rotationB = new Vector3D();
      messageA.getDesiredOrientationInWorld().getRotationVector(rotationA);
      messageB.getDesiredOrientationInWorld().getRotationVector(rotationB);

      if (messageA.getAngularSelectionMatrix().getXSelected() && !EuclidCoreTools.epsilonEquals(rotationA.getX(), rotationB.getX(), 1e-3))
         return false;
      if (messageA.getAngularSelectionMatrix().getYSelected() && !EuclidCoreTools.epsilonEquals(rotationA.getY(), rotationB.getY(), 1e-3))
         return false;
      if (messageA.getAngularSelectionMatrix().getZSelected() && !EuclidCoreTools.epsilonEquals(rotationA.getZ(), rotationB.getZ(), 1e-3))
         return false;

      return true;
   }

   private static boolean isCoMEnabled(CenterOfMassMotionControlAnchorDescription description)
   {
      return description != null &&
             (description.getInputMessage().getSelectionMatrix().getXSelected() ||
             description.getInputMessage().getSelectionMatrix().getYSelected() ||
             description.getInputMessage().getSelectionMatrix().getZSelected());
   }
}
