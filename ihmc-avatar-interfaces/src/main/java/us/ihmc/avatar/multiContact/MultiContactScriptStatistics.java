package us.ihmc.avatar.multiContact;

import toolbox_msgs.msg.dds.KinematicsToolboxRigidBodyMessage;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.log.LogTools;
import us.ihmc.tools.io.WorkspacePathTools;

import javax.swing.*;
import javax.swing.filechooser.FileNameExtensionFilter;
import java.io.File;
import java.nio.file.Path;
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

      int numKfs = script.size();
      int numSixDof = script.get(0).getSixDoFAnchors().size();
      int numSixDofContact = 0;
      int numSixDofNonContact = 0;

      for (int i = 0; i < script.get(0).getSixDoFAnchors().size(); i++)
      {
         if (script.get(0).getSixDoFAnchors().get(i).isContactState())
            numSixDofContact++;
         else
            numSixDofNonContact++;
      }

      int numOneDof = script.get(0).getOneDoFAnchors().size();
      int numComAnchors = (script.get(0).getCenterOfMassAnchor() != null && isCoMEnabled(script.get(0).getCenterOfMassAnchor())) ? 1 : 0;
      System.out.println("frame \t new \t old");

      for (int i = 1; i < script.size(); i++)
      {
         KinematicsToolboxSnapshotDescription thisKf = script.get(i);
         KinematicsToolboxSnapshotDescription prevKf = script.get(i - 1);

         int newSixDofs = 0;
         int oldSixDofs = 0;

         for (SixDoFMotionControlAnchorDescription thisSixDof : thisKf.getSixDoFAnchors())
         {
            boolean foundMatch = false;
            for (SixDoFMotionControlAnchorDescription prevSixDof : prevKf.getSixDoFAnchors())
            {
               if (areSixDoFAnchorsEqual(thisSixDof.getInputMessage(), prevSixDof.getInputMessage()))
               {
                  foundMatch = true;
                  break;
               }
            }

            if (!foundMatch)
            {
               numSixDof++;
               newSixDofs++;

               if (thisSixDof.isContactState())
                  numSixDofContact++;
               else
                  numSixDofNonContact++;
            }
            else
            {
               oldSixDofs++;
            }
         }

         System.out.println(i + "\t\t" + newSixDofs + "\t\t" + oldSixDofs);

         for (OneDoFMotionControlAnchorDescription thisOneDof : thisKf.getOneDoFAnchors())
         {
            boolean foundMatch = false;
            for (OneDoFMotionControlAnchorDescription prevOneDof : prevKf.getOneDoFAnchors())
            {
               if (thisOneDof.getInputMessage().epsilonEquals(prevOneDof.getInputMessage(), 1e-2))
               {
                  foundMatch = true;
                  break;
               }
            }

            if (!foundMatch)
               numOneDof++;
         }

         CenterOfMassMotionControlAnchorDescription thisCoM = thisKf.getCenterOfMassAnchor();
         CenterOfMassMotionControlAnchorDescription prevCoM = prevKf.getCenterOfMassAnchor();
         if (thisCoM != null && isCoMEnabled(thisCoM))
         {
            if (prevCoM == null || !thisCoM.getInputMessage().epsilonEquals(prevCoM.getInputMessage(), 1e-2))
            {
               numComAnchors++;
            }
         }
      }

      System.out.println("num kfs   :" + numKfs);
      System.out.println("num 6 dof :" + numSixDof);
      System.out.println("num 1 dof :" + numOneDof);
      System.out.println("num com   :" + numComAnchors);

      System.out.println("latex: " + numKfs + " & " + numSixDofNonContact + " & " + numSixDofContact + " & " + numOneDof + " & " + numComAnchors);
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
      return description.getInputMessage().getSelectionMatrix().getXSelected() ||
             description.getInputMessage().getSelectionMatrix().getYSelected() ||
             description.getInputMessage().getSelectionMatrix().getZSelected();
   }
}
