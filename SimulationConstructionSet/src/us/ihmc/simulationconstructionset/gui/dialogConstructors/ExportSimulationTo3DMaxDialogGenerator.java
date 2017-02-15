package us.ihmc.simulationconstructionset.gui.dialogConstructors;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;

import javax.swing.JFileChooser;
import javax.vecmath.Vector3d;

import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class ExportSimulationTo3DMaxDialogGenerator implements ExportSimulationTo3DMaxDialogConstructor
{
   private SimulationConstructionSet sim;
   private JFileChooser fileChooser = new JFileChooser();
   private ArrayList<String> partNamesThatExist = new ArrayList<String>();
   private ArrayList<Object[]> angleOffsets = new ArrayList<Object[]>();
   private ArrayList<Object[]> posOffsets = new ArrayList<Object[]>();
   private int index = 0;
   private boolean rightAnklePitchSet = false;
   private boolean rightAnkleRollSet = false;
   private boolean leftAnklePitchSet = false;
   private boolean leftAnkleRollSet = false;
   private Vector3d rightAnklePitchTran = new Vector3d();
   private Vector3d rightAnkleRollTran = new Vector3d();
   private Vector3d leftAnklePitchTran = new Vector3d();
   private Vector3d leftAnkleRollTran = new Vector3d();
   private float[] rightAnklePitchAngle = new float[3];
   private float[] rightAnkleRollAngle = new float[3];
   private float[] leftAnklePitchAngle = new float[3];
   private float[] leftAnkleRollAngle = new float[3];

   public ExportSimulationTo3DMaxDialogGenerator(SimulationConstructionSet sim)
   {
      this.sim = sim;
   }

   @Override
   public void constructDialog()
   {
//    for (Robot robot : sim.getRobots())
//    {
//       for (Joint joint : robot.getRootJoints())
//       {
//          printJointsAndPositions(joint);
//       }
//    }
//    System.exit(0);
      File home = new File(System.getProperty("user.home") + File.separator + "Documents" + File.separator);
      File[] files = home.listFiles();
      int highestIndex = 0;
      File file = null;
      if (files != null)
      {
         for (int fileindex = 0; fileindex < files.length; fileindex++)
         {
            if (files[fileindex].getName().startsWith("TestSCSScript"))
            {
               highestIndex = Math.max(Integer.parseInt(files[fileindex].getName().substring(13, 15)), highestIndex);
            }
         }

         highestIndex++;
         file = new File(System.getProperty("user.home") + File.separator + "Documents" + File.separator + "TestSCSScript" + highestIndex + ".ms");
      }
      else
      {
         if (fileChooser.showSaveDialog(null) == JFileChooser.APPROVE_OPTION)
         {
            file = fileChooser.getSelectedFile();
         }
      }

      if (file != null)
      {
         addExistingParts();
         addAngleOffsets();
         addPosOffsets();

         // File file = fileChooser.getSelectedFile();
         FileWriter fileWriter = null;
         try
         {
            fileWriter = new FileWriter(file);
            fileWriter.write("");
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }

         try
         {
            fileWriter.write(initialScript());
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }

         boolean done = false;
         sim.setTick(0);
         index = 0;

         while (!done)
         {
            try
            {
               if (index == sim.getIndex() / 30)
               {
                  fileWriter.append("\tat time " + index + "\n\t(\n");
               }
               else
               {
                  fileWriter.append("\tat time " + index
                                    + " -- transition --------------------------------------------------------------------------------------\n\t(\n");
               }
            }
            catch (IOException e)
            {
               e.printStackTrace();
            }

            for (Robot robot : sim.getRobots())
            {
               for (Joint joint : robot.getRootJoints())
               {
                  try
                  {
                     Vector3d translation = new Vector3d();
                     joint.getTranslationToWorld(translation);
                     fileWriter.append(getJointPositionsAndTranslations(joint, (index != sim.getIndex() / 30)));    // , translation));
                  }
                  catch (IOException e)
                  {
                     e.printStackTrace();
                  }
               }
            }

            try
            {
               fileWriter.append("\t)\n");
            }
            catch (IOException e)
            {
               e.printStackTrace();
            }

            if (index == sim.getIndex() / 30)
            {
               done = sim.tick();
            }

            if (!done)
            {
               index++;
            }
         }

         try
         {
            fileWriter.append(endScript());
            fileWriter.close();
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
      }

      sim.closeAndDispose();
      System.exit(0);
   }

   private String getJointPositionsAndTranslations(Joint joint, boolean transition)    // , Vector3d parentWorldPosition)
   {
      String ret = "";

//    Vector3d offset = new Vector3d();
//    joint.getOffset(offset);
//    offset.add(parentWorldPosition);
      if (doesPartExist(joint.getName()))
      {
         float xAngle = 0;
         float yAngle = 0;
         float zAngle = 0;
         float x = 0;
         float y = 0;
         float z = 0;
         if (!transition)
         {
            double[] rawAngle = joint.get3DRotation();
            xAngle = (float) Math.toDegrees(rawAngle[0]);
            yAngle = (float) Math.toDegrees(rawAngle[1]);
            zAngle = (float) Math.toDegrees(rawAngle[2]);
            Vector3d translation = new Vector3d();
            joint.getTranslationToWorld(translation);
            x = (float) translation.getX();
            y = (float) translation.getY();
            z = (float) translation.getZ();

//          x = offset.x;
//          y = offset.y;
//          z = offset.z;
         }
         else
         {
            sim.unTick();
            double[] rawAngle = joint.get3DRotation();
            double xAngle1 = Math.toDegrees(rawAngle[0]);
            double yAngle1 = Math.toDegrees(rawAngle[1]);
            double zAngle1 = Math.toDegrees(rawAngle[2]);
            Vector3d translation = new Vector3d();
            joint.getTranslationToWorld(translation);
            double x1 = translation.getX();
            double y1 = translation.getY();
            double z1 = translation.getZ();

//          joint.getOffset(offset);
//          offset.add(parentWorldPosition);
//          double x1 = offset.x;
//          double y1 = offset.y;
//          double z1 = offset.z;

            sim.tick();
            rawAngle = joint.get3DRotation();
            double xAngle2 = Math.toDegrees(rawAngle[0]);
            double yAngle2 = Math.toDegrees(rawAngle[1]);
            double zAngle2 = Math.toDegrees(rawAngle[2]);
            joint.getTranslationToWorld(translation);
            double x2 = translation.getX();
            double y2 = translation.getY();
            double z2 = translation.getZ();

//          joint.getOffset(offset);
//          offset.add(parentWorldPosition);
//          double x2 = offset.x;
//          double y2 = offset.y;
//          double z2 = offset.z;

            xAngle = (float) ((xAngle1 + xAngle2) / 2);
            yAngle = (float) ((yAngle1 + yAngle2) / 2);
            zAngle = (float) ((zAngle1 + zAngle2) / 2);

            x = (float) ((x1 + x2) / 2);
            y = (float) ((y1 + y2) / 2);
            z = (float) ((z1 + z2) / 2);
         }

         ret = "\t\t" + joint.getName() + ".pos = [" + convertDoubletoMaxScriptDoubleString(x) + "," + convertDoubletoMaxScriptDoubleString(y) + ","
               + convertDoubletoMaxScriptDoubleString(z) + "]\n";

//       double[] angleOffset = getAngleOffset(joint.getName());
         float[] angleOffset = getFloatAngleOffset(joint.getName());
         ret += "\t\tSetObjectRotation " + joint.getName() + " " + convertDoubletoMaxScriptDoubleString(xAngle) + " "
                + convertDoubletoMaxScriptDoubleString(yAngle) + " " + convertDoubletoMaxScriptDoubleString(zAngle) + " "
                + convertDoubletoMaxScriptDoubleString(angleOffset[0]) + " " + convertDoubletoMaxScriptDoubleString(angleOffset[1]) + " "
                + convertDoubletoMaxScriptDoubleString(angleOffset[2]) + "\n";
      }
      else if (joint.getName().equals("rightAnklePitch"))
      {
         rightAnklePitchSet = true;
         joint.getTranslationToWorld(rightAnklePitchTran);

//       rightAnklePitchAngle = joint.get3DRotation();
         double[] temprightAnklePitchAngle = joint.get3DRotation();
         for (int i = 0; i < temprightAnklePitchAngle.length; i++)
         {
            rightAnklePitchAngle[i] = (float) temprightAnklePitchAngle[i];
         }

         if (rightAnkleRollSet)
         {
            ret = setRightFoot();
         }
      }
      else if (joint.getName().equals("rightAnkleRoll"))
      {
         rightAnkleRollSet = true;
         joint.getTranslationToWorld(rightAnkleRollTran);

//       rightAnkleRollAngle = joint.get3DRotation();
         double[] temprightAnklePitchAngle = joint.get3DRotation();
         for (int i = 0; i < temprightAnklePitchAngle.length; i++)
         {
            rightAnkleRollAngle[i] = (float) temprightAnklePitchAngle[i];
         }

         if (rightAnklePitchSet)
         {
            ret = setRightFoot();
         }
      }
      else if (joint.getName().equals("leftAnklePitch"))
      {
         leftAnklePitchSet = true;
         joint.getTranslationToWorld(leftAnklePitchTran);

//       leftAnklePitchAngle = joint.get3DRotation();
         double[] temprightAnklePitchAngle = joint.get3DRotation();
         for (int i = 0; i < temprightAnklePitchAngle.length; i++)
         {
            leftAnklePitchAngle[i] = (float) temprightAnklePitchAngle[i];
         }

         if (leftAnkleRollSet)
         {
            ret = setLeftFoot();
         }
      }
      else if (joint.getName().equals("leftAnkleRoll"))
      {
         leftAnkleRollSet = true;
         joint.getTranslationToWorld(leftAnkleRollTran);

//       leftAnkleRollAngle = joint.get3DRotation();
         double[] temprightAnklePitchAngle = joint.get3DRotation();
         for (int i = 0; i < temprightAnklePitchAngle.length; i++)
         {
            leftAnkleRollAngle[i] = (float) temprightAnklePitchAngle[i];
         }

         if (leftAnklePitchSet)
         {
            ret = setLeftFoot();
         }
      }

      for (Joint childJoint : joint.getChildrenJoints())
      {
         ret += getJointPositionsAndTranslations(childJoint, transition);    // , offset);
      }

      return ret;
   }

   private String setRightFoot()
   {
      String ret = "\t\trightFoot.pos = [" + convertDoubletoMaxScriptDoubleString((float) rightAnklePitchTran.getX()) + ","
                   + convertDoubletoMaxScriptDoubleString((float) rightAnklePitchTran.getY()) + ","
                   + convertDoubletoMaxScriptDoubleString((float) rightAnklePitchTran.getZ()) + "]\n" + "\t\tSetObjectRotation rightFoot "
                   + convertDoubletoMaxScriptDoubleString(rightAnklePitchAngle[1]) + " " + convertDoubletoMaxScriptDoubleString(rightAnkleRollAngle[0])
                   + " 0 0 0 180\n";
      rightAnklePitchSet = false;
      rightAnkleRollSet = false;

      return ret;
   }

   private String setLeftFoot()
   {
      String ret = "\t\tleftFoot.pos = [" + convertDoubletoMaxScriptDoubleString((float) leftAnklePitchTran.getX()) + ","
                   + convertDoubletoMaxScriptDoubleString((float) leftAnklePitchTran.getY()) + ","
                   + convertDoubletoMaxScriptDoubleString((float) leftAnklePitchTran.getZ()) + "]\n" + "\t\tSetObjectRotation leftFoot "
                   + convertDoubletoMaxScriptDoubleString(leftAnklePitchAngle[1]) + " " + convertDoubletoMaxScriptDoubleString(leftAnkleRollAngle[0])
                   + " 0 0 0 180\n";
      leftAnklePitchSet = false;
      leftAnkleRollSet = false;

      return ret;
   }

   private String initialScript()
   {
      String code = "resetMaxFile #noprompt\n" + "loaded = loadMaxFile \"C:\\Users\\Madison\\Documents\\R2Robot_Madison_v09.max\" useFileUnits:true\n"
                    + "chest = $chest\n" + "leftShoulderPitch = $leftShoulderPitch\n" + "leftShoulderRoll = $leftShoulderRoll\n"
                    + "leftShoulderYaw = $leftShoulderYaw\n" + "leftElbow = $leftElbow\n" + "rightShoulderPitch = $rightShoulderPitch\n"
                    + "rightShoulderRoll = $rightShoulderRoll\n" + "rightShoulderYaw = $rightShoulderYaw\n" + "rightElbow = $rightElbow\n"
                    + "lowerNeckPitch = $lowerNeckPitch\n" + "neckYaw = $neckYaw\n" + "upperNeckPitch = $head\n" + "spinePitch = $spinePitch\n"
                    + "spineYaw = $spineYaw\n" + "spineRoll = $spineRoll\n" + "leftHipPitch = $leftHipPitch\n" + "leftHipRoll = $leftHipRoll\n"
                    + "leftHipYaw = $leftHipYaw\n" + "leftKnee = $leftKnee\n" +

      // "leftAnkleRoll = $Cylinder002\n"+
      // "leftAnklePitch = $Cylinder001\n"+

      "rightHipPitch = $rightHipPitch\n" + "rightHipRoll = $rightHipRoll\n" + "rightHipYaw = $rightHipYaw\n" + "rightKnee = $rightKnee\n" +

      // "rightAnkleRoll = $Cylinder026\n"+
      // "rightAnklePitch = $Cylinder028\n"+

      "rightFoot = $rightFoot\n" + "leftFoot = $leftFoot\n" + "\n" + "fn RotatePivotOnly obj rotation =\n" + "(\n"
                                 + "\tlocal rotValInv=inverse (rotation as quat)\n" + "\tin coordsys local obj.rotation*=RotValInv\n" + ")\n\n"
                                 + "fn SetObjectRotation obj rx ry rz ox oy oz =\n" + "(\n" + "\tlocal translateMat = transMatrix obj.transform.pos\n"
                                 + "\tlocal scaleMat = scaleMatrix obj.transform.scale\n" + "\tobj.transform = scaleMat * translateMat\n"
                                 + "\trot = eulerangles rx ry rz\n" + "\tRotatePivotOnly obj rot\n" + "\trot = eulerangles ox oy oz\n"
                                 + "\tRotatePivotOnly obj rot\n" + ")\n\n" + "animate on\n" + "(\n";

      return code;
   }

   private void addExistingParts()
   {
      partNamesThatExist.add("leftHipPitch");
      partNamesThatExist.add("leftHipRoll");
      partNamesThatExist.add("leftHipYaw");
      partNamesThatExist.add("leftKnee");
      partNamesThatExist.add("rightHipPitch");
      partNamesThatExist.add("rightHipRoll");
      partNamesThatExist.add("rightHipYaw");
      partNamesThatExist.add("rightKnee");
      partNamesThatExist.add("spineRoll");
      partNamesThatExist.add("spineYaw");
      partNamesThatExist.add("spinePitch");
      partNamesThatExist.add("lowerNeckPitch");
      partNamesThatExist.add("neckYaw");
      partNamesThatExist.add("upperNeckPitch");
      partNamesThatExist.add("chest");
      partNamesThatExist.add("leftShoulderPitch");
      partNamesThatExist.add("leftShoulderRoll");
      partNamesThatExist.add("leftShoulderYaw");
      partNamesThatExist.add("leftElbow");
      partNamesThatExist.add("rightShoulderPitch");
      partNamesThatExist.add("rightShoulderRoll");
      partNamesThatExist.add("rightShoulderYaw");
      partNamesThatExist.add("rightElbow");
   }

   private void addAngleOffsets()
   {
      angleOffsets.add(new Object[] {"leftHipPitch", 180.0, 180.0, 0.0});
      angleOffsets.add(new Object[] {"leftHipRoll", 90.0, 90.0, 0.0});
      angleOffsets.add(new Object[] {"leftKnee", -90.0, 0.0, 90.0});
      angleOffsets.add(new Object[] {"rightKnee", -90.0, 0.0, 90.0});
      angleOffsets.add(new Object[] {"spineRoll", 90.0, 0.0, 90.0});
      angleOffsets.add(new Object[] {"leftShoulderYaw", 0.0, 180.0, 0.0});
   }

   private void addPosOffsets()
   {
      posOffsets.add(new Object[] {"spineRoll", 0.0, 0.0, -0.1016});
      posOffsets.add(new Object[] {"spineYaw", 0.0254, 0.0, 0.0254});
      posOffsets.add(new Object[] {"spinePitch", 0.0254, 0.0, 0.03});
   }

// private double[] getAngleOffset(String partName)
// {
//    double[] angleOffset = {0, 0, 0};
//    for (int i = 0; i < angleOffsets.size(); i++)
//    {
//       if (partName.equals((String) angleOffsets.get(i)[0]))
//       {
//          angleOffset[0] = ((Double) angleOffsets.get(i)[1]).doubleValue();
//          angleOffset[1] = ((Double) angleOffsets.get(i)[2]).doubleValue();
//          angleOffset[2] = ((Double) angleOffsets.get(i)[3]).doubleValue();
//
//          break;
//       }
//    }
//
//    return angleOffset;
// }

   private float[] getFloatAngleOffset(String partName)
   {
      float[] angleOffset = {0, 0, 0};
      for (int i = 0; i < angleOffsets.size(); i++)
      {
         if (partName.equals((String) angleOffsets.get(i)[0]))
         {
            angleOffset[0] = (float) ((Double) angleOffsets.get(i)[1]).doubleValue();
            angleOffset[1] = (float) ((Double) angleOffsets.get(i)[2]).doubleValue();
            angleOffset[2] = (float) ((Double) angleOffsets.get(i)[3]).doubleValue();

            break;
         }
      }

      return angleOffset;
   }

// private double[] getPosOffset(String partName)
// {
//    double[] posOffset = { 0, 0, 0 };
//                    for (int i = 0; i < posOffsets.size(); i++) {
//                            if (partName.equals((String) posOffsets.get(i)[0])) {
//                                    posOffset[0] = ((Double) posOffsets.get(i)[1]).doubleValue();
//                                    posOffset[1] = ((Double) posOffsets.get(i)[2]).doubleValue();
//                                    posOffset[2] = ((Double) posOffsets.get(i)[3]).doubleValue();
//                                    break;
//                            }
//                    }
//    return posOffset;
// }

   private String endScript()
   {
      return ")\n" + "animationrange = interval 0 " + index;
   }

// private double convertMetersToInches(double meters)
// {
//    return meters;    // *39.3700787;
// }

   private boolean doesPartExist(String partName)
   {
      boolean exists = false;
      for (String name : partNamesThatExist)
      {
         if (partName.equals(name))
         {
            exists = true;

            break;
         }
      }

      return exists;
   }

// private String space(String name, int totalSpace)
// {
//    int numberOfSpacesNeeded = Math.max(totalSpace - name.length(), 0);
//    String spaces = "";
//    for (int i = 0; i < numberOfSpacesNeeded; i++)
//    {
//       spaces += " ";
//    }
//
//    return spaces;
// }

// private String convertDoubletoMaxScriptDoubleString(double value)
// {
//  return replaceAll(Double.toString(value), "E", "d");
// }

   private String convertDoubletoMaxScriptDoubleString(float value)
   {
      return replaceAll(Double.toString(value), "E", "e");
   }

   private String replaceAll(String orig, String regex, String rep)
   {
      int index = 0;
      while (index + regex.length() < orig.length())
      {
         if (orig.substring(index, index + regex.length()).equals(regex))
         {
            orig = replace(orig, index, index + regex.length(), rep);
            index += rep.length() - 1;
         }

         index++;
      }

      return orig;
   }

   private String replace(String fullString, int beginIndex, int endIndex, String replacement)
   {
      if ((beginIndex >= 0) && (beginIndex < fullString.length()) && (endIndex >= 0) && (endIndex < fullString.length()))
      {
         String prefix = fullString.substring(0, beginIndex);
         String suffix = fullString.substring(endIndex);

         return prefix + replacement + suffix;
      }
      else
      {
         return null;
      }
   }

   public void closeAndDispose()
   {
      sim = null;
      fileChooser = null;
   }

// private void printJointsAndPositions(Joint joint)
// {
//    Matrix3d rotation = new Matrix3d();
//    joint.getRotationFromWorld(rotation);
//
////   double xAngle = Math.toDegrees(Math.atan2(rotation.m20, rotation.m21));
////   double yAngle = Math.toDegrees(Math.acos(rotation.m22));
////   double zAngle = Math.toDegrees(-Math.atan2(rotation.m02, rotation.m12));
//    Vector3d translation = new Vector3d();
//    joint.getTranslationFromWorld(translation);
//
//    // for (int i = 0; i < 25; i++) {System.out.print(" ");}System.out.print("|"); for (int i = 0; i < 66; i++) {System.out.print(" ");} for (int j = 0; j < 3; j++) {System.out.print("|");   for (int i = 0; i < 24; i++) {System.out.print(" ");}}
//    //
//    // System.out.println("");
//    // System.out.println(joint.getName() + space(joint.getName(), 25) + "| "+translation + space(translation.toString(), 65) + "| "+xAngle +space(xAngle, 23)+ "| "+yAngle +space(yAngle, 23)+ "| "+zAngle);
//    System.out.print(joint.getName() + space(joint.getName(), 25));
//
//    if (joint instanceof FloatingJoint)
//    {
//       System.out.println("FloatingJoint");
//    }
//    else if (joint instanceof FloatingPlanarJoint)
//    {
//       System.out.println("FloatingPlanarJoint");
//    }
//    else if (joint instanceof CylinderJoint)
//    {
//       System.out.println("CylinderJoint");
//    }
//    else if (joint instanceof FreeJoint)
//    {
//       System.out.println("FreeJoint");
//    }
//    else if (joint instanceof GimbalJoint)
//    {
//       System.out.println("GimbalJoint");
//    }
//    else if (joint instanceof SliderJoint)
//    {
//       System.out.println("SliderJoint");
//    }
//    else if (joint instanceof UniversalJoint)
//    {
//       System.out.println("UniversalJoint");
//    }
//    else if (joint instanceof NullJoint)
//    {
//       System.out.println("NullJoint");
//    }
//    else if (joint instanceof PinJoint)
//    {
//       System.out.println("PinJoint");
//    }
//    else
//    {
//       System.out.println("None");
//    }
//
//    //
//    // for (int i = 0; i < 25; i++) {System.out.print("-");}System.out.print("+"); for (int i = 0; i < 66; i++) {System.out.print("-");} for (int j = 0; j < 3; j++) {System.out.print("+"); for (int i = 0; i < 24; i++) {System.out.print("-");}}
//    //
//    // System.out.println("");
//    for (Joint childJoint : joint.getChildrenJoints())
//    {
//       printJointsAndPositions(childJoint);
//    }
// }
}

