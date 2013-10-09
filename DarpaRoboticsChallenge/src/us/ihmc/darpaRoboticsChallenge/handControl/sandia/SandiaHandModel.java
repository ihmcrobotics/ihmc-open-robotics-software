package us.ihmc.darpaRoboticsChallenge.handControl.sandia;

import java.util.ArrayList;
import java.util.EnumMap;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.darpaRoboticsChallenge.handControl.FingerJoint;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.sensorProcessing.sensors.ForceSensorData;
import us.ihmc.utilities.screwTheory.OneDoFJoint;

public class SandiaHandModel
{
   public enum SandiaFingerName
   {

      THUMB, INDEX, MIDDLE, RING;

      public final static SandiaFingerName[] fingers = { INDEX, MIDDLE, RING };

      public int getNumber()
      {
         switch (this)
         {
         case THUMB:
            return 3;
         case INDEX:
            return 0;
         case MIDDLE:
            return 1;
         case RING:
            return 2;
         default:
            throw new RuntimeException("Unhandled Case");
         }
      }
      public String getShortName()
      {
         switch (this)
         {
         case THUMB:
            return "f3";
         case INDEX:
            return "f0";
         case MIDDLE:
            return "f1";
         case RING:
            return "f2";
         default:
            throw new RuntimeException("Unhandled Case");
         }
      }
   }

   public enum SandiaFingerJointName
   {
      BASEJOINT, FIRSTJOINT, SECONDJOINT;

      public int getNumber()
      {
         switch (this)
         {
         case BASEJOINT:
            return 0;
         case FIRSTJOINT:
            return 1;
         case SECONDJOINT:
            return 2;
         default:
            throw new RuntimeException("Unhandled Case");
         }
      }
      public String getShortName()
      {
         switch (this)
         {
         case BASEJOINT:
            return "j0";
         case FIRSTJOINT:
            return "j1";
         case SECONDJOINT:
            return "j2";
         default:
            throw new RuntimeException("Unhandled Case");
         }
      }
   }

   private final EnumMap<SandiaFingerName, EnumMap<SandiaFingerJointName, FingerJoint>> handJoints = new EnumMap<SandiaFingerName, EnumMap<SandiaFingerJointName, FingerJoint>>(
         SandiaFingerName.class);

   private final ArrayList<FingerJoint> allJoints = new ArrayList<FingerJoint>();

   private final ForceSensorData wristForceSensor;
   private final OneDoFJoint wristJoint;
   private final RobotSide robotSide;

   public SandiaHandModel(SDFFullRobotModel fullRobotModel, ForceSensorData forceSensorDataForController, RobotSide robotSide)
   {
      this.robotSide = robotSide;
      String prefix = robotSide == RobotSide.LEFT ? "left_" : "right_";
      addFingerJoints(prefix + "f3", SandiaFingerName.THUMB);
      addFingerJoints(prefix + "f0", SandiaFingerName.INDEX);
      addFingerJoints(prefix + "f1", SandiaFingerName.MIDDLE);
      addFingerJoints(prefix + "f2", SandiaFingerName.RING);

      this.wristForceSensor = forceSensorDataForController;
      wristJoint = fullRobotModel.getOneDoFJointByName(robotSide.getShortLowerCaseName() + "_arm_wrx");
   }

   private FingerJoint addFingerJoint(String name)
   {
      FingerJoint fingerJoint = new FingerJoint(name);
      allJoints.add(fingerJoint);
      return fingerJoint;
   }

   private void addFingerJoints(String prefix, SandiaFingerName finger)
   {
      EnumMap<SandiaFingerJointName, FingerJoint> fingerJoints = new EnumMap<SandiaFingerJointName, FingerJoint>(SandiaFingerJointName.class);
      fingerJoints.put(SandiaFingerJointName.BASEJOINT, addFingerJoint(prefix + "_j0"));
      fingerJoints.put(SandiaFingerJointName.FIRSTJOINT, addFingerJoint(prefix + "_j1"));
      fingerJoints.put(SandiaFingerJointName.SECONDJOINT, addFingerJoint(prefix + "_j2"));
      handJoints.put(finger, fingerJoints);
   }

   public EnumMap<SandiaFingerJointName, FingerJoint> getFingerJoints(SandiaFingerName finger)
   {
      return handJoints.get(finger);
   }

   public RobotSide getRobotSide()
   {
      return robotSide;
   }

   public OneDoFJoint getWristJoint()
   {
      return wristJoint;
   }

   public ForceSensorData getWristForceSensor()
   {
      return wristForceSensor;
   }

   public ArrayList<FingerJoint> getHandJoints()
   {
      return allJoints;
   }
   
   public EnumMap<SandiaFingerName,EnumMap<SandiaFingerJointName,FingerJoint>> getJointMap()
   {
      return handJoints;
   }
}
