package us.ihmc.atlas.handControl.sandia;

import java.util.ArrayList;
import java.util.EnumMap;

import us.ihmc.avatar.handControl.FingerJoint;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.sensors.ForceSensorDataReadOnly;

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

   private final ForceSensorDataReadOnly wristForceSensor;
   private final OneDoFJoint wristJoint;
   private final RobotSide robotSide;

   public SandiaHandModel(FullRobotModel fullRobotModel, ForceSensorDataReadOnly forceSensorDataForController, RobotSide robotSide)
   {
      this.robotSide = robotSide;
      String prefix = robotSide == RobotSide.LEFT ? "left_" : "right_";
      addFingerJoints(fullRobotModel, prefix + "f3", SandiaFingerName.THUMB);
      addFingerJoints(fullRobotModel, prefix + "f0", SandiaFingerName.INDEX);
      addFingerJoints(fullRobotModel, prefix + "f1", SandiaFingerName.MIDDLE);
      addFingerJoints(fullRobotModel, prefix + "f2", SandiaFingerName.RING);

      this.wristForceSensor = forceSensorDataForController;
      wristJoint = fullRobotModel.getOneDoFJointByName(robotSide.getShortLowerCaseName() + "_arm_wrx");
   }

   private FingerJoint addFingerJoint(String name,  FullRobotModel fullRobotModel)
   {
      FingerJoint fingerJoint = new FingerJoint(name);
      OneDoFJoint oneDoFJoint = fullRobotModel.getOneDoFJointByName(name);
      allJoints.add(fingerJoint);
      return fingerJoint;
   }

   private void addFingerJoints(FullRobotModel fullRobotModel, String prefix, SandiaFingerName finger)
   {
      EnumMap<SandiaFingerJointName, FingerJoint> fingerJoints = new EnumMap<SandiaFingerJointName, FingerJoint>(SandiaFingerJointName.class);
      fingerJoints.put(SandiaFingerJointName.BASEJOINT, addFingerJoint(prefix + "_j0", fullRobotModel));
      fingerJoints.put(SandiaFingerJointName.FIRSTJOINT, addFingerJoint(prefix + "_j1", fullRobotModel));
      fingerJoints.put(SandiaFingerJointName.SECONDJOINT, addFingerJoint(prefix + "_j2", fullRobotModel));
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

   public ForceSensorDataReadOnly getWristForceSensor()
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
