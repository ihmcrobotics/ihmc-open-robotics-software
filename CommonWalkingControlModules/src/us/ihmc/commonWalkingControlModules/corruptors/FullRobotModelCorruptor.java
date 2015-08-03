package us.ihmc.commonWalkingControlModules.corruptors;

import java.util.ArrayList;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Vector3d;

import us.ihmc.utilities.FormattingTools;
import us.ihmc.robotics.humanoidRobot.model.FullRobotModel;
import us.ihmc.robotics.humanoidRobot.partNames.ArmJointName;
import us.ihmc.robotics.humanoidRobot.partNames.LegJointName;
import us.ihmc.robotics.humanoidRobot.partNames.RobotSpecificJointNames;
import us.ihmc.robotics.humanoidRobot.partNames.SpineJointName;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RevoluteJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.yoUtilities.dataStructure.listener.VariableChangedListener;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;


public class FullRobotModelCorruptor
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final ArrayList<VariableChangedListener> variableChangedListeners = new ArrayList<VariableChangedListener>();

   public FullRobotModelCorruptor(final FullRobotModel fullRobotModel, YoVariableRegistry parentRegistry)
   {
      this("", fullRobotModel, parentRegistry);
   }

   public FullRobotModelCorruptor(String namePrefix, final FullRobotModel fullRobotModel, YoVariableRegistry parentRegistry)
   {
      RobotSpecificJointNames robotSpecificJointNames = fullRobotModel.getRobotSpecificJointNames();
      LegJointName[] legJointNames = robotSpecificJointNames.getLegJointNames();
      ArmJointName[] armJointNames = robotSpecificJointNames.getArmJointNames();
      SpineJointName[] spineJointNames = robotSpecificJointNames.getSpineJointNames();
      
      String chestName = "chest";
      final RigidBody chest = fullRobotModel.getChest();
      createMassAndCoMOffsetCorruptors(namePrefix, chestName, chest);
      
      String pelvisName = "pelvis";
      final RigidBody pelvis = fullRobotModel.getPelvis();
      createMassAndCoMOffsetCorruptors(namePrefix, pelvisName, pelvis);

      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         
         String thighName = sidePrefix + "Thigh";
         final RigidBody thigh = fullRobotModel.getLegJoint(robotSide, LegJointName.KNEE).getPredecessor();
         createMassAndCoMOffsetCorruptors(namePrefix, thighName, thigh);
         
         String shinName = sidePrefix + "Shin";
         final RigidBody shin = fullRobotModel.getLegJoint(robotSide, LegJointName.KNEE).getSuccessor();
         createMassAndCoMOffsetCorruptors(namePrefix, shinName, shin);
         
         String footName = sidePrefix + "Foot";
         final RigidBody foot = fullRobotModel.getFoot(robotSide);
         createMassAndCoMOffsetCorruptors(namePrefix, footName, foot);
     }
      
      
      for (RobotSide robotSide : RobotSide.values)
      {
         for (ArmJointName armJointName : armJointNames)
         {
            OneDoFJoint armJoint = fullRobotModel.getArmJoint(robotSide, armJointName);
            if (armJoint == null)
               continue;

            RigidBody rigidBody = armJoint.getSuccessor();
            createMassAndCoMOffsetCorruptors(namePrefix, rigidBody.getName(), rigidBody);
         }
     }


      final YoFrameVector hipYawOffset = new YoFrameVector("hipYawOffset", null, registry);
      VariableChangedListener hipYawOffsetChangedListener = new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            for (RobotSide robotSide : RobotSide.values)
            {
               OneDoFJoint hipYawJoint = fullRobotModel.getLegJoint(robotSide, LegJointName.HIP_YAW);
               ReferenceFrame frameBeforeJoint = hipYawJoint.getFrameBeforeJoint();

               RigidBodyTransform postCorruptionTransform = new RigidBodyTransform();
               Vector3d offsetVector = hipYawOffset.getVector3dCopy();
               offsetVector.setY(robotSide.negateIfRightSide(offsetVector.getY()));
               postCorruptionTransform.setTranslation(offsetVector);
               frameBeforeJoint.corruptTransformToParentPostMultiply(postCorruptionTransform);
            }
         }
      };
      hipYawOffset.attachVariableChangedListener(hipYawOffsetChangedListener);
      variableChangedListeners.add(hipYawOffsetChangedListener);


      // Joint Calibration offset errors:
      for (RobotSide robotSide : RobotSide.values)
      {
         for (LegJointName legJointName : legJointNames)
         {
            RevoluteJoint oneDoFJoint = (RevoluteJoint) fullRobotModel.getLegJoint(robotSide, legJointName);
            createJointAngleCorruptor(namePrefix, oneDoFJoint.getName(), oneDoFJoint);
         }
         
         for (ArmJointName armJointName : armJointNames)
         {
            RevoluteJoint oneDoFJoint = (RevoluteJoint) fullRobotModel.getArmJoint(robotSide, armJointName);
            createJointAngleCorruptor(namePrefix, oneDoFJoint.getName(), oneDoFJoint);
         }
      }

      for (SpineJointName spineJointName : spineJointNames)
      {
         RevoluteJoint oneDoFJoint = (RevoluteJoint) fullRobotModel.getSpineJoint(spineJointName);
         createJointAngleCorruptor(namePrefix, oneDoFJoint.getName(), oneDoFJoint);
      }

      parentRegistry.addChild(registry);
   }


   private void createJointAngleCorruptor(String namePrefix, String name, RevoluteJoint oneDoFJoint)
   {
      name = FormattingTools.addPrefixAndKeepCamelCase(namePrefix, name);
      final ReferenceFrame frameBeforeJoint = oneDoFJoint.getFrameBeforeJoint();
      final Vector3d jointAxis = oneDoFJoint.getJointAxis().getVectorCopy();

      final RigidBodyTransform preCorruptionTransform = new RigidBodyTransform();

      final DoubleYoVariable offset = new DoubleYoVariable(name + "Offset", registry);

      VariableChangedListener jointOffsetChangedListener = new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            AxisAngle4d axisAngle = new AxisAngle4d(jointAxis, offset.getDoubleValue());
            preCorruptionTransform.setRotationAndZeroTranslation(axisAngle);
            frameBeforeJoint.corruptTransformToParentPreMultiply(preCorruptionTransform);
         }
      };
      offset.addVariableChangedListener(jointOffsetChangedListener);
      variableChangedListeners.add(jointOffsetChangedListener);
   }


   private void createMassAndCoMOffsetCorruptors(String namePrefix, String name, final RigidBody rigidBody)
   {
      name = FormattingTools.addPrefixAndKeepCamelCase(namePrefix, name);
      final DoubleYoVariable massVariable = new DoubleYoVariable(name + "Mass", registry);
      massVariable.set(rigidBody.getInertia().getMass());
      
      VariableChangedListener massVariableChangedListener = new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            rigidBody.getInertia().setMass(massVariable.getDoubleValue());
         }
      };
      massVariable.addVariableChangedListener(massVariableChangedListener);
      variableChangedListeners.add(massVariableChangedListener);
      
      FramePoint originalCoMOffset = new FramePoint();
      rigidBody.packCoMOffset(originalCoMOffset);
      final YoFramePoint rigidBodyCoMOffset = new YoFramePoint(name + "CoMOffset", originalCoMOffset.getReferenceFrame(), registry);
      rigidBodyCoMOffset.set(originalCoMOffset);
      
      VariableChangedListener rigidBodyCoMOffsetChangedListener = new VariableChangedListener()
      {
         private final FramePoint tempFramePoint = new FramePoint();
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            rigidBodyCoMOffset.getFrameTupleIncludingFrame(tempFramePoint);
            rigidBody.setCoMOffset(tempFramePoint);
         }
      };
      rigidBodyCoMOffset.attachVariableChangedListener(rigidBodyCoMOffsetChangedListener);
      variableChangedListeners.add(rigidBodyCoMOffsetChangedListener);
   }
   
   
   public void corruptFullRobotModel()
   {
      for (VariableChangedListener variableChangedListener : variableChangedListeners)
      {
         variableChangedListener.variableChanged(null);
      }
   }
   
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }
}
