package us.ihmc.commonWalkingControlModules.corruptors;

import java.util.ArrayList;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.yoVariables.listener.VariableChangedListener;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFramePoint3D;
import us.ihmc.yoVariables.variable.YoVariable;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.RobotSpecificJointNames;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RevoluteJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.commons.FormattingTools;

public class FullRobotModelCorruptor
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final ArrayList<VariableChangedListener> variableChangedListeners = new ArrayList<VariableChangedListener>();

   public FullRobotModelCorruptor(final FullHumanoidRobotModel fullRobotModel, YoVariableRegistry parentRegistry)
   {
      this("", fullRobotModel, parentRegistry);
   }

   public FullRobotModelCorruptor(String namePrefix, final FullHumanoidRobotModel fullRobotModel, YoVariableRegistry parentRegistry)
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
         final RigidBody thigh = fullRobotModel.getLegJoint(robotSide, LegJointName.KNEE_PITCH).getPredecessor();
         createMassAndCoMOffsetCorruptors(namePrefix, thighName, thigh);

         String shinName = sidePrefix + "Shin";
         final RigidBody shin = fullRobotModel.getLegJoint(robotSide, LegJointName.KNEE_PITCH).getSuccessor();
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

//      final YoFrameVector hipYawOffset = new YoFrameVector("hipYawOffset", null, registry);
//      VariableChangedListener hipYawOffsetChangedListener = new VariableChangedListener()
//      {
//         @Override
//         public void notifyOfVariableChange(YoVariable<?> v)
//         {
//            for (RobotSide robotSide : RobotSide.values)
//            {
//               OneDoFJoint hipYawJoint = fullRobotModel.getLegJoint(robotSide, LegJointName.HIP_YAW);
//               ReferenceFrame frameBeforeJoint = hipYawJoint.getFrameBeforeJoint();
//
//               RigidBodyTransform postCorruptionTransform = new RigidBodyTransform();
//               Vector3D offsetVector = hipYawOffset.getVector3dCopy();
//               offsetVector.setY(robotSide.negateIfRightSide(offsetVector.getY()));
//               postCorruptionTransform.setTranslation(offsetVector);
//               frameBeforeJoint.corruptTransformToParentPostMultiply(postCorruptionTransform);
//            }
//         }
//      };
//      hipYawOffset.attachVariableChangedListener(hipYawOffsetChangedListener);
//      variableChangedListeners.add(hipYawOffsetChangedListener);

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
//      name = FormattingTools.addPrefixAndKeepCamelCase(namePrefix, name);
//      final ReferenceFrame frameBeforeJoint = oneDoFJoint.getFrameBeforeJoint();
//      final Vector3D jointAxis = oneDoFJoint.getJointAxis().getVectorCopy();
//
//      final RigidBodyTransform preCorruptionTransform = new RigidBodyTransform();
//
//      final YoDouble offset = new YoDouble(name + "Offset", registry);
//
//      VariableChangedListener jointOffsetChangedListener = new VariableChangedListener()
//      {
//         @Override
//         public void notifyOfVariableChange(YoVariable<?> v)
//         {
//            AxisAngle axisAngle = new AxisAngle(jointAxis, offset.getDoubleValue());
//            preCorruptionTransform.setRotationAndZeroTranslation(axisAngle);
//            frameBeforeJoint.corruptTransformToParentPreMultiply(preCorruptionTransform);
//         }
//      };
//      offset.addVariableChangedListener(jointOffsetChangedListener);
//      variableChangedListeners.add(jointOffsetChangedListener);
   }

   private void createMassAndCoMOffsetCorruptors(String namePrefix, String name, final RigidBody rigidBody)
   {
      if(rigidBody == null)
      {
         return;
      }
      
      name = FormattingTools.addPrefixAndKeepCamelCase(namePrefix, name);
      final YoDouble massVariable = new YoDouble(name + "Mass", registry);
      massVariable.set(rigidBody.getInertia().getMass());

      VariableChangedListener massVariableChangedListener = new VariableChangedListener()
      {
         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            rigidBody.getInertia().setMass(massVariable.getDoubleValue());
         }
      };
      massVariable.addVariableChangedListener(massVariableChangedListener);
      variableChangedListeners.add(massVariableChangedListener);

      FramePoint3D originalCoMOffset = new FramePoint3D();
      rigidBody.getCoMOffset(originalCoMOffset);
      final YoFramePoint3D rigidBodyCoMOffset = new YoFramePoint3D(name + "CoMOffset", rigidBody.getParentJoint().getFrameAfterJoint(), registry);
      rigidBodyCoMOffset.setMatchingFrame(originalCoMOffset);

      VariableChangedListener rigidBodyCoMOffsetChangedListener = new VariableChangedListener()
      {
         private final FramePoint3D tempFramePoint = new FramePoint3D();

         @Override
         public void notifyOfVariableChange(YoVariable<?> v)
         {
            tempFramePoint.setIncludingFrame(rigidBodyCoMOffset);
            tempFramePoint.changeFrame(rigidBody.getBodyFixedFrame());
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
         variableChangedListener.notifyOfVariableChange(null);
      }
   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }
}
