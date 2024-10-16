package us.ihmc.commonWalkingControlModules.corruptors;

import java.util.ArrayList;

import us.ihmc.commons.FormattingTools;
import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.commons.robotics.partNames.ArmJointName;
import us.ihmc.commons.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.RobotSpecificJointNames;
import us.ihmc.commons.robotics.partNames.SpineJointName;
import us.ihmc.commons.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.euclid.YoVector3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

public class FullRobotModelCorruptor
{
   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final ArrayList<YoVariableChangedListener> variableChangedListeners = new ArrayList<YoVariableChangedListener>();

   public FullRobotModelCorruptor(final FullHumanoidRobotModel fullRobotModel, YoRegistry parentRegistry)
   {
      this("", fullRobotModel, parentRegistry);
   }

   public FullRobotModelCorruptor(String namePrefix, final FullHumanoidRobotModel fullRobotModel, YoRegistry parentRegistry)
   {
      for (RigidBodyBasics rigidBody : fullRobotModel.getPelvis().subtreeIterable())
      {
         createMassAndCoMOffsetCorruptors(namePrefix, rigidBody.getName(), rigidBody);
      }

      RobotSpecificJointNames robotSpecificJointNames = fullRobotModel.getRobotSpecificJointNames();
      LegJointName[] legJointNames = robotSpecificJointNames.getLegJointNames();
      ArmJointName[] armJointNames = robotSpecificJointNames.getArmJointNames();
      SpineJointName[] spineJointNames = robotSpecificJointNames.getSpineJointNames();

      //      final YoFrameVector hipYawOffset = new YoFrameVector("hipYawOffset", null, registry);
      //      VariableChangedListener hipYawOffsetChangedListener = new VariableChangedListener()
      //      {
      //         @Override
      //         public void notifyOfVariableChange(YoVariable v)
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
            OneDoFJointBasics oneDoFJoint = (OneDoFJointBasics) fullRobotModel.getLegJoint(robotSide, legJointName);
            createJointAngleCorruptor(namePrefix, oneDoFJoint.getName(), oneDoFJoint);
         }

         for (ArmJointName armJointName : armJointNames)
         {
            OneDoFJointBasics oneDoFJoint = (OneDoFJointBasics) fullRobotModel.getArmJoint(robotSide, armJointName);
            createJointAngleCorruptor(namePrefix, oneDoFJoint.getName(), oneDoFJoint);
         }
      }

      for (SpineJointName spineJointName : spineJointNames)
      {
         OneDoFJointBasics oneDoFJoint = (OneDoFJointBasics) fullRobotModel.getSpineJoint(spineJointName);
         createJointAngleCorruptor(namePrefix, oneDoFJoint.getName(), oneDoFJoint);
      }

      parentRegistry.addChild(registry);
   }

   private void createJointAngleCorruptor(String namePrefix, String name, OneDoFJointBasics oneDoFJoint)
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
      //         public void notifyOfVariableChange(YoVariable v)
      //         {
      //            AxisAngle axisAngle = new AxisAngle(jointAxis, offset.getDoubleValue());
      //            preCorruptionTransform.setRotationAndZeroTranslation(axisAngle);
      //            frameBeforeJoint.corruptTransformToParentPreMultiply(preCorruptionTransform);
      //         }
      //      };
      //      offset.addVariableChangedListener(jointOffsetChangedListener);
      //      variableChangedListeners.add(jointOffsetChangedListener);
   }

   private void createMassAndCoMOffsetCorruptors(String namePrefix, String name, final RigidBodyBasics rigidBody)
   {
      if (rigidBody == null)
      {
         return;
      }

      name = FormattingTools.addPrefixAndKeepCamelCase(namePrefix, name);
      final YoDouble massVariable = new YoDouble(name + "Mass", registry);
      YoVector3D momentOfInertiaDiagonal = new YoVector3D(name + "MoIDiagonal", registry);
      Matrix3DBasics momentOfInertia = rigidBody.getInertia().getMomentOfInertia();
      momentOfInertiaDiagonal.set(momentOfInertia.getM00(), momentOfInertia.getM11(), momentOfInertia.getM22());
      massVariable.set(rigidBody.getInertia().getMass());

      YoVariableChangedListener massVariableChangedListener = new YoVariableChangedListener()
      {
         @Override
         public void changed(YoVariable v)
         {
            rigidBody.getInertia().setMass(massVariable.getDoubleValue());
         }
      };

      YoVariableChangedListener moiListener = new YoVariableChangedListener()
      {
         @Override
         public void changed(YoVariable source)
         {
            rigidBody.getInertia().getMomentOfInertia().setM00(momentOfInertiaDiagonal.getX());
            rigidBody.getInertia().getMomentOfInertia().setM11(momentOfInertiaDiagonal.getY());
            rigidBody.getInertia().getMomentOfInertia().setM22(momentOfInertiaDiagonal.getZ());
         }
      };

      massVariable.addListener(massVariableChangedListener);
      variableChangedListeners.add(massVariableChangedListener);
      momentOfInertiaDiagonal.attachVariableChangedListener(moiListener);
      variableChangedListeners.add(moiListener);

      FramePoint3D originalCoMOffset = new FramePoint3D();
      rigidBody.getCenterOfMass(originalCoMOffset);
      final YoFramePoint3D rigidBodyCoMOffset = new YoFramePoint3D(name + "CoMOffset", rigidBody.getParentJoint().getFrameAfterJoint(), registry);
      rigidBodyCoMOffset.setMatchingFrame(originalCoMOffset);

      YoVariableChangedListener rigidBodyCoMOffsetChangedListener = new YoVariableChangedListener()
      {
         private final FramePoint3D tempFramePoint = new FramePoint3D();

         @Override
         public void changed(YoVariable v)
         {
            tempFramePoint.setIncludingFrame(rigidBodyCoMOffset);
            tempFramePoint.changeFrame(rigidBody.getBodyFixedFrame());
            rigidBody.setCenterOfMass(tempFramePoint);
         }
      };
      rigidBodyCoMOffset.attachVariableChangedListener(rigidBodyCoMOffsetChangedListener);
      variableChangedListeners.add(rigidBodyCoMOffsetChangedListener);
   }

   public void corruptFullRobotModel()
   {
      for (YoVariableChangedListener variableChangedListener : variableChangedListeners)
      {
         variableChangedListener.changed(null);
      }
   }

   public YoRegistry getYoVariableRegistry()
   {
      return registry;
   }
}
