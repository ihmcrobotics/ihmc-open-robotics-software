package us.ihmc.commonWalkingControlModules.corruptors;

import java.util.ArrayList;

import us.ihmc.commons.FormattingTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.interfaces.SpatialInertiaBasics;
import us.ihmc.mecano.yoVariables.multiBodySystem.YoRigidBody;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.partNames.ArmJointName;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.RobotSpecificJointNames;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.listener.YoVariableChangedListener;
import us.ihmc.yoVariables.registry.YoRegistry;
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
      SpatialInertiaBasics spatialInertia = rigidBody.getInertia();
      // TODO(jfoster): No idea if this getTransformToParent call results in what I want
      YoRigidBody yoRigidBody = new YoRigidBody(name, rigidBody.getParentJoint(), spatialInertia.getMomentOfInertia(), spatialInertia.getMass(),
                                                spatialInertia.getBodyFrame().getTransformToParent(), registry);

      YoVariableChangedListener massChangedListener = new YoVariableChangedListener()
      {
         @Override
         public void changed(YoVariable source)
         {
            rigidBody.getInertia().setMass(yoRigidBody.getInertia().getMass());
         }
      };

      YoVariableChangedListener centerOfMassOffsetChangedListener = new YoVariableChangedListener()
      {
         @Override
         public void changed(YoVariable source)
         {
            rigidBody.getInertia().setCenterOfMassOffset(yoRigidBody.getInertia().getCenterOfMassOffset());
         }
      };

      YoVariableChangedListener momentOfInertiaChangedListener = new YoVariableChangedListener()
      {
         @Override
         public void changed(YoVariable source)
         {
            rigidBody.getInertia().setMomentOfInertia(yoRigidBody.getInertia().getMomentOfInertia().getM00(),
                                                      yoRigidBody.getInertia().getMomentOfInertia().getM01(),
                                                      yoRigidBody.getInertia().getMomentOfInertia().getM02(),
                                                      yoRigidBody.getInertia().getMomentOfInertia().getM11(),
                                                      yoRigidBody.getInertia().getMomentOfInertia().getM12(),
                                                      yoRigidBody.getInertia().getMomentOfInertia().getM22());
         }
      };

      yoRigidBody.getYoSpatialInertia().getYoMass().addListener(massChangedListener);
      variableChangedListeners.add(massChangedListener);

      yoRigidBody.getYoSpatialInertia().getYoCenterOfMassOffset().attachVariableChangedListener(centerOfMassOffsetChangedListener);
      variableChangedListeners.add(centerOfMassOffsetChangedListener);

      yoRigidBody.getYoSpatialInertia().getYoMomentOfInertia().attachVariableChangedListener(momentOfInertiaChangedListener);
      variableChangedListeners.add(momentOfInertiaChangedListener);
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
