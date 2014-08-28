package us.ihmc.commonWalkingControlModules.corruptors;

import javax.media.j3d.Transform3D;
import javax.vecmath.AxisAngle4d;
import javax.vecmath.Vector3d;

import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.humanoidRobot.partNames.LegJointName;
import us.ihmc.utilities.humanoidRobot.partNames.RobotSpecificJointNames;
import us.ihmc.utilities.humanoidRobot.partNames.SpineJointName;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.RevoluteJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.yoUtilities.dataStructure.listener.VariableChangedListener;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;

import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;

public class FullRobotModelCorruptor
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoFramePoint chestCoMOffset;
   private final DoubleYoVariable chestMass;
   private final YoFrameVector hipYawOffset;

   private final FramePoint tempFramePoint = new FramePoint();

   public FullRobotModelCorruptor(final FullRobotModel fullRobotModel, YoVariableRegistry parentRegistry)
   {
      FramePoint originalChestCoMOffset = new FramePoint();
      final RigidBody chest = fullRobotModel.getChest();
      chest.packCoMOffset(originalChestCoMOffset);

      chestCoMOffset = new YoFramePoint("chestCoMOffset", originalChestCoMOffset.getReferenceFrame(), registry);
      chestCoMOffset.set(originalChestCoMOffset);

      chestCoMOffset.attachVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            chestCoMOffset.getFrameTupleIncludingFrame(tempFramePoint);
            chest.setCoMOffset(tempFramePoint);
         }
      });

      chestMass = new DoubleYoVariable("chestMass", registry);
      chestMass.set(chest.getInertia().getMass());

      chestMass.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            RigidBody chest = fullRobotModel.getChest();
            chest.getInertia().setMass(chestMass.getDoubleValue());
         }
      });

//      for (RobotSide robotSide : RobotSide.values)
//      {
//         FramePoint originalArmCoMOffset = new FramePoint();
//         final RigidBody arm = fullRobotModel.getArmJoint(robotSide, ArmJointName.SHOULDER_YAW).getSuccessor(); // TODO Hacked for Valkyrie
//         arm.packCoMOffset(originalArmCoMOffset);
//
//         final YoFramePoint armCoMOffset = new YoFramePoint(robotSide.getCamelCaseNameForStartOfExpression() + "ArmCoMOffset", originalArmCoMOffset.getReferenceFrame(), registry);
//         armCoMOffset.set(originalArmCoMOffset);
//
//         armCoMOffset.attachVariableChangedListener(new VariableChangedListener()
//         {
//            @Override
//            public void variableChanged(YoVariable<?> v)
//            {
//               armCoMOffset.getFrameTupleIncludingFrame(tempFramePoint);
//               arm.setCoMOffset(tempFramePoint);
//            }
//         });
//      }
//
//      for (RobotSide robotSide : RobotSide.values)
//      {
//         FramePoint originalForearmCoMOffset = new FramePoint();
//         final RigidBody forearm = fullRobotModel.getArmJoint(robotSide, ArmJointName.ELBOW_PITCH).getSuccessor(); // TODO Hacked for Valkyrie
//         forearm.packCoMOffset(originalForearmCoMOffset);
//
//         final YoFramePoint forearmCoMOffset = new YoFramePoint(robotSide.getCamelCaseNameForStartOfExpression() + "ForearmCoMOffset", originalForearmCoMOffset.getReferenceFrame(), registry);
//         forearmCoMOffset.set(originalForearmCoMOffset);
//
//         forearmCoMOffset.attachVariableChangedListener(new VariableChangedListener()
//         {
//            @Override
//            public void variableChanged(YoVariable<?> v)
//            {
//               forearmCoMOffset.getFrameTupleIncludingFrame(tempFramePoint);
//               forearm.setCoMOffset(tempFramePoint);
//            }
//         });
//      }

      hipYawOffset = new YoFrameVector("hipYawOffset", null, registry);
      hipYawOffset.attachVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            for (RobotSide robotSide : RobotSide.values)
            {
               OneDoFJoint hipYawJoint = fullRobotModel.getLegJoint(robotSide, LegJointName.HIP_YAW);
               ReferenceFrame frameBeforeJoint = hipYawJoint.getFrameBeforeJoint();

               Transform3D postCorruptionTransform = new Transform3D();
               Vector3d offsetVector = hipYawOffset.getVector3dCopy();
               offsetVector.setY(robotSide.negateIfRightSide(offsetVector.getY()));
               postCorruptionTransform.setTranslation(offsetVector);
               frameBeforeJoint.corruptTransformToParentPostMultiply(postCorruptionTransform);
            }
         }
      });


      // Joint Calibration offset errors:
      RobotSpecificJointNames robotSpecificJointNames = fullRobotModel.getRobotSpecificJointNames();
      LegJointName[] legJointNames = robotSpecificJointNames.getLegJointNames();
      
      for (RobotSide robotSide : RobotSide.values)
      {
         for (LegJointName legJointName : legJointNames)
         {
            RevoluteJoint oneDoFJoint = (RevoluteJoint) fullRobotModel.getLegJoint(robotSide, legJointName);
            final ReferenceFrame frameBeforeJoint = oneDoFJoint.getFrameBeforeJoint();
            final Vector3d jointAxis = oneDoFJoint.getJointAxis().getVectorCopy();

            final Transform3D preCorruptionTransform = new Transform3D();

            final DoubleYoVariable offset = new DoubleYoVariable(robotSide + legJointName.getCamelCaseNameForMiddleOfExpression() + "Offset", registry);

            offset.addVariableChangedListener(new VariableChangedListener()
            {
               @Override
               public void variableChanged(YoVariable<?> v)
               {
                  AxisAngle4d axisAngle = new AxisAngle4d(jointAxis, offset.getDoubleValue());
                  preCorruptionTransform.set(axisAngle);
                  frameBeforeJoint.corruptTransformToParentPreMultiply(preCorruptionTransform);
               }
            });
         }
      }

      SpineJointName[] spineJointNames = robotSpecificJointNames.getSpineJointNames();

      for (SpineJointName spineJointName : spineJointNames)
      {
         RevoluteJoint oneDoFJoint = (RevoluteJoint) fullRobotModel.getSpineJoint(spineJointName);
         final ReferenceFrame frameBeforeJoint = oneDoFJoint.getFrameBeforeJoint();
         final Vector3d jointAxis = oneDoFJoint.getJointAxis().getVectorCopy();

         final Transform3D preCorruptionTransform = new Transform3D();

         final DoubleYoVariable offset = new DoubleYoVariable(spineJointName.getCamelCaseNameForStartOfExpression() + "Offset", registry);

         offset.addVariableChangedListener(new VariableChangedListener()
         {
            @Override
            public void variableChanged(YoVariable<?> v)
            {
               AxisAngle4d axisAngle = new AxisAngle4d(jointAxis, offset.getDoubleValue());
               preCorruptionTransform.set(axisAngle);
               frameBeforeJoint.corruptTransformToParentPreMultiply(preCorruptionTransform);
            }
         });
      }

      parentRegistry.addChild(registry);
   }
}
