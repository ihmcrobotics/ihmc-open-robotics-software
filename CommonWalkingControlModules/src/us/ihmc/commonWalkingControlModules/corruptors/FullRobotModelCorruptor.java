package us.ihmc.commonWalkingControlModules.corruptors;

import java.util.ArrayList;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Vector3d;

import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.humanoidRobot.partNames.LegJointName;
import us.ihmc.utilities.humanoidRobot.partNames.RobotSpecificJointNames;
import us.ihmc.utilities.humanoidRobot.partNames.SpineJointName;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.Transform3d;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.RevoluteJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.yoUtilities.dataStructure.listener.VariableChangedListener;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;


public class FullRobotModelCorruptor
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoFramePoint chestCoMOffset;
   private final DoubleYoVariable chestMass;
   private final YoFrameVector hipYawOffset;

   private final FramePoint tempFramePoint = new FramePoint();
   private final ArrayList<VariableChangedListener> variableChangedListeners = new ArrayList<VariableChangedListener>();
   
   public FullRobotModelCorruptor(final FullRobotModel fullRobotModel, YoVariableRegistry parentRegistry)
   {
      FramePoint originalChestCoMOffset = new FramePoint();
      final RigidBody chest = fullRobotModel.getChest();
      chest.packCoMOffset(originalChestCoMOffset);

      chestCoMOffset = new YoFramePoint("chestCoMOffset", originalChestCoMOffset.getReferenceFrame(), registry);
      chestCoMOffset.set(originalChestCoMOffset);

      VariableChangedListener chestCoMOffsetChangedListener = new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            chestCoMOffset.getFrameTupleIncludingFrame(tempFramePoint);
            chest.setCoMOffset(tempFramePoint);
         }
      };
      chestCoMOffset.attachVariableChangedListener(chestCoMOffsetChangedListener);
      variableChangedListeners.add(chestCoMOffsetChangedListener);

      chestMass = new DoubleYoVariable("chestMass", registry);
      chestMass.set(chest.getInertia().getMass());

      VariableChangedListener chestMassChangedListener = new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            RigidBody chest = fullRobotModel.getChest();
            chest.getInertia().setMass(chestMass.getDoubleValue());
         }
      };
      chestMass.addVariableChangedListener(chestMassChangedListener);
      variableChangedListeners.add(chestMassChangedListener);


      //Thighs
      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         final DoubleYoVariable thighMass = new DoubleYoVariable(sidePrefix + "ThighMass", registry);
         final RigidBody thigh = fullRobotModel.getLegJoint(robotSide, LegJointName.KNEE).getPredecessor();
         thighMass.set(thigh.getInertia().getMass());
         
         VariableChangedListener thighMassChangedListener = new VariableChangedListener()
         {
            @Override
            public void variableChanged(YoVariable<?> v)
            {
//               System.out.println("Thigh mass changed to " + thighMass.getDoubleValue());
               thigh.getInertia().setMass(thighMass.getDoubleValue());
            }
         };
         thighMass.addVariableChangedListener(thighMassChangedListener);
         variableChangedListeners.add(thighMassChangedListener);
         
         FramePoint originalThighCoMOffset = new FramePoint();
         thigh.packCoMOffset(originalThighCoMOffset);
         final YoFramePoint thighCoMOffset = new YoFramePoint(sidePrefix + "ThighCoMOffset", originalThighCoMOffset.getReferenceFrame(), registry);
         thighCoMOffset.set(originalThighCoMOffset);
         
         VariableChangedListener thighCoMOffsetChangedListener = new VariableChangedListener()
         {
            private final FramePoint tempFramePoint = new FramePoint();
            @Override
            public void variableChanged(YoVariable<?> v)
            {
               thighCoMOffset.getFrameTupleIncludingFrame(tempFramePoint);
               thigh.setCoMOffset(tempFramePoint);
            }
         };
         thighCoMOffset.attachVariableChangedListener(thighCoMOffsetChangedListener);
         variableChangedListeners.add(thighCoMOffsetChangedListener);

     }

      //Shins
      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         final DoubleYoVariable shinMass = new DoubleYoVariable(sidePrefix + "ShinMass", registry);
         final RigidBody shin = fullRobotModel.getLegJoint(robotSide, LegJointName.KNEE).getSuccessor();
         shinMass.set(shin.getInertia().getMass());
         
         VariableChangedListener shinMassChangedListener = new VariableChangedListener()
         {
            @Override
            public void variableChanged(YoVariable<?> v)
            {
               shin.getInertia().setMass(shinMass.getDoubleValue());
            }
         };
         shinMass.addVariableChangedListener(shinMassChangedListener);
         variableChangedListeners.add(shinMassChangedListener);

         FramePoint originalShinCoMOffset = new FramePoint();
         shin.packCoMOffset(originalShinCoMOffset);
         final YoFramePoint shinCoMOffset = new YoFramePoint(sidePrefix + "ShinCoMOffset", originalShinCoMOffset.getReferenceFrame(), registry);
         shinCoMOffset.set(originalShinCoMOffset);
         
         VariableChangedListener shinCoMOffsetChangedListener = new VariableChangedListener()
         {
            private final FramePoint tempFramePoint = new FramePoint();
            @Override
            public void variableChanged(YoVariable<?> v)
            {
               shinCoMOffset.getFrameTupleIncludingFrame(tempFramePoint);
               shin.setCoMOffset(tempFramePoint);
            }
         };
         shinCoMOffset.attachVariableChangedListener(shinCoMOffsetChangedListener);
         variableChangedListeners.add(shinCoMOffsetChangedListener);

     }
      
      
      // Feet
      
      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         final DoubleYoVariable footMass = new DoubleYoVariable(sidePrefix + "FootMass", registry);
         final RigidBody foot = fullRobotModel.getFoot(robotSide);
         footMass.set(foot.getInertia().getMass());
         
         VariableChangedListener footMassChangedListener = new VariableChangedListener()
         {
            @Override
            public void variableChanged(YoVariable<?> v)
            {
               foot.getInertia().setMass(footMass.getDoubleValue());
            }
         };
         footMass.addVariableChangedListener(footMassChangedListener);
         variableChangedListeners.add(footMassChangedListener);

         
         FramePoint originalFootCoMOffset = new FramePoint();
         foot.packCoMOffset(originalFootCoMOffset);
         final YoFramePoint footCoMOffset = new YoFramePoint(sidePrefix + "FootCoMOffset", originalFootCoMOffset.getReferenceFrame(), registry);
         footCoMOffset.set(originalFootCoMOffset);
         
         VariableChangedListener footCoMOffsetChangedListener = new VariableChangedListener()
         {
            private final FramePoint tempFramePoint = new FramePoint();
            @Override
            public void variableChanged(YoVariable<?> v)
            {
               footCoMOffset.getFrameTupleIncludingFrame(tempFramePoint);
               foot.setCoMOffset(tempFramePoint);
            }
         };
         footCoMOffset.attachVariableChangedListener(footCoMOffsetChangedListener);
         variableChangedListeners.add(footCoMOffsetChangedListener);
     }
      
      
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
      VariableChangedListener hipYawOffsetChangedListener = new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            for (RobotSide robotSide : RobotSide.values)
            {
               OneDoFJoint hipYawJoint = fullRobotModel.getLegJoint(robotSide, LegJointName.HIP_YAW);
               ReferenceFrame frameBeforeJoint = hipYawJoint.getFrameBeforeJoint();

               Transform3d postCorruptionTransform = new Transform3d();
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
      RobotSpecificJointNames robotSpecificJointNames = fullRobotModel.getRobotSpecificJointNames();
      LegJointName[] legJointNames = robotSpecificJointNames.getLegJointNames();
      
      for (RobotSide robotSide : RobotSide.values)
      {
         for (LegJointName legJointName : legJointNames)
         {
            RevoluteJoint oneDoFJoint = (RevoluteJoint) fullRobotModel.getLegJoint(robotSide, legJointName);
            final ReferenceFrame frameBeforeJoint = oneDoFJoint.getFrameBeforeJoint();
            final Vector3d jointAxis = oneDoFJoint.getJointAxis().getVectorCopy();

            final Transform3d preCorruptionTransform = new Transform3d();

            final DoubleYoVariable offset = new DoubleYoVariable(robotSide + legJointName.getCamelCaseNameForMiddleOfExpression() + "Offset", registry);

            VariableChangedListener legJointOffsetChangedListener = new VariableChangedListener()
            {
               @Override
               public void variableChanged(YoVariable<?> v)
               {
                  AxisAngle4d axisAngle = new AxisAngle4d(jointAxis, offset.getDoubleValue());
                  preCorruptionTransform.set(axisAngle);
                  frameBeforeJoint.corruptTransformToParentPreMultiply(preCorruptionTransform);
               }
            };
            offset.addVariableChangedListener(legJointOffsetChangedListener);
            variableChangedListeners.add(legJointOffsetChangedListener);
         }
      }

      SpineJointName[] spineJointNames = robotSpecificJointNames.getSpineJointNames();

      for (SpineJointName spineJointName : spineJointNames)
      {
         RevoluteJoint oneDoFJoint = (RevoluteJoint) fullRobotModel.getSpineJoint(spineJointName);
         final ReferenceFrame frameBeforeJoint = oneDoFJoint.getFrameBeforeJoint();
         final Vector3d jointAxis = oneDoFJoint.getJointAxis().getVectorCopy();

         final Transform3d preCorruptionTransform = new Transform3d();

         final DoubleYoVariable offset = new DoubleYoVariable(spineJointName.getCamelCaseNameForStartOfExpression() + "Offset", registry);

         VariableChangedListener spineJointOffsetChangedListener = new VariableChangedListener()
         {
            @Override
            public void variableChanged(YoVariable<?> v)
            {
               AxisAngle4d axisAngle = new AxisAngle4d(jointAxis, offset.getDoubleValue());
               preCorruptionTransform.set(axisAngle);
               frameBeforeJoint.corruptTransformToParentPreMultiply(preCorruptionTransform);
            }
         };
         offset.addVariableChangedListener(spineJointOffsetChangedListener);
         variableChangedListeners.add(spineJointOffsetChangedListener);
      }

      parentRegistry.addChild(registry);
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
