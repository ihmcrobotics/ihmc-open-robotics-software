package us.ihmc.commonWalkingControlModules.corruptors;

import java.util.ArrayList;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Vector3d;

import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.humanoidRobot.partNames.ArmJointName;
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
   private final ArrayList<VariableChangedListener> variableChangedListeners = new ArrayList<VariableChangedListener>();
   
   public FullRobotModelCorruptor(final FullRobotModel fullRobotModel, YoVariableRegistry parentRegistry)
   {
      String chestName = "chest";
      final RigidBody chest = fullRobotModel.getChest();
      createMassAndCoMOffsetCorruptors(chestName, chest);
      
      String pelvisName = "pelvis";
      final RigidBody pelvis = fullRobotModel.getPelvis();
      createMassAndCoMOffsetCorruptors(pelvisName, pelvis);

      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         
         String thighName = sidePrefix + "Thigh";
         final RigidBody thigh = fullRobotModel.getLegJoint(robotSide, LegJointName.KNEE).getPredecessor();
         createMassAndCoMOffsetCorruptors(thighName, thigh);
         
         String shinName = sidePrefix + "Shin";
         final RigidBody shin = fullRobotModel.getLegJoint(robotSide, LegJointName.KNEE).getSuccessor();
         createMassAndCoMOffsetCorruptors(shinName, shin);
         
         String footName = sidePrefix + "Foot";
         final RigidBody foot = fullRobotModel.getFoot(robotSide);
         createMassAndCoMOffsetCorruptors(footName, foot);
     }
      
      
      for (RobotSide robotSide : RobotSide.values)
      {
         String sidePrefix = robotSide.getCamelCaseNameForStartOfExpression();
         
         String upperArmName = sidePrefix + "UpperArm";
         final RigidBody upperArm = fullRobotModel.getArmJoint(robotSide, ArmJointName.ELBOW_PITCH).getPredecessor();
         createMassAndCoMOffsetCorruptors(upperArmName, upperArm);
         
         String lowerArmName = sidePrefix + "LowerArm";
         final RigidBody lowerArm = fullRobotModel.getArmJoint(robotSide, ArmJointName.ELBOW_PITCH).getSuccessor();
         createMassAndCoMOffsetCorruptors(lowerArmName, lowerArm);
         
         String handName = sidePrefix + "Hane";
         final RigidBody hand = fullRobotModel.getHand(robotSide);
         createMassAndCoMOffsetCorruptors(handName, hand);
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


   private void createMassAndCoMOffsetCorruptors(String name, final RigidBody rigidBody)
   {
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
