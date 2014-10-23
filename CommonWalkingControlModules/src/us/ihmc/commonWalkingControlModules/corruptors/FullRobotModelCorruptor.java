package us.ihmc.commonWalkingControlModules.corruptors;

import java.util.ArrayList;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Vector3d;

import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.humanoidRobot.partNames.ArmJointName;
import us.ihmc.utilities.humanoidRobot.partNames.LegJointName;
import us.ihmc.utilities.humanoidRobot.partNames.RobotSpecificJointNames;
import us.ihmc.utilities.humanoidRobot.partNames.SpineJointName;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
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
         OneDoFJoint elbowPitchJoint = fullRobotModel.getArmJoint(robotSide, ArmJointName.ELBOW_PITCH);
         if (elbowPitchJoint != null)
         {
            RigidBody upperArm = elbowPitchJoint.getPredecessor();
            createMassAndCoMOffsetCorruptors(upperArmName, upperArm);

            String lowerArmName = sidePrefix + "LowerArm";
            final RigidBody lowerArm = elbowPitchJoint.getSuccessor();
            createMassAndCoMOffsetCorruptors(lowerArmName, lowerArm);
         }

         String handName = sidePrefix + "Hand";
         final RigidBody hand = fullRobotModel.getHand(robotSide);
         if (hand != null) createMassAndCoMOffsetCorruptors(handName, hand);
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
      RobotSpecificJointNames robotSpecificJointNames = fullRobotModel.getRobotSpecificJointNames();
      LegJointName[] legJointNames = robotSpecificJointNames.getLegJointNames();
      ArmJointName[] armJointNames = robotSpecificJointNames.getArmJointNames();
      SpineJointName[] spineJointNames = robotSpecificJointNames.getSpineJointNames();
      
      for (RobotSide robotSide : RobotSide.values)
      {
         for (LegJointName legJointName : legJointNames)
         {
            RevoluteJoint oneDoFJoint = (RevoluteJoint) fullRobotModel.getLegJoint(robotSide, legJointName);
            String offsetName = robotSide + legJointName.getCamelCaseNameForMiddleOfExpression() + "Offset";

            createJointAngleCorruptor(oneDoFJoint, offsetName);
         }
         
         for (ArmJointName armJointName : armJointNames)
         {
            RevoluteJoint oneDoFJoint = (RevoluteJoint) fullRobotModel.getArmJoint(robotSide, armJointName);
            String offsetName = robotSide + armJointName.getCamelCaseNameForMiddleOfExpression() + "Offset";

            createJointAngleCorruptor(oneDoFJoint, offsetName);
         }
      }

      for (SpineJointName spineJointName : spineJointNames)
      {
         RevoluteJoint oneDoFJoint = (RevoluteJoint) fullRobotModel.getSpineJoint(spineJointName);
         String offsetName = spineJointName.getCamelCaseNameForStartOfExpression() + "Offset";

         createJointAngleCorruptor(oneDoFJoint, offsetName);
      }

      parentRegistry.addChild(registry);
   }


   private void createJointAngleCorruptor(RevoluteJoint oneDoFJoint, String offsetName)
   {
      final ReferenceFrame frameBeforeJoint = oneDoFJoint.getFrameBeforeJoint();
      final Vector3d jointAxis = oneDoFJoint.getJointAxis().getVectorCopy();

      final RigidBodyTransform preCorruptionTransform = new RigidBodyTransform();

      final DoubleYoVariable offset = new DoubleYoVariable(offsetName, registry);

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
