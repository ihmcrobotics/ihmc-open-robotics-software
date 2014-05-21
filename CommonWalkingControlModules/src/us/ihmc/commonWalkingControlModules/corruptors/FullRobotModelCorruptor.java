package us.ihmc.commonWalkingControlModules.corruptors;

import javax.media.j3d.Transform3D;
import javax.vecmath.Vector3d;

import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.humanoidRobot.partNames.LegJointName;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.RigidBody;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.VariableChangedListener;
import com.yobotics.simulationconstructionset.YoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;

public class FullRobotModelCorruptor
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoFramePoint chestCoMOffset;
   private final DoubleYoVariable chestMass;
   private final FramePoint tempFramePoint = new FramePoint();

   private final YoFrameVector hipYawOffset;

   
   public FullRobotModelCorruptor(final FullRobotModel fullRobotModel, YoVariableRegistry parentRegistry)
   {
      FramePoint originalChestCoMOffset = new FramePoint();
      RigidBody chest = fullRobotModel.getChest();
      chest.packCoMOffset(originalChestCoMOffset);

      chestCoMOffset = new YoFramePoint("chestCoMOffset", originalChestCoMOffset.getReferenceFrame(), registry);
      chestCoMOffset.set(originalChestCoMOffset);

      chestCoMOffset.attachVariableChangedListener(new VariableChangedListener()
      {

         @Override
         public void variableChanged(YoVariable<?> v)
         {
            RigidBody chest = fullRobotModel.getChest();
            tempFramePoint.setToZero(chestCoMOffset.getReferenceFrame());
            
            chestCoMOffset.getFrameTuple(tempFramePoint);

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


      parentRegistry.addChild(registry);
   }
}