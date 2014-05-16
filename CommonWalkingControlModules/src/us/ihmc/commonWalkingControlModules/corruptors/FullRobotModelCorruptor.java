package us.ihmc.commonWalkingControlModules.corruptors;

import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.screwTheory.RigidBody;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.VariableChangedListener;
import com.yobotics.simulationconstructionset.YoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;

public class FullRobotModelCorruptor
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final YoFramePoint chestCoMOffset;
   private final DoubleYoVariable chestMass;
   private final FramePoint tempFramePoint = new FramePoint();

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

      parentRegistry.addChild(registry);
   }
}