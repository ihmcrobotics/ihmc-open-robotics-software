package us.ihmc.commonWalkingControlModules.packetConsumers;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;

import us.ihmc.robotics.dataStructures.listener.VariableChangedListener;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class UserDesiredPelvisPoseProvider implements PelvisPoseProvider
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final BooleanYoVariable goToHomePosition = new BooleanYoVariable("userDesiredPelvisGoToHomePosition", registry);
   private final BooleanYoVariable goToHomeOrientation = new BooleanYoVariable("userDesiredPelvisGoToHomeOrientation", registry);

   private final DoubleYoVariable userPelvisTrajectoryTime = new DoubleYoVariable("userDesiredPelvisTrajectoryTime", registry);

   private final BooleanYoVariable isNewPelvisOrientationInformationAvailable = new BooleanYoVariable("isNewPelvisOrientationInformationAvailable", registry);
   private final YoFrameOrientation userPelvisOrientation = new YoFrameOrientation("userDesiredPelvis", null, registry);

   private final BooleanYoVariable isNewPelvisPositionInformationAvailable = new BooleanYoVariable("isNewPelvisPositionInformationAvailable", registry);
   private final YoFramePoint userPelvisPosition = new YoFramePoint("userDesiredPelvis", null, registry);

   private final Quat4d desiredQuat = new Quat4d();
   private final Point3d desiredPoint = new Point3d();
   private final FrameOrientation frameOrientation = new FrameOrientation();
   private final FramePoint framePoint = new FramePoint();

   public UserDesiredPelvisPoseProvider(YoVariableRegistry parentRegistry)
   {
      VariableChangedListener variableChangedListener = new VariableChangedListener()
      {
         public void variableChanged(YoVariable<?> v)
         {
            isNewPelvisOrientationInformationAvailable.set(true);
            userPelvisOrientation.getQuaternion(desiredQuat);
         }
      };

      userPelvisOrientation.attachVariableChangedListener(variableChangedListener);
      
      userPelvisPosition.attachVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            isNewPelvisPositionInformationAvailable.set(true);
            userPelvisPosition.get(desiredPoint);
         }
      });

      goToHomeOrientation.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            if (goToHomeOrientation.getBooleanValue())
               userPelvisOrientation.setYawPitchRoll(0.0, 0.0, 0.0, false);
         }
      });

      goToHomePosition.addVariableChangedListener(new VariableChangedListener()
      {
         @Override
         public void variableChanged(YoVariable<?> v)
         {
            if (goToHomePosition.getBooleanValue())
               userPelvisPosition.setToZero(false);
         }
      });

      parentRegistry.addChild(registry);
   }

   @Override
   public boolean checkForNewOrientation()
   {
      return isNewPelvisOrientationInformationAvailable.getBooleanValue();
   }

   @Override
   public void clearOrientation()
   {
      isNewPelvisOrientationInformationAvailable.set(false);
      goToHomeOrientation.set(false);
   }

   @Override
   public void clearPosition()
   {
      isNewPelvisPositionInformationAvailable.set(false);
      goToHomePosition.set(false);
   }

   @Override
   public FrameOrientation getDesiredPelvisOrientation(ReferenceFrame desiredPelvisFrame)
   {
      if (!isNewPelvisOrientationInformationAvailable.getBooleanValue())
         return null;

      isNewPelvisOrientationInformationAvailable.set(false);

      frameOrientation.setIncludingFrame(desiredPelvisFrame, desiredQuat);
      
      return frameOrientation;
   }

   @Override
   public boolean checkForNewPosition()
   {
      return isNewPelvisPositionInformationAvailable.getBooleanValue();
   }

   @Override
   public FramePoint getDesiredPelvisPosition(ReferenceFrame supportFrame)
   {
      if (!isNewPelvisPositionInformationAvailable.getBooleanValue())
         return null;
      
      isNewPelvisPositionInformationAvailable.set(false);
      
      framePoint.setIncludingFrame(supportFrame, desiredPoint);
      
      return framePoint;
   }

   @Override
   public double getTrajectoryTime()
   {
      return userPelvisTrajectoryTime.getDoubleValue();
   }

   @Override
   public boolean checkForHomePosition()
   {
      if (goToHomePosition.getBooleanValue())
      {
         goToHomePosition.set(false);
         return true;
      }
      return false;
   }

   @Override
   public boolean checkForHomeOrientation()
   {
      if (goToHomeOrientation.getBooleanValue())
      {
         goToHomeOrientation.set(false);
         return true;
      }
      return false;
   }
}
