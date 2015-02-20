package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import us.ihmc.utilities.math.geometry.FrameOrientation;
import us.ihmc.utilities.math.geometry.FrameOrientationWaypoint;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePointWaypoint;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.yoUtilities.dataStructure.listener.VariableChangedListener;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;
import us.ihmc.yoUtilities.math.frames.YoFrameOrientation;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;

public class UserDesiredPelvisPoseProvider extends PelvisPoseProvider
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
   public boolean checkForNewPosition()
   {
      return isNewPelvisPositionInformationAvailable.getBooleanValue();
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

   @Override
   public Double getTrajectoryTimeToHome()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public FramePointWaypoint[] getDesiredPelvisPosition(ReferenceFrame desiredReferenceFrame)
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public FrameOrientationWaypoint[] getDesiredPelvisOrientation(ReferenceFrame desiredReferenceFrame)
   {
      if (!isNewPelvisOrientationInformationAvailable.getBooleanValue())
         return null;

      isNewPelvisOrientationInformationAvailable.set(false);

      frameOrientation.setIncludingFrame(desiredReferenceFrame, desiredQuat);

      return new FrameOrientationWaypoint[]{  new FrameOrientationWaypoint( userPelvisTrajectoryTime.getDoubleValue(),frameOrientation ) };
   }


}
