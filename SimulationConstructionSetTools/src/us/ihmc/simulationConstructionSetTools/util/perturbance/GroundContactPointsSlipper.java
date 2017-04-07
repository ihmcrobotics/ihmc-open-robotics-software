package us.ihmc.simulationConstructionSetTools.util.perturbance;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Robot;

//Used to make ground contact points slip a delta.
public class GroundContactPointsSlipper implements RobotController
{
   private final YoVariableRegistry registry;

   private final ArrayList<GroundContactPoint> groundContactPointsToSlip;
   private final YoFrameVector slipAmount;
   private final YoFrameOrientation slipRotation;

   private final DoubleYoVariable percentToSlipPerTick;
   private final BooleanYoVariable doSlip;

   
   public GroundContactPointsSlipper(String registryPrefix)
   {
      registry = new YoVariableRegistry(registryPrefix + getClass().getSimpleName());

      groundContactPointsToSlip = new ArrayList<GroundContactPoint>();
      slipAmount = new YoFrameVector("slipAmount", ReferenceFrame.getWorldFrame(), registry);
      slipRotation = new YoFrameOrientation("slipRotation", ReferenceFrame.getWorldFrame(), registry);

      percentToSlipPerTick = new DoubleYoVariable("percentToSlipPerTick", registry);
      doSlip = new BooleanYoVariable("doSlip", registry);
   }

   public void addGroundContactPoints(List<GroundContactPoint> footGroundContactPoints)
   {
      for (GroundContactPoint groundContactPoint : footGroundContactPoints)
      {
         addGroundContactPoint(groundContactPoint);
      }
   }

   public void setGroundContactPoints(List<GroundContactPoint> footGroundContactPoints)
   {
      this.groundContactPointsToSlip.clear();

      for (GroundContactPoint groundContactPoint : footGroundContactPoints)
      {
         addGroundContactPoint(groundContactPoint);
      }
   }

   public void addGroundContactPoint(GroundContactPoint groundContactPoint)
   {
      this.groundContactPointsToSlip.add(groundContactPoint);
   }

   public void addGroundContactPoint(Robot robot, String groundContactPointName)
   {
      ArrayList<GroundContactPoint> allGroundContactPoints = robot.getAllGroundContactPoints();

      for (GroundContactPoint groundContactPoint : allGroundContactPoints)
      {
         if (groundContactPoint.getName().equals(groundContactPointName))
         {
            this.addGroundContactPoint(groundContactPoint);

            return;
         }
      }
   }

   public void setDoSlip(boolean doSlip)
   {
      this.doSlip.set(doSlip);
   }

   public void setPercentToSlipPerTick(double percentToSlipPerTick)
   {
      this.percentToSlipPerTick.set(percentToSlipPerTick);
   }

   public void setSlipTranslation(Vector3D slipAmount)
   {
      this.slipAmount.set(slipAmount);
   }
   
   public void setSlipRotationYawPitchRoll(double[] yawPitchRoll) 
   {
      this.slipRotation.setYawPitchRoll(yawPitchRoll[0], yawPitchRoll[1], yawPitchRoll[2]);
   }
   
   public void setSlipRotationYawPitchRoll(double yaw, double pitch, double roll)
   {
      this.slipRotation.setYawPitchRoll(yaw, pitch, roll);
   }
   
   public void setSlipRotationEulerAngles(Vector3D eulerAngles)
   {
      this.slipRotation.setEulerAngles(eulerAngles);
   }
   
   public boolean isDoneSlipping()
   {
      boolean translationalSlipDone = slipAmount.lengthSquared() < 0.0001 * 0.0001;
      
      Vector3D eulerAngles = new Vector3D();
      slipRotation.getEulerAngles(eulerAngles);
      boolean rotationalSlipDone = eulerAngles.lengthSquared() < 0.001 * 0.001;
      
      return translationalSlipDone & rotationalSlipDone;
   }

   public void slipALittle(double percentOfDelta)
   {
      if (percentOfDelta < 0.0)
         return;
      if (percentOfDelta > 1.0)
         return;

      applyTranslationalSlip(percentOfDelta);
      applyRotationalSlip(percentOfDelta);
   }
   
   private void applyTranslationalSlip(double percentOfDelta) 
   {
      FrameVector slipDelta = slipAmount.getFrameVectorCopy();
      slipDelta.scale(percentOfDelta);
      slipAmount.sub(slipDelta);

      Point3D touchdownLocation = new Point3D();

      for (int i = 0; i < groundContactPointsToSlip.size(); i++)
      {
         GroundContactPoint groundContactPointToSlip = groundContactPointsToSlip.get(i);

         boolean touchedDown = (groundContactPointToSlip.isInContact());

         if (touchedDown)
         {
            groundContactPointToSlip.getTouchdownLocation(touchdownLocation);
            touchdownLocation.add(slipDelta.getVectorCopy());
            groundContactPointToSlip.setTouchdownLocation(touchdownLocation);
         }
      }
   }
   
   private void applyRotationalSlip(double percentOfDelta)
   {
      FrameOrientation identity = new FrameOrientation(ReferenceFrame.getWorldFrame());
      FrameOrientation desired = slipRotation.getFrameOrientationCopy();
      FrameOrientation delta = new FrameOrientation();

      delta.interpolate(identity, desired, percentOfDelta);
      
      desired.interpolate(identity, desired, 1.0-percentOfDelta);
      slipRotation.set(desired);

      Point3D touchdownCoM = computeTouchdownCoM();
      RotationMatrix deltaRotation = delta.getMatrix3dCopy();

      Point3D touchdownLocation = new Point3D();

      for (int i = 0; i < groundContactPointsToSlip.size(); i++)
      {
         GroundContactPoint groundContactPointToSlip = groundContactPointsToSlip.get(i);

         boolean touchedDown = (groundContactPointToSlip.isInContact());

         if (touchedDown)
         {
            groundContactPointToSlip.getTouchdownLocation(touchdownLocation);
            touchdownLocation.sub(touchdownCoM);
            deltaRotation.transform(touchdownLocation);
            touchdownLocation.add(touchdownCoM);
            groundContactPointToSlip.setTouchdownLocation(touchdownLocation);
         }
      }
   }

   private Point3D computeTouchdownCoM()
   {
      int touchdownCount = 0;
      Point3D touchdownCoM = new Point3D();
      Point3D touchdownLocation = new Point3D();

      for (int i = 0; i < groundContactPointsToSlip.size(); i++)
      {
         GroundContactPoint groundContactPointToSlip = groundContactPointsToSlip.get(i);

         boolean touchedDown = (groundContactPointToSlip.isInContact());
         if (touchedDown)
         {
            groundContactPointToSlip.getTouchdownLocation(touchdownLocation);
            touchdownCoM.add(touchdownLocation);
            touchdownCount++;
         }
      }

      touchdownCoM.scale(1.0/touchdownCount);
      return touchdownCoM;
   }

   @Override
   public void initialize()
   {
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return registry.getName();
   }

   @Override
   public String getDescription()
   {
      return getName();
   }

   @Override
   public void doControl()
   {
      if (doSlip.getBooleanValue())
      {
         slipALittle(percentToSlipPerTick.getDoubleValue());
      }
   }


}
