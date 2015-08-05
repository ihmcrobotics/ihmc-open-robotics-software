package us.ihmc.commonWalkingControlModules.optimalSwing;

import java.util.ArrayList;
import java.util.EnumMap;

import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointPositions;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegJointVelocities;
import us.ihmc.humanoidRobotics.partNames.LegJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.YoVariable;


public class SwingParameters
{
   private final YoVariableRegistry registry = new YoVariableRegistry("SwingParameters");
   private final BooleanYoVariable currentlyInSwing = new BooleanYoVariable("currentlyInSwing", registry);
   private final EnumYoVariable<RobotSide> robotSide = new EnumYoVariable<RobotSide>("robotSide", registry, RobotSide.class);
   private final DoubleYoVariable totalSwingTime = new DoubleYoVariable("totalSwingTime", registry);
   private final DoubleYoVariable swingTimeRemaining = new DoubleYoVariable("swingTimeRemaining", registry);
   private final SideDependentList<EnumMap<LegJointName, DoubleYoVariable>> desiredJointPositionsAtEndOfStep = new SideDependentList<EnumMap<LegJointName, DoubleYoVariable>>();
   private final SideDependentList<EnumMap<LegJointName, DoubleYoVariable>> desiredJointVelocitiesAtEndOfStep = new SideDependentList<EnumMap<LegJointName, DoubleYoVariable>>();

   //   private final DoubleYoVariable desiredStepLocationInHipYawFrameX = new DoubleYoVariable("desiredStepLocationInHipYawFrameX", registry);
   //   private final DoubleYoVariable desiredStepLocationInHipYawFrameY = new DoubleYoVariable("desiredStepLocationInHipYawFrameY", registry);
   //   private final DoubleYoVariable desiredStepLocationInHipYawFrameZ = new DoubleYoVariable("desiredStepLocationInHipYawFrameZ", registry);
   //   
   //   private final DoubleYoVariable currentFootLocationInHipYawFrameX = new DoubleYoVariable("currentFootLocationInHipYawFrameX", registry);
   //   private final DoubleYoVariable currentFootLocationInHipYawFrameY = new DoubleYoVariable("currentFootLocationInHipYawFrameY", registry);
   //   private final DoubleYoVariable currentFootLocationInHipYawFrameZ = new DoubleYoVariable("currentFootLocationInHipYawFrameZ", registry);
   //   
   //   private final DoubleYoVariable currentFootVelocityInHipYawFrameX = new DoubleYoVariable("currentFootVelocityInHipYawFrameX", registry);
   //   private final DoubleYoVariable currentFootVelocityInHipYawFrameY = new DoubleYoVariable("currentFootVelocityInHipYawFrameY", registry);
   //   private final DoubleYoVariable currentFootVelocityInHipYawFrameZ = new DoubleYoVariable("currentFootVelocityInHipYawFrameZ", registry);
   //
   //   
   //   private final DoubleYoVariable hipHeight = new DoubleYoVariable("hipHeight", registry);

   private final LegJointName[] legJointNames;

   private final ArrayList<YoVariable<?>> allVariables;

   public SwingParameters(LegJointName[] legJointNames, YoVariableRegistry parentRegistry)
   {
      this.legJointNames = legJointNames;

      for (RobotSide robotSide : RobotSide.values)
      {
         desiredJointPositionsAtEndOfStep.put(robotSide, new EnumMap<LegJointName, DoubleYoVariable>(LegJointName.class));
         desiredJointVelocitiesAtEndOfStep.put(robotSide, new EnumMap<LegJointName, DoubleYoVariable>(LegJointName.class));

         for (LegJointName legJointName : legJointNames)
         {
            desiredJointPositionsAtEndOfStep.get(robotSide).put(
                  legJointName,
                  new DoubleYoVariable("desired" + robotSide.getCamelCaseNameForMiddleOfExpression() + legJointName.getCamelCaseNameForMiddleOfExpression()
                        + "Position", registry));
            desiredJointVelocitiesAtEndOfStep.get(robotSide).put(
                  legJointName,
                  new DoubleYoVariable("desired" + robotSide.getCamelCaseNameForMiddleOfExpression() + legJointName.getCamelCaseNameForMiddleOfExpression()
                        + "Velocity", registry));
         }

      }

      allVariables = registry.getAllVariablesIncludingDescendants();
      if (parentRegistry != null)
      {
         parentRegistry.addChild(registry);
      }
   }

   public double getSwingTimeRemaining()
   {
      return swingTimeRemaining.getDoubleValue();
   }

   public void setSwingTimeRemaining(double swingTimeRemaining)
   {
      this.swingTimeRemaining.set(swingTimeRemaining);
   }

   public RobotSide getRobotSide()
   {
      return robotSide.getEnumValue();
   }

   public void setRobotSide(RobotSide robotSide)
   {
      this.robotSide.set(robotSide);
   }

   public void setDesiredJointPosition(RobotSide robotSide, LegJointName jointName, double value)
   {
      desiredJointPositionsAtEndOfStep.get(robotSide).get(jointName).set(value);
   }

   public double getDesiredJointPosition(RobotSide robotSide, LegJointName jointName)
   {
      return desiredJointPositionsAtEndOfStep.get(robotSide).get(jointName).getDoubleValue();
   }

   public void getDesiredJointPositions(LegJointPositions legJointPositionsToPack)
   {
      RobotSide robotSide = legJointPositionsToPack.getRobotSide();
      for (LegJointName jointName : legJointNames)
      {
         legJointPositionsToPack.setJointPosition(jointName, desiredJointPositionsAtEndOfStep.get(robotSide).get(jointName).getDoubleValue());
      }
   }

   public void getDesiredJointVelocities(LegJointVelocities legJointVelocitiesToPack)
   {
      RobotSide robotSide = legJointVelocitiesToPack.getRobotSide();
      for (LegJointName jointName : legJointNames)
      {
         legJointVelocitiesToPack.setJointVelocity(jointName, desiredJointVelocitiesAtEndOfStep.get(robotSide).get(jointName).getDoubleValue());
      }
   }

   public void setDesiredJointVelocity(RobotSide robotSide, LegJointName jointName, double value)
   {
      desiredJointVelocitiesAtEndOfStep.get(robotSide).get(jointName).set(value);
   }

   public double getDesiredJointVelocity(RobotSide robotSide, LegJointName jointName)
   {
      return desiredJointVelocitiesAtEndOfStep.get(robotSide).get(jointName).getDoubleValue();
   }

   public ArrayList<YoVariable<?>> getAllVariables()
   {
      return allVariables;
   }

   public void setCurrentlyInSwing(boolean currentlyInSwing)
   {
      this.currentlyInSwing.set(currentlyInSwing);
   }

   public boolean isCurrentlyInSwing()
   {
      return currentlyInSwing.getBooleanValue();
   }
   
   public void setTotalSwingTime(double value)
   {
      totalSwingTime.set(value);
   }
   
   public double getTotalSwingTime()
   {
      return totalSwingTime.getDoubleValue();
   }

   //   public void setDesiredStepLocationInHipYawFrame(Point3d point)
   //   {
   //      desiredStepLocationInHipYawFrameX.set(point.x);
   //      desiredStepLocationInHipYawFrameY.set(point.y);
   //      desiredStepLocationInHipYawFrameZ.set(point.z);
   //   }
   //   
   //   public Point3d getDesiredStepLocation()
   //   {
   //      return new Point3d(desiredStepLocationInHipYawFrameX.getDoubleValue(), desiredStepLocationInHipYawFrameY.getDoubleValue(), desiredStepLocationInHipYawFrameZ.getDoubleValue());
   //   }

   //   public void setcurrentFootLocation(Point3d point)
   //   {
   //      currentFootLocationInHipYawFrameX.set(point.x);
   //      currentFootLocationInHipYawFrameY.set(point.y);
   //      currentFootLocationInHipYawFrameZ.set(point.z);
   //   }
   //   
   //   public Point3d getCurrentFootLocation()
   //   {
   //      return new Point3d(currentFootLocationInHipYawFrameX.getDoubleValue(), currentFootLocationInHipYawFrameY.getDoubleValue(), currentFootLocationInHipYawFrameZ.getDoubleValue());
   //   }
   //   
   //   public void setcurrentFootVelocity(Vector3d vector)
   //   {
   //      currentFootVelocityInHipYawFrameX.set(vector.x);
   //      currentFootVelocityInHipYawFrameY.set(vector.y);
   //      currentFootVelocityInHipYawFrameZ.set(vector.z);
   //   }
   //   
   //   public Vector3d getCurrentFootVelocity()
   //   {
   //      return new Vector3d(currentFootVelocityInHipYawFrameX.getDoubleValue(), currentFootVelocityInHipYawFrameY.getDoubleValue(), currentFootVelocityInHipYawFrameZ.getDoubleValue());
   //   }
   //   
   //   public void setHipHeight(double val)
   //   {
   //      hipHeight.set(val);
   //   }
   //   
   //   public double getHipHeight()
   //   {
   //      return hipHeight.getDoubleValue();
   //   }

}
