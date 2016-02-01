package us.ihmc.commonWalkingControlModules.optimalSwing;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.EnumMap;
import java.util.List;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.partNames.LegJointName;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.math.frames.YoFrameOrientation;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;


public class LegConfigurationData
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);
   
   
   private final List<LegJointName> jointsToInterpolate;
   private final List<LegJointName> jointsToOptimize;
   private final List<LegJointName> allJoints;
   
   private final EnumYoVariable<RobotSide> robotSide = new EnumYoVariable<RobotSide>("robotSide", registry, RobotSide.class);
   private final BooleanYoVariable currentlyInSwing = new BooleanYoVariable("currentlyInSwing", registry);
   
   private final DoubleYoVariable timeInSwing = new DoubleYoVariable("timeInSwing", registry);
   private final DoubleYoVariable swingTimeRemaining = new DoubleYoVariable("swingTimeRemaining", registry);
   
   
   private final DoubleYoVariable hipRollHeight = new DoubleYoVariable("hipRollHeight", registry);
   
   private final EnumMap<LegJointName, DoubleYoVariable> currentJointAngles = new EnumMap<LegJointName, DoubleYoVariable>(LegJointName.class);
   private final EnumMap<LegJointName, DoubleYoVariable> currentJointVelocities = new EnumMap<LegJointName, DoubleYoVariable>(LegJointName.class);
   
   private final EnumMap<LegJointName, DoubleYoVariable> finalDesiredJointAngles = new EnumMap<LegJointName, DoubleYoVariable>(LegJointName.class);
   private final EnumMap<LegJointName, DoubleYoVariable> finalDesiredJointVelocities = new EnumMap<LegJointName, DoubleYoVariable>(LegJointName.class);
   
   private final YoFramePoint upperBodyPositionInWorld = new YoFramePoint("upperBodyPosition", "InWorld", ReferenceFrame.getWorldFrame(), registry);
   private final YoFrameOrientation upperBodyOrientationInWorld = new YoFrameOrientation("upperBodyOrientation", "InWorld", ReferenceFrame.getWorldFrame(), registry);
   
   private final ArrayList<YoVariable<?>> allVariables;

   public LegConfigurationData(LegJointName[] jointsToInterpolate, LegJointName[] jointsToOptimize, YoVariableRegistry parentRegistry)
   {
      this.jointsToInterpolate = Collections.unmodifiableList(Arrays.asList(jointsToInterpolate));
      this.jointsToOptimize = Collections.unmodifiableList(Arrays.asList(jointsToOptimize));
      
      ArrayList<LegJointName> allJointNames = new ArrayList<LegJointName>();
      allJointNames.addAll(Arrays.asList(jointsToInterpolate));
      allJointNames.addAll(Arrays.asList(jointsToOptimize));
      this.allJoints = Collections.unmodifiableList(allJointNames);
      
      
      for(LegJointName jointName : allJointNames)
      {
         currentJointAngles.put(jointName, new DoubleYoVariable("current"+jointName.getCamelCaseNameForMiddleOfExpression()+"Angle", registry));
         currentJointVelocities.put(jointName, new DoubleYoVariable("current"+jointName.getCamelCaseNameForMiddleOfExpression()+"Velocity", registry));
         
         finalDesiredJointAngles.put(jointName, new DoubleYoVariable("finalDesired"+jointName.getCamelCaseNameForMiddleOfExpression()+"Angle", registry));
         finalDesiredJointVelocities.put(jointName, new DoubleYoVariable("finalDesired"+jointName.getCamelCaseNameForMiddleOfExpression()+"Velocity", registry));
         
      }
      
      if(parentRegistry != null)
      {
         parentRegistry.addChild(registry);
      }
      
      allVariables = registry.getAllVariablesIncludingDescendants();

   }
   
   public RobotSide getRobotSide()
   {
      return robotSide.getEnumValue();
   }
   
   public void setRobotSide(RobotSide robotSide)
   {
      this.robotSide.set(robotSide);
   }
   
   public double getSwingTimeRemaining()
   {
      return swingTimeRemaining.getDoubleValue();
   }
   
   public void setSwingTimeRemaining(double swingTimeRemaining)
   {
      this.swingTimeRemaining.set(swingTimeRemaining);
   }
   
   public boolean iscurrentlyInSwing()
   {
      return currentlyInSwing.getBooleanValue();
   }
   
   public void setcurrentlyInSwing(boolean initializeSwing)
   {
      this.currentlyInSwing.set(initializeSwing);
   }
   
   public double getCurrentJointAngle(LegJointName jointName)
   {
      return currentJointAngles.get(jointName).getDoubleValue();
   }
   
   public void setCurrentJointAngle(LegJointName jointName, double value)
   {
      currentJointAngles.get(jointName).set(value);
   }
   
   
   public double getCurrentJointVelocity(LegJointName jointName)
   {
      return currentJointVelocities.get(jointName).getDoubleValue();
   }
   
   public void setCurrentJointVelocity(LegJointName jointName, double value)
   {
      currentJointVelocities.get(jointName).set(value);
   }
   
   
   public double getFinalDesiredJointAngle(LegJointName jointName)
   {
      return finalDesiredJointAngles.get(jointName).getDoubleValue();
   }
   
   public void setFinalDesiredJointAngle(LegJointName jointName, double value)
   {
      finalDesiredJointAngles.get(jointName).set(value);
   }
   
   
   public double getFinalDesiredJointVelocity(LegJointName jointName)
   {
      return finalDesiredJointVelocities.get(jointName).getDoubleValue();
   }
   
   public void setFinalDesiredJointVelocity(LegJointName jointName, double value)
   {
      finalDesiredJointVelocities.get(jointName).set(value);
   }
   
   public List<LegJointName> getJointsToOptimize()
   {
      return jointsToOptimize;
   }
   
   public List<LegJointName> getJointsToInterpolate()
   {
      return jointsToInterpolate;
   }
   
   public List<LegJointName> getAllJoints()
   {
      return allJoints;
   }
   
   public int getNumberOfJointsToOptimize()
   {
      return jointsToOptimize.size();
   }
   
   public int getNumberOfJointsToInterpolate()
   {
      return jointsToInterpolate.size();
   }
   
   public LegJointName getJointToOptimize(int index)
   {
      return jointsToOptimize.get(index);
   }
   
   public LegJointName getJointToInterpolate(int index)
   {
      return jointsToInterpolate.get(index);
   }
   
   public ArrayList<YoVariable<?>> getAllVariables()
   {
      return allVariables;
   }

   public FrameOrientation getUpperBodyOrientationInWorld()
   {
      return upperBodyOrientationInWorld.getFrameOrientationCopy();
   }
   
   public void setUpperBodyOrientationInWorld(FrameOrientation orientation)
   {
      this.upperBodyOrientationInWorld.set(orientation);
   }

   public double getHipRollHeight()
   {
      return hipRollHeight.getDoubleValue();
   }
   
   public void setHipRollHeight(double value)
   {
      hipRollHeight.set(value);
   }
   
   public double getTimeInSwing()
   {
      return timeInSwing.getDoubleValue();
   }

   
   public void setTimeInSwing(double time)
   {
      timeInSwing.set(time);
   }

   public FramePoint getUpperBodyPositionInWorld()
   {
      return upperBodyPositionInWorld.getFramePointCopy();
   }

   public void setUpperBodyPositionInWorld(Vector3d upperBodyPositionInWorld)
   {
      Point3d upperBodyPostion = new Point3d(upperBodyPositionInWorld);
      this.upperBodyPositionInWorld.set(upperBodyPostion);
   }
}
