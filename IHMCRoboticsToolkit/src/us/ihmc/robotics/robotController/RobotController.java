package us.ihmc.robotics.robotController;

/**
 * Interface for controlling a robot. doControl() gets called each update.
 * Has a name and a YoVariableRegistry. This registry should not be attached to a 
 * parent before it is added to a robot. It will be added to the robots registry.
 *  
 * Copyright:    Copyright (c) 2000-2011
 * Company:      IHMC and Yobotics, Inc. 
 */
public interface RobotController extends RobotControlElement
{
   public abstract void doControl();
}
