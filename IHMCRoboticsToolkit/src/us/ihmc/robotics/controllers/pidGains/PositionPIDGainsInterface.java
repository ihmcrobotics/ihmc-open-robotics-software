package us.ihmc.robotics.controllers.pidGains;

import us.ihmc.robotics.controllers.TangentialDampingGains;

public interface PositionPIDGainsInterface extends PID3DGains
{
   public abstract TangentialDampingGains getTangentialDampingGains();
}