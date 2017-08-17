package us.ihmc.robotics.controllers.pidGains;

/**
 * An extension of the {@link PIDSE3Gains} interface that provides additional access
 * to YoVariables in the implementation.
 */
public interface YoPIDSE3Gains extends PIDSE3Gains
{
   @Override
   public abstract YoPID3DGains getPositionGains();

   @Override
   public abstract YoPID3DGains getOrientationGains();
}
