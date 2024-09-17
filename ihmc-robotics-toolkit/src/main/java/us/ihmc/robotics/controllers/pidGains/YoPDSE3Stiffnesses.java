package us.ihmc.robotics.controllers.pidGains;

/**
 * An extension of the {@link PDSE3Stiffnesses} interface that provides additional access
 * to YoVariables in the implementation.
 */
public interface YoPDSE3Stiffnesses extends PDSE3Stiffnesses
{
   @Override
   public abstract YoPD3DStiffnesses getPositionStiffnesses();

   @Override
   public abstract YoPD3DStiffnesses getOrientationStiffnesses();
}
