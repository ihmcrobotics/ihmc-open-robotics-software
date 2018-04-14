package us.ihmc.graphicsDescription.yoGraphics;

import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoVariable;

/**
 * This interface is applied to YoGraphics and Artifacts which support
 * being packed into YoVariables and sent from simulation to viewer.
 *
 * See YoGraphicFactory and YoVariableHandshakeServer
 *
 * @author Alex Lesman
 *
 */
public interface RemoteYoGraphic
{
   public String getName();

   public YoVariable<?>[] getVariables();

   public double[] getConstants();

   public AppearanceDefinition getAppearance();

   public RemoteYoGraphic duplicate(YoVariableRegistry newRegistry);
}
