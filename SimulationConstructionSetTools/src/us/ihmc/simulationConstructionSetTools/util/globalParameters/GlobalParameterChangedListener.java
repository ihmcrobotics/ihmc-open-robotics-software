package us.ihmc.simulationConstructionSetTools.util.globalParameters;

/**
 * <p>Title: </p>
 *
 * <p>Description: </p>
 *
 * <p>Copyright: Copyright (c) 2006</p>
 *
 * <p>Company: </p>
 *
 * @author not attributable
 * @version 1.0
 */
public interface GlobalParameterChangedListener
{
   public abstract void booleanValueChanged(GlobalParameter globalParameter, String comment, boolean previousValue, boolean newValue);

   public abstract void doubleValueChanged(GlobalParameter globalParameter, String comment, double previousValue, double newValue);

   public abstract void integerValueChanged(GlobalParameter globalParameter, String comment, int previousValue, int newValue);

   public abstract void enumValueChanged(GlobalParameter globalParameter, String comment, Enum<?> previousValue, Enum<?> newValue);

   public abstract void globalParameterCreated(GlobalParameter globalParameter);
}
