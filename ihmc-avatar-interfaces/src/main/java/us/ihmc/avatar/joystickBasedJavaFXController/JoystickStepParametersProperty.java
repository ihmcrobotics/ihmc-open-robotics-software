package us.ihmc.avatar.joystickBasedJavaFXController;

import java.util.HashMap;
import java.util.Map;

import javafx.beans.property.Property;
import us.ihmc.avatar.joystickBasedJavaFXController.JoystickStepParametersProperty.JoystickStepParameters;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.robotEnvironmentAwareness.ui.properties.ParametersProperty;

public class JoystickStepParametersProperty extends ParametersProperty<JoystickStepParameters>
{
   private final DoubleField swingHeight = new DoubleField(JoystickStepParameters::getSwingHeight, JoystickStepParameters::setSwingHeight);
   private final DoubleField swingDuration = new DoubleField(JoystickStepParameters::getSwingDuration, JoystickStepParameters::setSwingDuration);
   private final DoubleField transferDuration = new DoubleField(JoystickStepParameters::getTransferDuration, JoystickStepParameters::setTransferDuration);
   private final DoubleField maxStepLength = new DoubleField(JoystickStepParameters::getMaxStepLength, JoystickStepParameters::setMaxStepLength);
   private final DoubleField defaultStepWidth = new DoubleField(JoystickStepParameters::getDefaultStepWidth, JoystickStepParameters::setDefaultStepWidth);
   private final DoubleField minStepWidth = new DoubleField(JoystickStepParameters::getMinStepWidth, JoystickStepParameters::setMinStepWidth);
   private final DoubleField maxStepWidth = new DoubleField(JoystickStepParameters::getMaxStepWidth, JoystickStepParameters::setMaxStepWidth);
   private final DoubleField turnStepWidth = new DoubleField(JoystickStepParameters::getTurnStepWidth, JoystickStepParameters::setTurnStepWidth);
   private final DoubleField turnMaxAngleInward = new DoubleField(JoystickStepParameters::getTurnMaxAngleInward, JoystickStepParameters::setTurnMaxAngleInward);
   private final DoubleField turnMaxAngleOutward = new DoubleField(JoystickStepParameters::getTurnMaxAngleOutward,
                                                                   JoystickStepParameters::setTurnMaxAngleOutward);

   public JoystickStepParametersProperty(Object bean, String name)
   {
      super(bean, name, new JoystickStepParameters());
   }

   public void bindBidirectionalSwingHeight(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, swingHeight);
   }

   public void bindBidirectionalSwingDuration(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, swingDuration);
   }

   public void bindBidirectionalTransferDuration(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, transferDuration);
   }

   public void bindBidirectionalMaxStepLength(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, maxStepLength);
   }

   public void bindBidirectionalDefaultStepWidth(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, defaultStepWidth);
   }

   public void bindBidirectionalMinStepWidth(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, minStepWidth);
   }

   public void bindBidirectionalMaxStepWidth(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, maxStepWidth);
   }

   public void bindBidirectionalTurnStepWidth(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, turnStepWidth);
   }

   public void bindBidirectionalTurnMaxAngleInward(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, turnMaxAngleInward);
   }

   public void bindBidirectionalTurnMaxAngleOutward(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, turnMaxAngleOutward);
   }

   @Override
   public JoystickStepParameters getValue()
   {
      return super.getValue();
   }

   @Override
   protected JoystickStepParameters getValueCopy(JoystickStepParameters valueToCopy)
   {
      return new JoystickStepParameters(valueToCopy);
   }

   public static class JoystickStepParameters implements Settable<JoystickStepParameters>
   {
      private double swingHeight;
      private double swingDuration, transferDuration;
      private double maxStepLength;
      private double defaultStepWidth, minStepWidth, maxStepWidth;
      private double turnStepWidth, turnMaxAngleInward, turnMaxAngleOutward;

      public JoystickStepParameters()
      {
      }

      public JoystickStepParameters(WalkingControllerParameters walkingControllerParameters)
      {
         set(walkingControllerParameters);
      }

      public JoystickStepParameters(JoystickStepParameters other)
      {
         set(other);
      }

      public void set(WalkingControllerParameters walkingControllerParameters)
      {
         swingDuration = walkingControllerParameters.getDefaultSwingTime();
         transferDuration = walkingControllerParameters.getDefaultTransferTime();

         SteppingParameters steppingParameters = walkingControllerParameters.getSteppingParameters();
         swingHeight = steppingParameters.getMinSwingHeightFromStanceFoot();
         maxStepLength = steppingParameters.getMaxStepLength();
         defaultStepWidth = steppingParameters.getInPlaceWidth();
         minStepWidth = steppingParameters.getMinStepWidth();
         maxStepWidth = steppingParameters.getMaxStepWidth();
         turnStepWidth = steppingParameters.getTurningStepWidth();
         turnMaxAngleInward = steppingParameters.getMaxAngleTurnInwards();
         turnMaxAngleOutward = steppingParameters.getMaxAngleTurnOutwards();
      }

      @Override
      public void set(JoystickStepParameters other)
      {
         swingHeight = other.swingHeight;
         swingDuration = other.swingDuration;
         transferDuration = other.transferDuration;
         maxStepLength = other.maxStepLength;
         defaultStepWidth = other.defaultStepWidth;
         minStepWidth = other.minStepWidth;
         maxStepWidth = other.maxStepWidth;
         turnStepWidth = other.turnStepWidth;
         turnMaxAngleInward = other.turnMaxAngleInward;
         turnMaxAngleOutward = other.turnMaxAngleOutward;
      }

      public void setSwingHeight(double swingHeight)
      {
         this.swingHeight = swingHeight;
      }

      public void setSwingDuration(double swingDuration)
      {
         this.swingDuration = swingDuration;
      }

      public void setTransferDuration(double transferDuration)
      {
         this.transferDuration = transferDuration;
      }

      public void setMaxStepLength(double maxStepLength)
      {
         this.maxStepLength = maxStepLength;
      }

      public void setDefaultStepWidth(double defaultStepWidth)
      {
         this.defaultStepWidth = defaultStepWidth;
      }

      public void setMinStepWidth(double minStepWidth)
      {
         this.minStepWidth = minStepWidth;
      }

      public void setMaxStepWidth(double maxStepWidth)
      {
         this.maxStepWidth = maxStepWidth;
      }

      public void setTurnStepWidth(double turnStepWidth)
      {
         this.turnStepWidth = turnStepWidth;
      }

      public void setTurnMaxAngleInward(double turnMaxAngleInward)
      {
         this.turnMaxAngleInward = turnMaxAngleInward;
      }

      public void setTurnMaxAngleOutward(double turnMaxAngleOutward)
      {
         this.turnMaxAngleOutward = turnMaxAngleOutward;
      }

      public double getSwingHeight()
      {
         return swingHeight;
      }

      public double getSwingDuration()
      {
         return swingDuration;
      }

      public double getTransferDuration()
      {
         return transferDuration;
      }

      public double getMaxStepLength()
      {
         return maxStepLength;
      }

      public double getDefaultStepWidth()
      {
         return defaultStepWidth;
      }

      public double getMinStepWidth()
      {
         return minStepWidth;
      }

      public double getMaxStepWidth()
      {
         return maxStepWidth;
      }

      public double getTurnStepWidth()
      {
         return turnStepWidth;
      }

      public double getTurnMaxAngleInward()
      {
         return turnMaxAngleInward;
      }

      public double getTurnMaxAngleOutward()
      {
         return turnMaxAngleOutward;
      }

      public static JoystickStepParameters parseFromPropertyMap(Map<String, String> properties, JoystickStepParameters defaultParameters)
      {
         JoystickStepParameters parsed = new JoystickStepParameters();
         parsed.setSwingHeight(extractDoubleProperty(properties, "SwingHeight", defaultParameters.getSwingHeight()));
         parsed.setSwingDuration(extractDoubleProperty(properties, "SwingDuration", defaultParameters.getSwingDuration()));
         parsed.setTransferDuration(extractDoubleProperty(properties, "TransferDuration", defaultParameters.getTransferDuration()));
         parsed.setMaxStepLength(extractDoubleProperty(properties, "MaxStepLength", defaultParameters.getMaxStepLength()));
         parsed.setDefaultStepWidth(extractDoubleProperty(properties, "DefaultStepWidth", defaultParameters.getDefaultStepWidth()));
         parsed.setMinStepWidth(extractDoubleProperty(properties, "MinStepWidth", defaultParameters.getMinStepWidth()));
         parsed.setMaxStepWidth(extractDoubleProperty(properties, "MaxStepWidth", defaultParameters.getMaxStepWidth()));
         parsed.setTurnStepWidth(extractDoubleProperty(properties, "TurnStepWidth", defaultParameters.getTurnStepWidth()));
         parsed.setTurnMaxAngleInward(extractDoubleProperty(properties, "TurnMaxAngleInward", defaultParameters.getTurnMaxAngleInward()));
         parsed.setTurnMaxAngleOutward(extractDoubleProperty(properties, "TurnMaxAngleOutward", defaultParameters.getTurnMaxAngleOutward()));
         return parsed;
      }

      private static double extractDoubleProperty(Map<String, String> properties, String propertyName, double defaultValue)
      {
         return Double.parseDouble(properties.getOrDefault(propertyName, Double.toString(defaultValue)));
      }

      public static Map<String, String> exportToPropertyMap(JoystickStepParameters parametersToExport)
      {
         Map<String, String> properties = new HashMap<>();
         properties.put("SwingHeight", Double.toString(parametersToExport.getSwingHeight()));
         properties.put("SwingDuration", Double.toString(parametersToExport.getSwingDuration()));
         properties.put("TransferDuration", Double.toString(parametersToExport.getTransferDuration()));
         properties.put("MaxStepLength", Double.toString(parametersToExport.getMaxStepLength()));
         properties.put("DefaultStepWidth", Double.toString(parametersToExport.getDefaultStepWidth()));
         properties.put("MinStepWidth", Double.toString(parametersToExport.getMinStepWidth()));
         properties.put("MaxStepWidth", Double.toString(parametersToExport.getMaxStepWidth()));
         properties.put("TurnStepWidth", Double.toString(parametersToExport.getTurnStepWidth()));
         properties.put("TurnMaxAngleInward", Double.toString(parametersToExport.getTurnMaxAngleInward()));
         properties.put("TurnMaxAngleOutward", Double.toString(parametersToExport.getTurnMaxAngleOutward()));
         return properties;
      }

      @Override
      public String toString()
      {
         return "swing height: " + swingHeight + ", swing duration: " + swingDuration + ", transfer duration: " + transferDuration + ", max step length: "
               + maxStepLength + ", default step width: " + defaultStepWidth + ", min step width: " + minStepWidth + ", max step width: " + maxStepWidth
               + ", turn step width: " + turnStepWidth + ", turn max angle inward: " + turnMaxAngleInward + ", turn max angle outward: " + turnMaxAngleOutward;
      }
   }
}
