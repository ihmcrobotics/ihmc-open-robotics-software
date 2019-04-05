package us.ihmc.avatar.joystickBasedJavaFXController;

import java.util.Scanner;

import javafx.beans.property.Property;
import us.ihmc.avatar.joystickBasedJavaFXController.JoystickStepParametersProperty.JoystickStepParameters;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.jOctoMap.tools.ScannerTools;
import us.ihmc.robotEnvironmentAwareness.ui.properties.ParametersProperty;

public class JoystickStepParametersProperty extends ParametersProperty<JoystickStepParameters>
{
   private final DoubleField swingHeight = new DoubleField(JoystickStepParameters::getSwingHeight, JoystickStepParameters::setSwingHeight);
   private final DoubleField swingDuration = new DoubleField(JoystickStepParameters::getSwingDuration, JoystickStepParameters::setSwingDuration);
   private final DoubleField transferDuration = new DoubleField(JoystickStepParameters::getTransferDuration, JoystickStepParameters::setTransferDuration);
   private final DoubleField defaultStepLength = new DoubleField(JoystickStepParameters::getDefaultStepLength, JoystickStepParameters::setDefaultStepLength);
   private final DoubleField maxStepLength = new DoubleField(JoystickStepParameters::getMaxStepLength, JoystickStepParameters::setMaxStepLength);
   private final DoubleField defaultStepWidth = new DoubleField(JoystickStepParameters::getDefaultStepWidth, JoystickStepParameters::setDefaultStepWidth);
   private final DoubleField minStepWidth = new DoubleField(JoystickStepParameters::getMinStepWidth, JoystickStepParameters::setMinStepWidth);
   private final DoubleField maxStepWidth = new DoubleField(JoystickStepParameters::getMaxStepWidth, JoystickStepParameters::setMaxStepWidth);
   private final DoubleField turnStepWidth = new DoubleField(JoystickStepParameters::getTurnStepWidth, JoystickStepParameters::setTurnStepWidth);
   private final DoubleField turnMaxAngleInwards = new DoubleField(JoystickStepParameters::getTurnMaxAngleInward,
                                                                   JoystickStepParameters::setTurnMaxAngleInward);
   private final DoubleField turnMaxAngleOutwards = new DoubleField(JoystickStepParameters::getTurnMaxAngleOutward,
                                                                    JoystickStepParameters::setTurnMaxAngleOutward);

   public JoystickStepParametersProperty(Object bean, String name)
   {
      super(bean, name, new JoystickStepParameters());
   }

   public void bindBidirectionalSwingHeightThreshold(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, swingHeight);
   }

   public void bindBidirectionalSwingDurationThreshold(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, swingDuration);
   }

   public void bindBidirectionalTransferDurationThreshold(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, transferDuration);
   }

   public void bindBidirectionalDefaultStepLengthThreshold(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, defaultStepLength);
   }

   public void bindBidirectionalMaxStepLengthThreshold(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, maxStepLength);
   }

   public void bindBidirectionalDefaultStepWidthThreshold(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, defaultStepWidth);
   }

   public void bindBidirectionalMinStepWidthThreshold(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, minStepWidth);
   }

   public void bindBidirectionalMaxStepWidthThreshold(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, maxStepWidth);
   }

   public void bindBidirectionalTurnStepWidthThreshold(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, turnStepWidth);
   }

   public void bindBidirectionalTurnMaxAngleInwardsThreshold(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, turnMaxAngleInwards);
   }

   public void bindBidirectionalTurnMaxAngleOutwardsThreshold(Property<? extends Number> property)
   {
      bindFieldBidirectionalToNumberProperty(property, turnMaxAngleOutwards);
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
      private double defaultStepLength, maxStepLength;
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
         defaultStepLength = steppingParameters.getDefaultStepLength();
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
         defaultStepLength = other.defaultStepLength;
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

      public void setDefaultStepLength(double defaultStepLength)
      {
         this.defaultStepLength = defaultStepLength;
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

      public double getDefaultStepLength()
      {
         return defaultStepLength;
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

      public static JoystickStepParameters parse(String parametersAsString)
      {
         parametersAsString = parametersAsString.replace(",", "");
         Scanner scanner = new Scanner(parametersAsString);
         JoystickStepParameters parameters = new JoystickStepParameters();
         parameters.setSwingHeight(ScannerTools.readNextDouble(scanner, parameters.getSwingHeight()));
         parameters.setSwingDuration(ScannerTools.readNextDouble(scanner, parameters.getSwingDuration()));
         parameters.setTransferDuration(ScannerTools.readNextDouble(scanner, parameters.getTransferDuration()));
         parameters.setDefaultStepLength(ScannerTools.readNextDouble(scanner, parameters.getDefaultStepLength()));
         parameters.setMaxStepLength(ScannerTools.readNextDouble(scanner, parameters.getMaxStepLength()));
         parameters.setDefaultStepWidth(ScannerTools.readNextDouble(scanner, parameters.getDefaultStepWidth()));
         parameters.setMinStepWidth(ScannerTools.readNextDouble(scanner, parameters.getMinStepWidth()));
         parameters.setMaxStepWidth(ScannerTools.readNextDouble(scanner, parameters.getMaxStepWidth()));
         parameters.setTurnStepWidth(ScannerTools.readNextDouble(scanner, parameters.getTurnStepWidth()));
         parameters.setTurnMaxAngleInward(ScannerTools.readNextDouble(scanner, parameters.getTurnMaxAngleInward()));
         parameters.setTurnMaxAngleOutward(ScannerTools.readNextDouble(scanner, parameters.getTurnMaxAngleOutward()));
         scanner.close();
         return parameters;
      }

      @Override
      public String toString()
      {
         return "swing height: " + swingHeight + ", swing duration: " + swingDuration + ", transfer duration: " + transferDuration + ", default step length: "
               + defaultStepLength + ", max step length: " + maxStepLength + "default step width: " + defaultStepWidth + ", min step width: " + minStepWidth
               + ", max step width: " + maxStepWidth + ", turn step width: " + turnStepWidth + ", turn max angle inward: " + turnMaxAngleInward
               + ", turn max angle outward: " + turnMaxAngleOutward;
      }
   }
}
