package us.ihmc.javaFXToolkit;

import java.util.function.UnaryOperator;

import javafx.scene.control.TextField;
import javafx.scene.control.TextFormatter;
import javafx.scene.control.TextFormatter.Change;

/**
 * This class provides simple tools for setting up JavaFX UI control text formatter such as {@link TextField#setTextFormatter(TextFormatter)}. 
 * @author Sylvain Bertrand
 */
public class TextFormatterTools
{
   private static final String ipAddressRegex = makePartialIPRegex();

   /**
    * @return a TextFormatter that ensures proper formatting of an IP address.
    */
   public static TextFormatter<Change> ipAddressTextFormatter()
   {
      UnaryOperator<Change> ipAddressFilter = change -> change.getControlNewText().matches(ipAddressRegex) ? change : null;
      return new TextFormatter<>(ipAddressFilter);
   }

   private static String makePartialIPRegex()
   {
      String partialBlock = "(([01]?[0-9]{0,2})|(2[0-4][0-9])|(25[0-5]))";
      String subsequentPartialBlock = "(\\." + partialBlock + ")";
      String ipAddress = partialBlock + "?" + subsequentPartialBlock + "{0,3}";
      return "^" + ipAddress;
   }
}
