package us.ihmc.tools.io;

import com.fasterxml.jackson.core.JsonGenerator;
import com.fasterxml.jackson.core.util.DefaultPrettyPrinter;

public class JSONLinePrettyPrinter extends DefaultPrettyPrinter
{
   @Override
   public void writeObjectFieldValueSeparator(JsonGenerator g) throws java.io.IOException
   {
      g.writeRaw(": ");
   }

   @Override
   public void writeArrayValueSeparator(JsonGenerator g) throws java.io.IOException
   {
      g.writeRaw(", ");
   }
}
