package us.ihmc.simulationconstructionset.util.globalParameters;

import us.ihmc.simulationconstructionset.util.gui.GUIMessageFrame;

public class GlobalParameterGUIMessageFrame extends GUIMessageFrame
{

   public GlobalParameterGUIMessageFrame(String messageWindowName)
   {
      super(messageWindowName);
   }

   private static final int CHARACTERS_IN_VALUE_STRING = 12;

   public void globalParameterCreated(GlobalParameter globalParameter)
   {
      appendParameterMessage(globalParameter.getShortName() + "= " + padWithSpacesOrTrim(globalParameter.getValueInStringFormat(), CHARACTERS_IN_VALUE_STRING)
            + " created");
   }

   public void booleanValueChanged(GlobalParameter globalParameter, String comment, boolean previousValue, boolean newValue)
   {
      appendParameterMessage(globalParameter.getShortName() + " GlobalParameter changed from   "
            + padWithSpacesOrTrim(Boolean.toString(previousValue), CHARACTERS_IN_VALUE_STRING) + " to   "
            + padWithSpacesOrTrim(Boolean.toString(newValue), CHARACTERS_IN_VALUE_STRING) + " " + comment);
   }

   public void doubleValueChanged(GlobalParameter globalParameter, String comment, double previousValue, double newValue)
   {
      appendParameterMessage(globalParameter.getShortName() + " GlobalParameter changed from   "
            + padWithSpacesOrTrim(Double.toString(previousValue), CHARACTERS_IN_VALUE_STRING) + " to   "
            + padWithSpacesOrTrim(Double.toString(newValue), CHARACTERS_IN_VALUE_STRING) + " " + comment);
   }

   public void integerValueChanged(GlobalParameter globalParameter, String comment, int previousValue, int newValue)
   {
      appendParameterMessage(globalParameter.getShortName() + " GlobalParameter changed from   "
            + padWithSpacesOrTrim(Integer.toString(previousValue), CHARACTERS_IN_VALUE_STRING) + " to   "
            + padWithSpacesOrTrim(Integer.toString(newValue), CHARACTERS_IN_VALUE_STRING) + " " + comment);
   }

   public void enumValueChanged(GlobalParameter globalParameter, String comment, Enum<?> previousValue, Enum<?> newValue)
   {
      appendParameterMessage(globalParameter.getShortName() + " GlobalParameter changed from   "
            + padWithSpacesOrTrim(previousValue.toString(), CHARACTERS_IN_VALUE_STRING) + " to   "
            + padWithSpacesOrTrim(newValue.toString(), CHARACTERS_IN_VALUE_STRING) + " " + comment);
   }

   private String padWithSpacesOrTrim(String string, int sizeToBe)
   {
      if (string.length() == sizeToBe)
         return string;

      if (string.length() > sizeToBe)
         return string.substring(0, sizeToBe);

      StringBuffer padded = new StringBuffer(string);
      while (padded.length() < sizeToBe)
      {
         padded.append(" ");
      }

      return padded.toString();
   }

}
