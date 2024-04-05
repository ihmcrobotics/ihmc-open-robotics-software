package us.ihmc.avatar.sakeGripper;

import java.util.ArrayList;
import java.util.List;

public class SakeHandErrorCodeInterpreter
{
   public static List<String> getErrorNames(int errorCode) {
      return getErrorList(errorCode).stream().map(SakeHandError::toString).toList();
   }

   public static List<SakeHandError> getErrorList(int errorCode)
   {
      List<SakeHandError> errorList = new ArrayList<>();

      for (SakeHandError error : SakeHandError.values()) {
         if ((errorCode & error.errorCode) != 0)
            errorList.add(error);
      }

      return errorList;
   }
}
