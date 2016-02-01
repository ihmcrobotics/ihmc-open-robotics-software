package us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects;


public class StackTraceRecorder<E>
{
   private final E object;
   private final StackTraceElement[] stackTrace;

   public StackTraceRecorder(E object)
   {
      this.object = object;
      this.stackTrace = Thread.currentThread().getStackTrace();
   }

   public String toString()
   {
      return getStackInformation(stackTrace) + ", " + object;
   }

   private static String getStackInformation(StackTraceElement[] stackTrace)
   {
      int stackIndexForCallIntoMomentumBasedController = 4;
      StackTraceElement stackTraceElement = stackTrace[stackIndexForCallIntoMomentumBasedController];
      String className = stackTraceElement.getClassName();

      int lastDotIndex = className.lastIndexOf('.');
      className = className.substring(lastDotIndex + 1);

      String methodName = stackTraceElement.getMethodName();
      int lineNumber = stackTrace[stackIndexForCallIntoMomentumBasedController].getLineNumber();

      return "+++ " + className + "." + methodName + "(): Line " + lineNumber;
   }
}
