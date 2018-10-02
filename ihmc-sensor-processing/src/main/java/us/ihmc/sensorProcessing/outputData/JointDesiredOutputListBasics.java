package us.ihmc.sensorProcessing.outputData;

public interface JointDesiredOutputListBasics extends JointDesiredOutputListReadOnly
{
   void clear();

   void overwriteWith(JointDesiredOutputListReadOnly other);

   void completeWith(JointDesiredOutputListReadOnly other);

   @Override
   JointDesiredOutputBasics getJointDesiredOutput(int index);
}
