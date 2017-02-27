package us.ihmc.robotics.math;

public class Epsilons
{
   /** 1e0 */
   public static final double ONE = 1.0;
   /** 1e-1 */
   public static final double ONE_TENTH = 0.1;
   /** 1e-2 */
   public static final double ONE_HUNDREDTH = 0.01;
   /** 1e-3 */
   public static final double ONE_THOUSANDTH = 0.001;
   /** 1e-4 */
   public static final double ONE_TEN_THOUSANDTH = 0.0001;
   /** 1e-5 */
   public static final double ONE_HUNDRED_THOUSANDTH = 0.00001;
   /** 1e-6 */
   public static final double ONE_MILLIONTH = 0.000001;
   /** 1e-7 */
   public static final double ONE_TEN_MILLIONTH = 0.0000001;
   /** 1e-8 */
   public static final double ONE_HUNDRED_MILLIONTH = 0.00000001;
   /** 1e-9 */
   public static final double ONE_BILLIONTH = 0.000000001;
   /** 1e-10 */
   public static final double ONE_TEN_BILLIONTH = 0.0000000001;
   /** 1e-11 */
   public static final double ONE_HUNDRED_BILLIONTH = 0.00000000001;
   /** 1e-12 */
   public static final double ONE_TRILLIONTH = 0.000000000001;
   
   public static double tenths(int number)
   {
      return number * ONE_TENTH;
   }
   
   public static double hundredths(int number)
   {
      return number * ONE_HUNDREDTH;
   }
   
   public static double thousandths(int number)
   {
      return number * ONE_THOUSANDTH;
   }
   
   public static double tenThousandths(int number)
   {
      return number * ONE_TEN_THOUSANDTH;
   }
   
   public static double hundredThousandths(int number)
   {
      return number * ONE_HUNDRED_THOUSANDTH;
   }
   
   public static double millionths(int number)
   {
      return number * ONE_MILLIONTH;
   }
   
   public static double tenMillionths(int number)
   {
      return number * ONE_TEN_MILLIONTH;
   }
   
   public static double hundredMillionths(int number)
   {
      return number * ONE_HUNDRED_MILLIONTH;
   }
   
   public static double billionths(int number)
   {
      return number * ONE_BILLIONTH;
   }
   
   public static double tenBillionths(int number)
   {
      return number * ONE_TEN_BILLIONTH;
   }
   
   public static double hundredBillionths(int number)
   {
      return number * ONE_HUNDRED_BILLIONTH;
   }
   
   public static double trillionths(int number)
   {
      return number * ONE_TRILLIONTH;
   }
}
